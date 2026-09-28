#!/usr/bin/env python3
"""G80 RAW Golden Dataset recorder for openpilot/sunnypilot.

Purpose
-------
Capture the *inputs* needed to replay the G80 radar perception stack offline,
without depending on the current decoder/fusion/Kalman/IMM implementation.

Recorded cereal services (exact serialized Event bytes):
  1) can         - ALL raw CAN frames/buses delivered on the openpilot CAN service
  2) modelV2     - C4 model/path/lane/road-edge/leads input
  3) carState    - vEgo/aEgo/steering/blinker/brake/cruise state, etc.
  4) radarState  - stock/production radarState reference (diagnostic only)

Output root:
  /data/radar/golden/<session>/

The recorder writes 60-second chunk files. A chunk is first written with a
'.part' suffix and atomically renamed when closed. Abrupt power loss therefore
normally affects only the active chunk.

Custom stream format (after decompression):
  magic: b'G80RAW1\\0'
  uint32 little-endian header_json_length
  header_json UTF-8 bytes
  repeated records:
    uint8  service_id
    uint8  flags (0)
    uint16 reserved (0)
    uint64 recorder_recv_monotonic_ns
    uint64 event_logMonoTime_ns
    uint32 payload_length
    uint32 payload_crc32
    payload_length bytes = exact serialized cereal Event

Enable marker:
  /data/radar/ENABLE_G80_RAW_GOLDEN

Environment overrides:
  G80_RAW_ROOT=/data/radar/golden
  G80_RAW_CHUNK_SEC=60
  G80_RAW_CODEC=auto        # auto | zstd | gzip | none
  G80_RAW_MIN_FREE_GB=10
  G80_RAW_MAX_HOURS=3
  G80_RAW_QUEUE=16384

This process is RECEIVE-ONLY. It never publishes cereal messages and never
transmits CAN.
"""

from __future__ import annotations

import argparse
import gzip
import json
import os
import platform
import queue
import shutil
import signal
import struct
import sys
import threading
import time
import zlib
from datetime import datetime, timezone
from pathlib import Path
from typing import BinaryIO, Optional

FORMAT_VERSION = 1
MAGIC = b"G80RAW1\0"
RECORD_HEADER = struct.Struct("<BBHQQII")
UINT32 = struct.Struct("<I")

SERVICE_IDS = {
  "can": 1,
  "modelV2": 2,
  "carState": 3,
  "radarState": 4,
}
ID_SERVICES = {v: k for k, v in SERVICE_IDS.items()}

DEFAULT_ROOT = "/data/radar/golden"
DEFAULT_ENABLE_MARKER = "/data/radar/ENABLE_G80_RAW_GOLDEN"


def _utc_now() -> str:
  return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def _local_stamp() -> str:
  return datetime.now().strftime("%Y-%m-%d_%H-%M-%S")


def _atomic_json(path: Path, obj: dict) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  tmp = path.with_suffix(path.suffix + ".tmp")
  tmp.write_text(json.dumps(obj, ensure_ascii=False, indent=2, sort_keys=True), encoding="utf-8")
  os.replace(tmp, path)


def _event_to_bytes(msg) -> bytes:
  """Return exact serialized capnp Event bytes for Reader or Builder objects."""
  try:
    return bytes(msg.to_bytes())
  except Exception:
    try:
      return bytes(msg.as_builder().to_bytes())
    except Exception as e:
      raise RuntimeError(f"cannot serialize cereal Event: {e}") from e


def _build_versions() -> dict:
  out = {}
  try:
    from openpilot.selfdrive.g80_radar.build_info import BUILD_VERSION, BUILD_TAG
    out["g80_build_version"] = int(BUILD_VERSION)
    out["g80_build_tag"] = str(BUILD_TAG)
  except Exception:
    pass
  return out


class _CompressedOutput:
  def __init__(self, path_base: Path, codec: str):
    self.codec_requested = codec
    self.codec = codec
    self.raw: Optional[BinaryIO] = None
    self.stream = None
    self.path_part: Path
    self.path_final: Path

    if codec == "auto":
      try:
        import zstandard  # noqa: F401
        self.codec = "zstd"
      except Exception:
        self.codec = "gzip"

    if self.codec not in ("zstd", "gzip", "none"):
      raise ValueError(f"invalid codec: {self.codec}")

    ext = {"zstd": ".g80raw.zst", "gzip": ".g80raw.gz", "none": ".g80raw"}[self.codec]
    self.path_final = Path(str(path_base) + ext)
    self.path_part = Path(str(self.path_final) + ".part")
    self.raw = open(self.path_part, "wb", buffering=1024 * 1024)

    if self.codec == "zstd":
      import zstandard as zstd
      cctx = zstd.ZstdCompressor(level=1, threads=0, write_checksum=True)
      self.stream = cctx.stream_writer(self.raw, closefd=False)
    elif self.codec == "gzip":
      self.stream = gzip.GzipFile(fileobj=self.raw, mode="wb", compresslevel=1, mtime=0)
    else:
      self.stream = self.raw

  def write(self, data: bytes) -> None:
    self.stream.write(data)

  def compressed_size(self) -> int:
    try:
      return int(self.raw.tell()) if self.raw is not None else 0
    except Exception:
      return 0

  def close(self, finalize: bool = True) -> None:
    if self.stream is None:
      return
    try:
      if self.codec == "zstd":
        try:
          import zstandard as zstd
          self.stream.flush(zstd.FLUSH_FRAME)
        except Exception:
          pass
        self.stream.close()
      elif self.codec == "gzip":
        self.stream.close()
      else:
        self.stream.flush()
    finally:
      if self.raw is not None:
        try:
          self.raw.flush()
          os.fsync(self.raw.fileno())
        except Exception:
          pass
        try:
          self.raw.close()
        except Exception:
          pass
      self.stream = None
      self.raw = None

    if finalize and self.path_part.exists():
      os.replace(self.path_part, self.path_final)


class GoldenWriter:
  def __init__(self, session_dir: Path, chunk_sec: float, codec: str, queue_size: int):
    self.session_dir = session_dir
    self.chunk_ns = max(5, int(chunk_sec)) * 1_000_000_000
    self.codec = codec
    self.q: queue.Queue = queue.Queue(maxsize=max(1024, queue_size))
    self.stop_event = threading.Event()
    self.thread = threading.Thread(target=self._run, name="g80_raw_writer", daemon=True)
    self.chunk_index = 0
    self.chunk_start_ns = 0
    self.out: Optional[_CompressedOutput] = None
    self.stats_lock = threading.Lock()
    self.stats = {
      "records": 0,
      "records_by_service": {k: 0 for k in SERVICE_IDS},
      "payload_bytes": 0,
      "compressed_bytes_closed": 0,
      "chunks_closed": 0,
      "queue_full_drops": 0,
      "serialization_errors": 0,
      "write_errors": 0,
      "last_record_recv_ns": 0,
      "active_chunk": None,
      "codec": codec,
    }

  def start(self) -> None:
    self.thread.start()

  def submit(self, service: str, recv_ns: int, log_mono_ns: int, payload: bytes) -> bool:
    item = (SERVICE_IDS[service], recv_ns, log_mono_ns, payload)
    try:
      self.q.put(item, timeout=0.75)
      return True
    except queue.Full:
      with self.stats_lock:
        self.stats["queue_full_drops"] += 1
      return False

  def note_serialization_error(self) -> None:
    with self.stats_lock:
      self.stats["serialization_errors"] += 1

  def snapshot(self) -> dict:
    with self.stats_lock:
      d = json.loads(json.dumps(self.stats))
    d["queue_depth"] = self.q.qsize()
    d["queue_capacity"] = self.q.maxsize
    if self.out is not None:
      d["active_chunk_compressed_bytes"] = self.out.compressed_size()
      d["codec_active"] = self.out.codec
    return d

  def stop(self) -> None:
    self.stop_event.set()
    self.thread.join(timeout=15.0)

  def _open_chunk(self, recv_ns: int) -> None:
    self.chunk_start_ns = recv_ns
    base = self.session_dir / f"chunk_{self.chunk_index:06d}"
    self.out = _CompressedOutput(base, self.codec)
    header = {
      "format": "G80_RAW_GOLDEN",
      "format_version": FORMAT_VERSION,
      "magic": "G80RAW1",
      "chunk_index": self.chunk_index,
      "created_utc": _utc_now(),
      "chunk_start_recv_monotonic_ns": recv_ns,
      "service_ids": SERVICE_IDS,
      "record_header": "<BBHQQII",
      "record_header_fields": ["service_id", "flags", "reserved", "recv_ns", "logMonoTime_ns", "payload_len", "crc32"],
      "payload": "exact serialized openpilot cereal Event",
      "codec": self.out.codec,
    }
    hb = json.dumps(header, separators=(",", ":"), sort_keys=True).encode("utf-8")
    self.out.write(MAGIC)
    self.out.write(UINT32.pack(len(hb)))
    self.out.write(hb)
    with self.stats_lock:
      self.stats["active_chunk"] = self.out.path_final.name
      self.stats["codec"] = self.out.codec

  def _close_chunk(self) -> None:
    if self.out is None:
      return
    size = self.out.compressed_size()
    self.out.close(finalize=True)
    with self.stats_lock:
      self.stats["compressed_bytes_closed"] += size
      self.stats["chunks_closed"] += 1
      self.stats["active_chunk"] = None
    self.out = None
    self.chunk_index += 1

  def _write_item(self, item) -> None:
    service_id, recv_ns, log_mono_ns, payload = item
    if self.out is None:
      self._open_chunk(recv_ns)
    elif recv_ns - self.chunk_start_ns >= self.chunk_ns:
      self._close_chunk()
      self._open_chunk(recv_ns)

    crc = zlib.crc32(payload) & 0xFFFFFFFF
    rec = RECORD_HEADER.pack(service_id, 0, 0, recv_ns, log_mono_ns, len(payload), crc)
    self.out.write(rec)
    self.out.write(payload)

    service = ID_SERVICES.get(service_id, str(service_id))
    with self.stats_lock:
      self.stats["records"] += 1
      self.stats["records_by_service"][service] = self.stats["records_by_service"].get(service, 0) + 1
      self.stats["payload_bytes"] += len(payload)
      self.stats["last_record_recv_ns"] = recv_ns

  def _run(self) -> None:
    while not self.stop_event.is_set() or not self.q.empty():
      try:
        item = self.q.get(timeout=0.25)
      except queue.Empty:
        continue
      try:
        self._write_item(item)
      except Exception:
        with self.stats_lock:
          self.stats["write_errors"] += 1
      finally:
        self.q.task_done()
    try:
      self._close_chunk()
    except Exception:
      with self.stats_lock:
        self.stats["write_errors"] += 1


def _session_metadata(session_dir: Path, codec: str, chunk_sec: int) -> dict:
  return {
    "format": "G80_RAW_GOLDEN",
    "format_version": FORMAT_VERSION,
    "session": session_dir.name,
    "created_utc": _utc_now(),
    "created_local": datetime.now().astimezone().isoformat(timespec="seconds"),
    "host": platform.node(),
    "python": sys.version,
    "services": list(SERVICE_IDS.keys()),
    "service_ids": SERVICE_IDS,
    "codec_requested": codec,
    "chunk_sec": chunk_sec,
    "receive_only": True,
    "can_tx": False,
    **_build_versions(),
  }


def run_recorder() -> int:
  # Keep this recorder below perception/UI priority. Writer runs in a separate thread.
  try:
    os.nice(10)
  except Exception:
    pass

  from openpilot.cereal import messaging

  root = Path(os.getenv("G80_RAW_ROOT", DEFAULT_ROOT))
  root.mkdir(parents=True, exist_ok=True)
  chunk_sec = max(10, int(float(os.getenv("G80_RAW_CHUNK_SEC", "60"))))
  codec = os.getenv("G80_RAW_CODEC", "auto").strip().lower()
  min_free_gb = max(1.0, float(os.getenv("G80_RAW_MIN_FREE_GB", "10")))
  max_hours = max(0.25, float(os.getenv("G80_RAW_MAX_HOURS", "3")))
  queue_size = max(1024, int(os.getenv("G80_RAW_QUEUE", "16384")))

  session_dir = root / _local_stamp()
  if session_dir.exists():
    session_dir = root / f"{_local_stamp()}_{os.getpid()}"
  session_dir.mkdir(parents=True, exist_ok=False)
  _atomic_json(session_dir / "metadata.json", _session_metadata(session_dir, codec, chunk_sec))
  (root / "CURRENT_SESSION.txt").write_text(str(session_dir) + "\n", encoding="utf-8")

  writer = GoldenWriter(session_dir, chunk_sec, codec, queue_size)
  writer.start()

  stop = threading.Event()
  stop_reason = {"reason": "normal"}

  def request_stop(signum=None, frame=None):
    stop_reason["reason"] = f"signal_{signum}" if signum is not None else "requested"
    stop.set()

  signal.signal(signal.SIGTERM, request_stop)
  signal.signal(signal.SIGINT, request_stop)

  # Exact inputs used by g80_radar/live_service. CAN must never conflate.
  sockets = {
    "can": messaging.sub_sock("can", timeout=0, conflate=False),
    "modelV2": messaging.sub_sock("modelV2", timeout=0, conflate=False),
    "carState": messaging.sub_sock("carState", timeout=0, conflate=False),
    "radarState": messaging.sub_sock("radarState", timeout=0, conflate=False),
  }
  drain_limits = {"can": 800, "modelV2": 80, "carState": 300, "radarState": 80}

  start_mono = time.monotonic()
  start_ns = time.monotonic_ns()
  next_status = 0.0
  last_status = {}

  try:
    while not stop.is_set():
      now = time.monotonic()
      if now - start_mono >= max_hours * 3600.0:
        stop_reason["reason"] = "max_hours"
        break

      got = False
      for service, sock in sockets.items():
        for _ in range(drain_limits[service]):
          msg = messaging.recv_one_or_none(sock)
          if msg is None:
            break
          got = True
          recv_ns = time.monotonic_ns()
          try:
            payload = _event_to_bytes(msg)
            log_mono_ns = int(msg.logMonoTime)
          except Exception:
            writer.note_serialization_error()
            continue
          writer.submit(service, recv_ns, log_mono_ns, payload)

      if now >= next_status:
        usage = shutil.disk_usage(root)
        free_gb = usage.free / (1024 ** 3)
        stats = writer.snapshot()
        last_status = {
          "session": str(session_dir),
          "running": True,
          "start_monotonic_ns": start_ns,
          "elapsed_sec": round(now - start_mono, 1),
          "free_gb": round(free_gb, 2),
          "min_free_gb": min_free_gb,
          **stats,
        }
        _atomic_json(session_dir / "status.json", last_status)
        _atomic_json(root / "STATUS.json", last_status)
        next_status = now + 2.0
        if free_gb < min_free_gb:
          stop_reason["reason"] = "low_disk_space"
          break

      if not got:
        time.sleep(0.001)
  finally:
    writer.stop()
    now = time.monotonic()
    usage = shutil.disk_usage(root)
    final = {
      **last_status,
      **writer.snapshot(),
      "running": False,
      "ended_utc": _utc_now(),
      "elapsed_sec": round(now - start_mono, 1),
      "free_gb": round(usage.free / (1024 ** 3), 2),
      "stop_reason": stop_reason["reason"],
    }
    _atomic_json(session_dir / "status.json", final)
    _atomic_json(root / "STATUS.json", final)
    (root / "LAST_SESSION.txt").write_text(str(session_dir) + "\n", encoding="utf-8")
    try:
      (root / "CURRENT_SESSION.txt").unlink()
    except FileNotFoundError:
      pass
  return 0


def _open_decompressed(path: Path):
  if path.suffix == ".gz":
    return gzip.open(path, "rb")
  if path.suffix == ".zst":
    import zstandard as zstd
    raw = open(path, "rb")
    reader = zstd.ZstdDecompressor().stream_reader(raw)
    return reader
  return open(path, "rb")


def run_self_test(out_root: Path) -> int:
  session = out_root / "selftest"
  if session.exists():
    shutil.rmtree(session)
  session.mkdir(parents=True)
  w = GoldenWriter(session, chunk_sec=10, codec="gzip", queue_size=1024)
  w.start()
  base_ns = time.monotonic_ns()
  expected = []
  for i in range(20):
    service = list(SERVICE_IDS)[i % len(SERVICE_IDS)]
    payload = (f"fake-event-{service}-{i}" * (i + 1)).encode()
    expected.append((SERVICE_IDS[service], base_ns + i * 1_000_000, 1000 + i, payload))
    assert w.submit(service, base_ns + i * 1_000_000, 1000 + i, payload)
  w.stop()
  files = sorted(session.glob("chunk_*.g80raw.gz"))
  assert len(files) == 1, files
  with _open_decompressed(files[0]) as f:
    assert f.read(len(MAGIC)) == MAGIC
    hlen = UINT32.unpack(f.read(4))[0]
    json.loads(f.read(hlen).decode())
    got = []
    while True:
      h = f.read(RECORD_HEADER.size)
      if not h:
        break
      assert len(h) == RECORD_HEADER.size
      sid, flags, reserved, recv_ns, log_ns, plen, crc = RECORD_HEADER.unpack(h)
      payload = f.read(plen)
      assert len(payload) == plen
      assert (zlib.crc32(payload) & 0xFFFFFFFF) == crc
      got.append((sid, recv_ns, log_ns, payload))
  assert got == expected
  print(json.dumps({"self_test": "PASS", "records": len(got), "file": str(files[0])}, indent=2))
  return 0


def main() -> int:
  ap = argparse.ArgumentParser()
  ap.add_argument("--self-test", metavar="DIR", help="run format/writer self-test without openpilot")
  args = ap.parse_args()
  if args.self_test:
    return run_self_test(Path(args.self_test))
  return run_recorder()


if __name__ == "__main__":
  raise SystemExit(main())

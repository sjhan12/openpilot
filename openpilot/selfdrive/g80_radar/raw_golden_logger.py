#!/usr/bin/env python3
"""G80 RAW Golden automatic event recorder for openpilot/sunnypilot.

V3 AUTO EVENT design
--------------------
- Process is resident automatically while the car is onroad (manager condition).
- It continuously keeps only the most recent RAW input records in a RAM ring buffer.
- When a useful driving event is detected, it persists:
    PRE seconds before trigger + POST seconds after the last trigger.
- Re-triggers during an active event extend the same event instead of creating duplicates.
- Exact serialized cereal Event bytes are preserved in the existing G80RAW1 stream format.
- Single-instance flock prevents duplicate real recorders.
- RECEIVE ONLY: no cereal publish, no CAN TX.

Recorded cereal services:
  can, modelV2, carState, radarState

Automatic trigger sources (default):
  - left/right blinker rising edge
  - hard ego deceleration
  - radarState lead acquire/lost/swap-like jump
  - short front TTC
  - fast rear approach from already-decoded G80 corner object fields (when decoder import is available)
  - optional fresh /dev/shm/g80_radar.json diagnostics (KF/IMM reset, stable incoming,
    CAUTION/BLOCKED decision) when such a state file is enabled by the radar service
  - optional manual marker /data/radar/FORCE_G80_RAW_EVENT (removed after trigger)

Compatibility / filenames:
  raw_golden_logger.py      (same filename)
  manager process name      g80rawlogger (same name)
  /data/radar/golden/STATUS.json (same status location)

Default event output:
  /data/radar/golden_events/<event_session>/

Environment overrides:
  G80_RAW_PRE_SEC=30
  G80_RAW_POST_SEC=45
  G80_RAW_MAX_EVENT_SEC=180
  G80_RAW_RING_MAX_MB=96
  G80_RAW_EVENT_ROOT=/data/radar/golden_events
  G80_RAW_ROOT=/data/radar/golden
  G80_RAW_CODEC=auto
  G80_RAW_CHUNK_SEC=60
  G80_RAW_QUEUE=32768
  G80_RAW_EVENT_MIN_FREE_GB=4
  G80_RAW_AEGO_TRIGGER=-2.5
  G80_RAW_FRONT_TTC_TRIGGER=4.0
  G80_RAW_REAR_TTC_TRIGGER=4.5
  G80_RAW_REAR_CLOSING_MPS=5.0
  G80_RAW_STATE_PATH=/dev/shm/g80_radar.json

Disable automatic resident logger before driving:
  touch /data/radar/DISABLE_G80_RAW_GOLDEN
Remove that file to re-enable on the next manager evaluation/onroad start.
"""

from __future__ import annotations

import argparse
import collections
import fcntl
import gzip
import json
import math
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
RECORDER_VERSION = 3
MAGIC = b"G80RAW1\0"
RECORD_HEADER = struct.Struct("<BBHQQII")
UINT32 = struct.Struct("<I")

SERVICE_IDS = {"can": 1, "modelV2": 2, "carState": 3, "radarState": 4}
ID_SERVICES = {v: k for k, v in SERVICE_IDS.items()}

DEFAULT_ROOT = "/data/radar/golden"
DEFAULT_EVENT_ROOT = "/data/radar/golden_events"
DEFAULT_FORCE_EVENT = "/data/radar/FORCE_G80_RAW_EVENT"
DEFAULT_STATE_PATH = "/dev/shm/g80_radar.json"


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


def _safe_float(v, default=None):
  try:
    x = float(v)
    return x if math.isfinite(x) else default
  except Exception:
    return default


def _sanitize_reason(s: str) -> str:
  out = "".join(ch if ch.isalnum() or ch in "_-" else "_" for ch in str(s).upper())
  return out[:40] or "EVENT"


class _CompressedOutput:
  def __init__(self, path_base: Path, codec: str):
    self.codec_requested = codec
    self.codec = codec
    self.raw: Optional[BinaryIO] = None
    self.stream = None
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
          self.raw.flush(); os.fsync(self.raw.fileno())
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
  """Asynchronous writer for the backward-compatible G80RAW1 chunk stream."""
  def __init__(self, session_dir: Path, chunk_sec: float, codec: str, queue_size: int):
    self.session_dir = session_dir
    self.chunk_ns = max(5, int(chunk_sec)) * 1_000_000_000
    self.codec = codec
    self.q: queue.Queue = queue.Queue(maxsize=max(1024, queue_size))
    self.stop_event = threading.Event()
    self.thread = threading.Thread(target=self._run, name="g80_raw_event_writer", daemon=True)
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
    try:
      self.q.put((SERVICE_IDS[service], recv_ns, log_mono_ns, payload), timeout=0.20)
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
    self.thread.join(timeout=20.0)

  def _open_chunk(self, recv_ns: int) -> None:
    self.chunk_start_ns = recv_ns
    base = self.session_dir / f"chunk_{self.chunk_index:06d}"
    self.out = _CompressedOutput(base, self.codec)
    header = {
      "format": "G80_RAW_GOLDEN",
      "format_version": FORMAT_VERSION,
      "recorder_version": RECORDER_VERSION,
      "magic": "G80RAW1",
      "chunk_index": self.chunk_index,
      "created_utc": _utc_now(),
      "chunk_start_recv_monotonic_ns": recv_ns,
      "service_ids": SERVICE_IDS,
      "record_header": "<BBHQQII",
      "record_header_fields": ["service_id", "flags", "reserved", "recv_ns", "logMonoTime_ns", "payload_len", "crc32"],
      "payload": "exact serialized openpilot cereal Event",
      "codec": self.out.codec,
      "capture_mode": "AUTO_EVENT",
    }
    hb = json.dumps(header, separators=(",", ":"), sort_keys=True).encode("utf-8")
    self.out.write(MAGIC); self.out.write(UINT32.pack(len(hb))); self.out.write(hb)
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
      self._close_chunk(); self._open_chunk(recv_ns)
    crc = zlib.crc32(payload) & 0xFFFFFFFF
    self.out.write(RECORD_HEADER.pack(service_id, 0, 0, recv_ns, log_mono_ns, len(payload), crc))
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


class RollingBuffer:
  def __init__(self, pre_sec: float, max_bytes: int):
    self.pre_ns = int(max(5.0, pre_sec) * 1e9)
    self.max_bytes = max(8 * 1024 * 1024, int(max_bytes))
    self.items = collections.deque()
    self.bytes = 0

  @staticmethod
  def _size(item) -> int:
    return len(item[3]) + 64

  def append(self, item) -> None:
    self.items.append(item)
    self.bytes += self._size(item)
    newest_ns = item[1]
    cutoff = newest_ns - self.pre_ns
    while self.items and (self.items[0][1] < cutoff or self.bytes > self.max_bytes):
      old = self.items.popleft(); self.bytes -= self._size(old)

  def snapshot(self):
    return list(self.items)

  def age_sec(self) -> float:
    if len(self.items) < 2:
      return 0.0
    return max(0.0, (self.items[-1][1] - self.items[0][1]) / 1e9)


class EventSession:
  def __init__(self, root: Path, pre_items, trigger_reasons, trigger_ns: int,
               post_sec: float, max_event_sec: float, chunk_sec: int, codec: str, queue_size: int):
    stamp = _local_stamp()
    tag = "_".join(_sanitize_reason(r) for r in trigger_reasons[:3])
    self.path = root / f"{stamp}_{tag}"
    if self.path.exists():
      self.path = root / f"{stamp}_{tag}_{os.getpid()}"
    self.path.mkdir(parents=True, exist_ok=False)
    self.writer = GoldenWriter(self.path, chunk_sec, codec, queue_size)
    self.writer.start()
    self.trigger_ns = trigger_ns
    self.start_ns = pre_items[0][1] if pre_items else trigger_ns
    self.max_end_ns = trigger_ns + int(max_event_sec * 1e9)
    self.deadline_ns = min(self.max_end_ns, trigger_ns + int(post_sec * 1e9))
    self.reasons = []
    self.trigger_log = []
    self.pre_records = len(pre_items)
    self.closed = False
    for r in trigger_reasons:
      self.extend(r, trigger_ns, post_sec)
    for service, recv_ns, log_ns, payload in pre_items:
      self.writer.submit(service, recv_ns, log_ns, payload)
    self._write_metadata(active=True)

  def extend(self, reason: str, now_ns: int, post_sec: float) -> None:
    reason = _sanitize_reason(reason)
    if reason not in self.reasons:
      self.reasons.append(reason)
    self.trigger_log.append({"reason": reason, "mono_ns": int(now_ns), "wall_time": datetime.now().astimezone().isoformat(timespec="milliseconds")})
    self.deadline_ns = min(self.max_end_ns, max(self.deadline_ns, now_ns + int(post_sec * 1e9)))

  def submit(self, item) -> bool:
    return self.writer.submit(*item)

  def due(self, now_ns: int) -> bool:
    return now_ns >= self.deadline_ns or now_ns >= self.max_end_ns

  def _write_metadata(self, active: bool) -> None:
    obj = {
      "format": "G80_RAW_GOLDEN_AUTO_EVENT",
      "format_version": FORMAT_VERSION,
      "recorder_version": RECORDER_VERSION,
      "capture_mode": "AUTO_EVENT",
      "receive_only": True,
      "can_tx": False,
      "active": bool(active),
      "event_dir": str(self.path),
      "event_start_monotonic_ns": int(self.start_ns),
      "first_trigger_monotonic_ns": int(self.trigger_ns),
      "deadline_monotonic_ns": int(self.deadline_ns),
      "pre_records": int(self.pre_records),
      "reasons": list(self.reasons),
      "triggers": list(self.trigger_log),
      "writer": self.writer.snapshot(),
      **_build_versions(),
    }
    _atomic_json(self.path / "event.json", obj)

  def close(self) -> dict:
    if self.closed:
      return self.writer.snapshot()
    self.writer.stop()
    self.closed = True
    self._write_metadata(active=False)
    return self.writer.snapshot()


class TriggerDetector:
  def __init__(self):
    self.prev_left = False
    self.prev_right = False
    self.prev_hard_brake = False
    self.prev_lead = None
    self.last_reason_ns = {}
    self.prev_state_sig = {}
    self.last_corner_probe_ns = 0
    self.aego_trigger = float(os.getenv("G80_RAW_AEGO_TRIGGER", "-2.5"))
    self.front_ttc_trigger = max(1.0, float(os.getenv("G80_RAW_FRONT_TTC_TRIGGER", "4.0")))
    self.rear_ttc_trigger = max(1.0, float(os.getenv("G80_RAW_REAR_TTC_TRIGGER", "4.5")))
    self.rear_closing_mps = max(2.0, float(os.getenv("G80_RAW_REAR_CLOSING_MPS", "5.0")))
    self.cooldown_ns = int(max(1.0, float(os.getenv("G80_RAW_TRIGGER_COOLDOWN_SEC", "5.0"))) * 1e9)

  def _allow(self, reason: str, now_ns: int) -> bool:
    last = self.last_reason_ns.get(reason, 0)
    if now_ns - last < self.cooldown_ns:
      return False
    self.last_reason_ns[reason] = now_ns
    return True

  def carstate(self, msg, now_ns: int):
    reasons = []
    try:
      cs = msg.carState
      left = bool(cs.leftBlinker); right = bool(cs.rightBlinker)
      aego = _safe_float(cs.aEgo, 0.0) or 0.0
      if left and not self.prev_left and self._allow("BLINKER_LEFT", now_ns): reasons.append("BLINKER_LEFT")
      if right and not self.prev_right and self._allow("BLINKER_RIGHT", now_ns): reasons.append("BLINKER_RIGHT")
      hard = aego <= self.aego_trigger
      if hard and not self.prev_hard_brake and self._allow("HARD_BRAKE", now_ns): reasons.append("HARD_BRAKE")
      self.prev_left, self.prev_right, self.prev_hard_brake = left, right, hard
    except Exception:
      pass
    return reasons

  def radarstate(self, msg, now_ns: int):
    reasons = []
    try:
      lead = msg.radarState.leadOne
      status = bool(lead.status)
      cur = {
        "status": status,
        "dRel": _safe_float(lead.dRel),
        "yRel": _safe_float(lead.yRel),
        "vRel": _safe_float(lead.vRel),
        "ns": now_ns,
      }
      prev = self.prev_lead
      if prev is not None:
        if status and not prev["status"] and self._allow("LEAD_ACQUIRE", now_ns): reasons.append("LEAD_ACQUIRE")
        if (not status) and prev["status"] and self._allow("LEAD_LOST", now_ns): reasons.append("LEAD_LOST")
        if status and prev["status"] and now_ns - prev["ns"] < int(1.0e9):
          dd = abs((cur["dRel"] or 0.0) - (prev["dRel"] or 0.0)) if cur["dRel"] is not None and prev["dRel"] is not None else 0.0
          dy = abs((cur["yRel"] or 0.0) - (prev["yRel"] or 0.0)) if cur["yRel"] is not None and prev["yRel"] is not None else 0.0
          if (dd >= 6.0 or dy >= 1.4) and self._allow("LEAD_SWAP", now_ns): reasons.append("LEAD_SWAP")
      if status and cur["dRel"] is not None and cur["vRel"] is not None and cur["dRel"] > 1.0 and cur["vRel"] < -1.0:
        ttc = cur["dRel"] / max(0.1, -cur["vRel"])
        if ttc <= self.front_ttc_trigger and self._allow("FRONT_TTC", now_ns): reasons.append("FRONT_TTC")
      self.prev_lead = cur
    except Exception:
      pass
    return reasons

  def corner_can(self, msg, now_ns: int, decoder_ctx):
    reasons = []
    if decoder_ctx is None or now_ns - self.last_corner_probe_ns < 50_000_000:
      return reasons
    self.last_corner_probe_ns = now_ns
    try:
      CORNER_A, CORNER_B, decode_corner24 = decoder_ctx
      log_ns = int(msg.logMonoTime)
      for f in msg.can:
        addr = int(f.address)
        if addr not in CORNER_A and addr not in CORNER_B:
          continue
        o = decode_corner24(addr, bytes(f.dat), log_ns, now_ns)
        if not o:
          continue
        x = _safe_float(getattr(o, "x", None)); y = _safe_float(getattr(o, "y", None)); vx = _safe_float(getattr(o, "vx", None))
        if x is None or y is None or vx is None:
          continue
        if -60.0 <= x <= -3.0 and abs(y) <= 8.5 and vx >= self.rear_closing_mps:
          ttc = (-x) / max(0.1, vx)
          if ttc <= self.rear_ttc_trigger and self._allow("REAR_FAST_APPROACH", now_ns):
            reasons.append("REAR_FAST_APPROACH")
            break
    except Exception:
      pass
    return reasons

  def optional_state(self, state: dict, now_ns: int):
    """Use V35+/future diagnostic state only when a fresh file already exists.

    The logger does not force g80radard to generate this file, so this adds zero
    mandatory UI/debug load. It is opportunistic.
    """
    reasons = []
    try:
      fg = state.get("future_gap", {}) or {}
      for side in ("left", "right"):
        d = fg.get(side, {}) or {}
        stable = int(d.get("stable_incoming_count", 0) or 0)
        prev_stable = int(self.prev_state_sig.get(f"{side}_stable", 0) or 0)
        if stable > 0 and prev_stable == 0:
          r = f"STABLE_INCOMING_{side.upper()}"
          if self._allow(r, now_ns): reasons.append(r)
        self.prev_state_sig[f"{side}_stable"] = stable
        decision = ((d.get("decision") or {}).get("state") or "").upper()
        prev_dec = self.prev_state_sig.get(f"{side}_decision", "")
        if decision in ("CAUTION_SHADOW", "BLOCKED_SHADOW") and decision != prev_dec:
          r = f"{side.upper()}_{decision.replace('_SHADOW','')}"
          if self._allow(r, now_ns): reasons.append(r)
        self.prev_state_sig[f"{side}_decision"] = decision
      ks = state.get("kalman_motion_stats", {}) or {}
      ims = state.get("imm_motion_stats", {}) or {}
      kfreset = int(ks.get("kf_reset_suspect_tracks", 0) or 0)
      imreset = int(ims.get("reset_suspect_tracks", 0) or 0)
      if kfreset > 0 and not self.prev_state_sig.get("kfreset", 0) and self._allow("KF_RESET_SUSPECT", now_ns): reasons.append("KF_RESET_SUSPECT")
      if imreset > 0 and not self.prev_state_sig.get("imreset", 0) and self._allow("IMM_RESET_SUSPECT", now_ns): reasons.append("IMM_RESET_SUSPECT")
      self.prev_state_sig["kfreset"] = kfreset
      self.prev_state_sig["imreset"] = imreset
    except Exception:
      pass
    return reasons


def _acquire_singleton(root: Path):
  lock_path = root / ".g80_raw_golden.lock"
  fp = open(lock_path, "a+", encoding="utf-8")
  try:
    fcntl.flock(fp.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
  except BlockingIOError:
    print(f"G80 golden recorder: another instance owns {lock_path}; exiting duplicate", flush=True)
    fp.close()
    return None
  fp.seek(0); fp.truncate(); fp.write(f"pid={os.getpid()} started={_utc_now()} mode=AUTO_EVENT\n"); fp.flush()
  return fp


def _fresh_optional_state(path: Path, now_ns: int, max_age_ns: int = 1_500_000_000):
  try:
    st = json.loads(path.read_text(encoding="utf-8"))
    mono_ns = int(st.get("mono_ns", 0) or 0)
    if mono_ns <= 0 or abs(now_ns - mono_ns) > max_age_ns:
      return None
    return st
  except Exception:
    return None


def _event_summary(event: Optional[EventSession]) -> dict:
  if event is None:
    return {"active": False}
  return {
    "active": True,
    "path": str(event.path),
    "reasons": list(event.reasons),
    "start_ns": int(event.start_ns),
    "trigger_ns": int(event.trigger_ns),
    "deadline_ns": int(event.deadline_ns),
    "pre_records": int(event.pre_records),
    "writer": event.writer.snapshot(),
  }


def run_recorder() -> int:
  try:
    os.nice(10)
  except Exception:
    pass

  from openpilot.cereal import messaging

  root = Path(os.getenv("G80_RAW_ROOT", DEFAULT_ROOT))
  event_root = Path(os.getenv("G80_RAW_EVENT_ROOT", DEFAULT_EVENT_ROOT))
  root.mkdir(parents=True, exist_ok=True); event_root.mkdir(parents=True, exist_ok=True)
  singleton_fp = _acquire_singleton(root)
  if singleton_fp is None:
    return 0

  pre_sec = max(5.0, float(os.getenv("G80_RAW_PRE_SEC", "30")))
  post_sec = max(5.0, float(os.getenv("G80_RAW_POST_SEC", "45")))
  max_event_sec = max(post_sec + 5.0, float(os.getenv("G80_RAW_MAX_EVENT_SEC", "180")))
  ring_max_mb = max(16.0, float(os.getenv("G80_RAW_RING_MAX_MB", "96")))
  chunk_sec = max(10, int(float(os.getenv("G80_RAW_CHUNK_SEC", "60"))))
  codec = os.getenv("G80_RAW_CODEC", "auto").strip().lower()
  queue_size = max(4096, int(os.getenv("G80_RAW_QUEUE", "32768")))
  min_free_gb = max(1.0, float(os.getenv("G80_RAW_EVENT_MIN_FREE_GB", "4")))
  force_marker = Path(os.getenv("G80_RAW_FORCE_EVENT_MARKER", DEFAULT_FORCE_EVENT))
  state_path = Path(os.getenv("G80_RAW_STATE_PATH", DEFAULT_STATE_PATH))

  ring = RollingBuffer(pre_sec, int(ring_max_mb * 1024 * 1024))
  detector = TriggerDetector()
  active: Optional[EventSession] = None
  event_count = 0
  trigger_counts = collections.Counter()
  serialization_errors = 0
  disk_blocked_events = 0
  last_event_dir = None

  try:
    from openpilot.selfdrive.g80_radar.decoder import CORNER_A, CORNER_B, decode_corner24
    decoder_ctx = (CORNER_A, CORNER_B, decode_corner24)
  except Exception:
    decoder_ctx = None

  stop = threading.Event()
  stop_reason = {"reason": "normal"}
  def request_stop(signum=None, frame=None):
    stop_reason["reason"] = f"signal_{signum}" if signum is not None else "requested"
    stop.set()
  signal.signal(signal.SIGTERM, request_stop); signal.signal(signal.SIGINT, request_stop)

  sockets = {
    "can": messaging.sub_sock("can", timeout=0, conflate=False),
    "modelV2": messaging.sub_sock("modelV2", timeout=0, conflate=False),
    "carState": messaging.sub_sock("carState", timeout=0, conflate=False),
    "radarState": messaging.sub_sock("radarState", timeout=0, conflate=False),
  }
  drain_limits = {"can": 800, "modelV2": 80, "carState": 300, "radarState": 80}
  start_mono = time.monotonic(); start_ns = time.monotonic_ns(); next_status = 0.0; next_state_probe = 0.0
  last_status = {}

  def trigger(reasons, now_ns):
    nonlocal active, event_count, disk_blocked_events, last_event_dir
    reasons = [r for r in reasons if r]
    if not reasons:
      return
    for r in reasons:
      trigger_counts[_sanitize_reason(r)] += 1
    if active is not None:
      for r in reasons:
        active.extend(r, now_ns, post_sec)
      active._write_metadata(active=True)
      return
    free_gb = shutil.disk_usage(event_root).free / (1024 ** 3)
    if free_gb < min_free_gb:
      disk_blocked_events += 1
      return
    active = EventSession(event_root, ring.snapshot(), reasons, now_ns, post_sec, max_event_sec, chunk_sec, codec, queue_size)
    event_count += 1
    last_event_dir = active.path
    (root / "CURRENT_SESSION.txt").write_text(str(active.path) + "\n", encoding="utf-8")

  try:
    while not stop.is_set():
      got = False
      for service, sock in sockets.items():
        for _ in range(drain_limits[service]):
          msg = messaging.recv_one_or_none(sock)
          if msg is None:
            break
          got = True
          recv_ns = time.monotonic_ns()
          try:
            payload = _event_to_bytes(msg); log_ns = int(msg.logMonoTime)
          except Exception:
            serialization_errors += 1
            continue
          item = (service, recv_ns, log_ns, payload)
          active_before = active is not None
          ring.append(item)
          if active_before:
            active.submit(item)

          reasons = []
          if service == "carState": reasons.extend(detector.carstate(msg, recv_ns))
          elif service == "radarState": reasons.extend(detector.radarstate(msg, recv_ns))
          elif service == "can": reasons.extend(detector.corner_can(msg, recv_ns, decoder_ctx))
          if reasons:
            trigger(reasons, recv_ns)

      now = time.monotonic(); now_ns = time.monotonic_ns()

      if force_marker.exists():
        try:
          force_marker.unlink()
        except Exception:
          pass
        trigger(["MANUAL_MARKER"], now_ns)

      if now >= next_state_probe:
        st = _fresh_optional_state(state_path, now_ns)
        if st is not None:
          trigger(detector.optional_state(st, now_ns), now_ns)
        next_state_probe = now + 0.25

      if active is not None and active.due(now_ns):
        finished = active
        active = None
        stats = finished.close()
        last_event_dir = finished.path
        (root / "LAST_SESSION.txt").write_text(str(finished.path) + "\n", encoding="utf-8")
        try:
          (root / "CURRENT_SESSION.txt").unlink()
        except FileNotFoundError:
          pass

      if now >= next_status:
        free_gb = shutil.disk_usage(event_root).free / (1024 ** 3)
        last_status = {
          "mode": "AUTO_EVENT",
          "running": True,
          "recording": active is not None,
          "buffering": True,
          "recorder_version": RECORDER_VERSION,
          "singleton_pid": os.getpid(),
          "start_monotonic_ns": start_ns,
          "elapsed_sec": round(now - start_mono, 1),
          "free_gb": round(free_gb, 2),
          "event_min_free_gb": min_free_gb,
          "pre_sec": pre_sec,
          "post_sec": post_sec,
          "max_event_sec": max_event_sec,
          "ring_records": len(ring.items),
          "ring_bytes": ring.bytes,
          "ring_age_sec": round(ring.age_sec(), 2),
          "ring_max_mb": ring_max_mb,
          "event_count": event_count,
          "active_event": _event_summary(active),
          "last_event": str(last_event_dir) if last_event_dir else None,
          "trigger_counts": dict(trigger_counts),
          "serialization_errors": serialization_errors,
          "disk_blocked_events": disk_blocked_events,
          "decoder_corner_trigger_available": decoder_ctx is not None,
          "optional_state_fresh": _fresh_optional_state(state_path, now_ns) is not None,
          "receive_only": True,
          "can_tx": False,
        }
        _atomic_json(root / "STATUS.json", last_status)
        next_status = now + 2.0

      if not got:
        time.sleep(0.001)
  finally:
    if active is not None:
      try:
        active.close(); last_event_dir = active.path
        (root / "LAST_SESSION.txt").write_text(str(active.path) + "\n", encoding="utf-8")
      except Exception:
        pass
    final = {
      **last_status,
      "running": False,
      "recording": False,
      "ended_utc": _utc_now(),
      "stop_reason": stop_reason["reason"],
      "last_event": str(last_event_dir) if last_event_dir else None,
      "trigger_counts": dict(trigger_counts),
      "serialization_errors": serialization_errors,
      "disk_blocked_events": disk_blocked_events,
    }
    _atomic_json(root / "STATUS.json", final)
    try:
      (root / "CURRENT_SESSION.txt").unlink()
    except FileNotFoundError:
      pass
    try:
      fcntl.flock(singleton_fp.fileno(), fcntl.LOCK_UN); singleton_fp.close()
    except Exception:
      pass
  return 0


def _open_decompressed(path: Path):
  if path.suffix == ".gz": return gzip.open(path, "rb")
  if path.suffix == ".zst":
    import zstandard as zstd
    raw = open(path, "rb")
    return zstd.ZstdDecompressor().stream_reader(raw)
  return open(path, "rb")


def _read_all_records(session: Path):
  got = []
  files = sorted(list(session.glob("chunk_*.g80raw.gz")) + list(session.glob("chunk_*.g80raw.zst")) + list(session.glob("chunk_*.g80raw")))
  for p in files:
    with _open_decompressed(p) as f:
      assert f.read(len(MAGIC)) == MAGIC
      hlen = UINT32.unpack(f.read(4))[0]
      json.loads(f.read(hlen).decode())
      while True:
        h = f.read(RECORD_HEADER.size)
        if not h: break
        assert len(h) == RECORD_HEADER.size
        sid, flags, reserved, recv_ns, log_ns, plen, crc = RECORD_HEADER.unpack(h)
        payload = f.read(plen)
        assert len(payload) == plen
        assert (zlib.crc32(payload) & 0xFFFFFFFF) == crc
        got.append((sid, recv_ns, log_ns, payload))
  return got


def run_self_test(out_root: Path) -> int:
  if out_root.exists(): shutil.rmtree(out_root)
  out_root.mkdir(parents=True)
  ring = RollingBuffer(pre_sec=5.0, max_bytes=8 * 1024 * 1024)
  base = time.monotonic_ns()
  source = []
  # 6 s of 20 Hz fake input -> ring should retain approximately the last 5 s.
  for i in range(120):
    service = list(SERVICE_IDS)[i % 4]
    item = (service, base + i * 50_000_000, 1000 + i, f"fake-{service}-{i}".encode())
    source.append(item); ring.append(item)
  pre = ring.snapshot()
  ev = EventSession(out_root, pre, ["SELFTEST_TRIGGER"], source[-1][1], post_sec=2.0,
                    max_event_sec=10.0, chunk_sec=10, codec="gzip", queue_size=4096)
  post = []
  for i in range(20):
    item = ("can", source[-1][1] + (i + 1) * 50_000_000, 5000 + i, f"post-{i}".encode())
    post.append(item); ev.submit(item)
  stats = ev.close()
  got = _read_all_records(ev.path)
  expected = [(SERVICE_IDS[s], rn, ln, pl) for s, rn, ln, pl in pre + post]
  assert got == expected, (len(got), len(expected))
  assert stats["queue_full_drops"] == 0 and stats["write_errors"] == 0
  meta = json.loads((ev.path / "event.json").read_text())
  assert meta["active"] is False and "SELFTEST_TRIGGER" in meta["reasons"]
  print(json.dumps({
    "self_test": "PASS",
    "recorder_version": RECORDER_VERSION,
    "pre_records": len(pre),
    "post_records": len(post),
    "total_records": len(got),
    "event_dir": str(ev.path),
  }, indent=2))
  return 0


def main() -> int:
  ap = argparse.ArgumentParser()
  ap.add_argument("--self-test", metavar="DIR", help="run rolling-buffer/event-writer self-test without openpilot")
  args = ap.parse_args()
  if args.self_test:
    return run_self_test(Path(args.self_test))
  return run_recorder()


if __name__ == "__main__":
  raise SystemExit(main())

#!/usr/bin/env python3
"""BSD + SCC/HDA2-triggered CAN logger for comma/openpilot/sunnypilot.

v4 async-writer changes:
  - CAN receive loop never writes CSV data directly.
  - BSD pre-trigger history is snapshotted and queued to a dedicated writer thread.
  - Live CAN rows are queued once and fanned out to all active captures by the writer.
  - CSV flush/fsync/close are performed only by the writer thread.
  - This avoids the multi-second CAN capture gap that can occur while dumping the
    20 s BSD pre-buffer in the receive thread.

BSD capture:
  - raw CAN bus 0/1/2
  - 20 s before BSD turns on
  - whole BSD active interval
  - 20 s after BSD turns off

SCC/HDA2 capture:
  - detects CarState ButtonEvent.Type.mainCruise press
  - treats alternating mainCruise presses as ON/OFF, starting from OFF at logger start
  - on SCC Main ON, records raw CAN bus 0/1/2 for exactly 60 s from the trigger
  - fallback: if no mainCruise ButtonEvent has ever been seen, cruiseState.enabled
    rising edge can trigger the SCC capture

HDA2 state markers recorded in every CSV row:
  - scc_main_on: local SCC-M toggle state seen by the logger
  - cruise_enabled: carState.cruiseState.enabled
  - ego_speed_mps: carState.vEgo
  - hda_mode2_raw: raw HDA_MODE2 from CAN 0x1EA
  - hda_cntrl_mod_raw: raw HDA_CntrlModSta from CAN 0x1E0
  - raw HDA values are intentionally not mapped to ACTIVE/INACTIVE until validated

Output naming is kept compatible with v3.
"""

from __future__ import annotations

import csv
import os
import queue
import signal
import threading
import time
import traceback
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Deque, Dict, Optional, TextIO, Tuple

from openpilot.cereal import messaging
from opendbc.car.structs import car


ROOT_DIR = Path("/data/radar")
BUS_IDS = (0, 1, 2)
PRE_TRIGGER_S = 20.0
POST_TRIGGER_S = 20.0
SCC_CAPTURE_S = 60.0
CAN_SOCKET_TIMEOUT_MS = 20
FLUSH_INTERVAL_S = 1.0
CARSTATE_POLL_INTERVAL_S = 0.05  # 20 Hz
STATUS_LOG = ROOT_DIR / "bsm_can_logger.log"

# The queue is intentionally unbounded. Dropping CAN rows is worse than allowing
# a temporary RAM backlog if storage is briefly slower than the CAN input rate.
WRITER_IDLE_POLL_S = 0.20
WRITER_QUEUE_WARN_ITEMS = 100000

ButtonType = car.CarState.ButtonEvent.Type


@dataclass(slots=True)
class CanRow:
  recv_mono_ns: int
  log_mono_ns: int
  wall_time_ns: int
  bus: int
  address: int
  data: bytes
  left_bsd: bool
  right_bsd: bool
  scc_main_on: bool
  cruise_enabled: bool
  ego_speed_mps: float
  hda_mode2_raw: int
  hda_cntrl_mod_raw: int


@dataclass(slots=True)
class OpenCaptureCmd:
  capture_id: str
  trigger_mono_ns: int
  paths: Dict[int, Path]
  prebuffer: Tuple[CanRow, ...]


@dataclass(slots=True)
class WriteRowCmd:
  capture_ids: Tuple[str, ...]
  row: CanRow


@dataclass(slots=True)
class CloseCaptureCmd:
  capture_id: str


@dataclass(slots=True)
class StopWriterCmd:
  pass


@dataclass(slots=True)
class _WriterCapture:
  trigger_mono_ns: int
  files: Dict[int, TextIO]
  writers: Dict[int, csv.writer]


class AsyncCsvWriter:
  """Single background writer that owns every CSV file handle.

  FIFO command ordering guarantees:
    OPEN(prebuffer) -> live ROW commands -> CLOSE
  for each capture. The CAN receive loop only enqueues commands and therefore
  never waits for CSV formatting, filesystem writes, flush, or fsync.
  """

  HEADER = [
    "wall_time",
    "recv_mono_ns",
    "log_mono_ns",
    "rel_trigger_s",
    "bus",
    "address_hex",
    "address_dec",
    "dlc",
    "data_hex",
    "left_bsd",
    "right_bsd",
    "scc_main_on",
    "cruise_enabled",
    "ego_speed_mps",
    "hda_mode2_raw",
    "hda_cntrl_mod_raw",
  ]

  def __init__(self) -> None:
    self._q: queue.Queue[object] = queue.Queue()
    self._thread = threading.Thread(target=self._run, name="bsm_csv_writer", daemon=False)
    self._captures: Dict[str, _WriterCapture] = {}
    self._failed_exc: Optional[BaseException] = None
    self._failed_traceback: Optional[str] = None
    self._last_flush_mono = time.monotonic()
    self._max_queue_depth = 0
    self._thread.start()

  @property
  def max_queue_depth(self) -> int:
    return self._max_queue_depth

  def queue_depth(self) -> int:
    try:
      return self._q.qsize()
    except NotImplementedError:
      return -1

  def raise_if_failed(self) -> None:
    if self._failed_exc is not None:
      detail = self._failed_traceback or repr(self._failed_exc)
      raise RuntimeError(f"CSV writer thread failed:\n{detail}") from self._failed_exc

  def open_capture(self, capture_id: str, trigger_mono_ns: int,
                   paths: Dict[int, Path], prebuffer: Tuple[CanRow, ...] = ()) -> None:
    self.raise_if_failed()
    self._put(OpenCaptureCmd(capture_id, trigger_mono_ns, paths, prebuffer))

  def write_row(self, capture_ids: Tuple[str, ...], row: CanRow) -> None:
    if not capture_ids:
      return
    self.raise_if_failed()
    self._put(WriteRowCmd(capture_ids, row))

  def close_capture(self, capture_id: str) -> None:
    self.raise_if_failed()
    self._put(CloseCaptureCmd(capture_id))

  def stop_and_join(self) -> None:
    # FIFO: all pending OPEN/ROW/CLOSE commands are handled before STOP.
    self._put(StopWriterCmd())
    self._thread.join()
    self.raise_if_failed()

  def _put(self, cmd: object) -> None:
    self._q.put_nowait(cmd)
    try:
      depth = self._q.qsize()
      if depth > self._max_queue_depth:
        self._max_queue_depth = depth
    except NotImplementedError:
      pass

  @classmethod
  def _write_row_to_capture(cls, cap: _WriterCapture, row: CanRow) -> None:
    writer = cap.writers.get(row.bus)
    if writer is None:
      return

    wall_dt = datetime.fromtimestamp(row.wall_time_ns / 1e9)
    rel_s = (row.recv_mono_ns - cap.trigger_mono_ns) / 1e9
    writer.writerow([
      wall_dt.isoformat(timespec="milliseconds"),
      row.recv_mono_ns,
      row.log_mono_ns,
      f"{rel_s:.6f}",
      row.bus,
      f"0x{row.address:X}",
      row.address,
      len(row.data),
      row.data.hex().upper(),
      int(row.left_bsd),
      int(row.right_bsd),
      int(row.scc_main_on),
      int(row.cruise_enabled),
      f"{row.ego_speed_mps:.3f}",
      row.hda_mode2_raw,
      row.hda_cntrl_mod_raw,
    ])

  def _open_capture(self, cmd: OpenCaptureCmd) -> None:
    if cmd.capture_id in self._captures:
      raise RuntimeError(f"duplicate capture_id: {cmd.capture_id}")

    files: Dict[int, TextIO] = {}
    writers: Dict[int, csv.writer] = {}
    try:
      for bus, path in cmd.paths.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        f = open(path, "w", newline="", buffering=1024 * 1024)
        writer = csv.writer(f)
        writer.writerow(self.HEADER)
        files[bus] = f
        writers[bus] = writer

      cap = _WriterCapture(cmd.trigger_mono_ns, files, writers)
      self._captures[cmd.capture_id] = cap

      # This potentially large write is intentionally done here, never in the
      # CAN receive loop.
      for row in cmd.prebuffer:
        self._write_row_to_capture(cap, row)
    except Exception:
      for f in files.values():
        try:
          f.close()
        except OSError:
          pass
      raise

  def _close_capture(self, capture_id: str) -> None:
    cap = self._captures.pop(capture_id, None)
    if cap is None:
      return

    for f in cap.files.values():
      try:
        f.flush()
        os.fsync(f.fileno())
      except OSError:
        pass
      try:
        f.close()
      except OSError:
        pass

  def _flush_all(self, force: bool = False) -> None:
    now = time.monotonic()
    if not force and (now - self._last_flush_mono) < FLUSH_INTERVAL_S:
      return
    for cap in self._captures.values():
      for f in cap.files.values():
        try:
          f.flush()
        except OSError:
          pass
    self._last_flush_mono = now

  def _close_all(self) -> None:
    for capture_id in list(self._captures):
      self._close_capture(capture_id)

  def _run(self) -> None:
    try:
      while True:
        try:
          cmd = self._q.get(timeout=WRITER_IDLE_POLL_S)
        except queue.Empty:
          self._flush_all()
          continue

        if isinstance(cmd, OpenCaptureCmd):
          self._open_capture(cmd)
        elif isinstance(cmd, WriteRowCmd):
          for capture_id in cmd.capture_ids:
            cap = self._captures.get(capture_id)
            if cap is not None:
              self._write_row_to_capture(cap, cmd.row)
        elif isinstance(cmd, CloseCaptureCmd):
          self._close_capture(cmd.capture_id)
        elif isinstance(cmd, StopWriterCmd):
          self._flush_all(force=True)
          self._close_all()
          break
        else:
          raise RuntimeError(f"unknown writer command: {type(cmd)!r}")

        self._flush_all()
    except BaseException as exc:
      self._failed_exc = exc
      self._failed_traceback = traceback.format_exc()
      try:
        self._close_all()
      except Exception:
        pass


class CaptureBase:
  def __init__(self, capture_id: str, trigger_mono_ns: int,
               event_dir: Path, paths: Dict[int, Path]):
    self.capture_id = capture_id
    self.trigger_mono_ns = trigger_mono_ns
    self.event_dir = event_dir
    self.paths = paths


class BsdCapture(CaptureBase):
  def __init__(self, side: str, trigger_dt: datetime, trigger_mono_ns: int):
    self.side = side  # 'L' or 'R'
    self.trigger_dt = trigger_dt
    self.tail_deadline_ns: Optional[int] = None

    event_dir = self._make_unique_event_dir(trigger_dt, side)
    event_dir.mkdir(parents=True, exist_ok=True)
    hhmmss = trigger_dt.strftime("%H_%M_%S")
    paths = {
      bus: event_dir / f"{side}_radar_bus{bus}_{hhmmss}.csv"
      for bus in BUS_IDS
    }
    capture_id = f"BSD:{side}:{event_dir.name}:{trigger_mono_ns}"
    super().__init__(capture_id, trigger_mono_ns, event_dir, paths)

  @staticmethod
  def _make_unique_event_dir(trigger_dt: datetime, side: str) -> Path:
    base = ROOT_DIR / trigger_dt.strftime("%Y-%m-%d_%H-%M-%S")
    if not base.exists():
      return base

    # Opposite side may share the same event directory.
    hhmmss = trigger_dt.strftime("%H_%M_%S")
    side_probe = base / f"{side}_radar_bus0_{hhmmss}.csv"
    if not side_probe.exists():
      return base

    for idx in range(1, 1000):
      candidate = ROOT_DIR / f"{trigger_dt.strftime('%Y-%m-%d_%H-%M-%S')}_{idx:02d}"
      if not candidate.exists():
        return candidate
    raise RuntimeError("Could not allocate a unique radar event directory")

  def set_bsd_state(self, active: bool, now_mono_ns: int) -> None:
    if active:
      self.tail_deadline_ns = None
    elif self.tail_deadline_ns is None:
      self.tail_deadline_ns = now_mono_ns + int(POST_TRIGGER_S * 1e9)

  def should_close(self, now_mono_ns: int) -> bool:
    return self.tail_deadline_ns is not None and now_mono_ns >= self.tail_deadline_ns


class SccCapture(CaptureBase):
  def __init__(self, trigger_dt: datetime, trigger_mono_ns: int, trigger_reason: str):
    self.trigger_dt = trigger_dt
    self.trigger_reason = trigger_reason
    self.deadline_ns = trigger_mono_ns + int(SCC_CAPTURE_S * 1e9)

    event_dir = self._make_unique_event_dir(trigger_dt)
    event_dir.mkdir(parents=True, exist_ok=True)
    hhmmss = trigger_dt.strftime("%H_%M_%S")
    paths = {
      bus: event_dir / f"scc_bus{bus}_{hhmmss}.csv"
      for bus in BUS_IDS
    }
    capture_id = f"SCC:{event_dir.name}:{trigger_mono_ns}"
    super().__init__(capture_id, trigger_mono_ns, event_dir, paths)

  @staticmethod
  def _make_unique_event_dir(trigger_dt: datetime) -> Path:
    stem = trigger_dt.strftime("%Y-%m-%d_%H-%M-%S") + "_SCC"
    base = ROOT_DIR / stem
    if not base.exists():
      return base

    for idx in range(1, 1000):
      candidate = ROOT_DIR / f"{stem}_{idx:02d}"
      if not candidate.exists():
        return candidate
    raise RuntimeError("Could not allocate a unique SCC event directory")

  def should_close(self, now_mono_ns: int) -> bool:
    return now_mono_ns >= self.deadline_ns


running = True


def _signal_handler(signum, frame) -> None:
  del signum, frame
  global running
  running = False


def log_status(text: str) -> None:
  ROOT_DIR.mkdir(parents=True, exist_ok=True)
  ts = datetime.now().isoformat(timespec="seconds")
  line = f"{ts} {text}\n"
  try:
    with open(STATUS_LOG, "a", buffering=1) as f:
      f.write(line)
  except OSError:
    pass
  print(f"[bsm_can_logger] {text}", flush=True)


def _active_capture_ids(bsd_captures: Dict[str, BsdCapture],
                        scc_capture: Optional[SccCapture]) -> Tuple[str, ...]:
  ids = [cap.capture_id for cap in bsd_captures.values()]
  if scc_capture is not None:
    ids.append(scc_capture.capture_id)
  return tuple(ids)


def main() -> None:
  global running

  ROOT_DIR.mkdir(parents=True, exist_ok=True)
  signal.signal(signal.SIGTERM, _signal_handler)
  signal.signal(signal.SIGINT, _signal_handler)

  # Raw CAN must not be conflated: every published CAN Event is relevant.
  can_sock = messaging.sub_sock("can", timeout=CAN_SOCKET_TIMEOUT_MS, conflate=False)

  # State only needs edge detection. Read the newest carState at 20 Hz.
  carstate_sock = messaging.sub_sock("carState", conflate=True)

  writer = AsyncCsvWriter()

  prebuffer: Deque[CanRow] = deque()
  bsd_captures: Dict[str, BsdCapture] = {}
  scc_capture: Optional[SccCapture] = None

  prev_left = False
  prev_right = False
  cur_left = False
  cur_right = False
  cur_cruise_enabled = False
  cur_ego_speed_mps = 0.0

  # HDA state hints decoded directly from raw CAN.
  # 0x1EA ADRV_0x1ea: HDA_MODE2 = start bit 32, length 3, little-endian.
  # 0x1E0 LFAHDA_CLUSTER: HDA_CntrlModSta = start bit 30, length 2, little-endian.
  # Keep raw numeric states instead of assuming which value means fully ACTIVE.
  hda_mode2_raw = -1
  hda_cntrl_mod_raw = -1

  last_carstate_poll_ns = 0
  last_queue_warn_ns = 0

  # mainCruise is a momentary ButtonEvent, not a latched state. Track the
  # ON/OFF toggle locally; logger normally starts with ignition/onroad and SCC Main OFF.
  scc_main_on = False
  saw_main_button_event = False

  # Fallback for ports that do not publish mainCruise ButtonEvent.
  carstate_initialized = False
  prev_cruise_enabled = False

  log_status(
    f"START v4-async buses={BUS_IDS} BSD(pre={PRE_TRIGGER_S:.1f}s post={POST_TRIGGER_S:.1f}s) "
    f"SCC={SCC_CAPTURE_S:.1f}s HDAraw=(0x1EA.HDA_MODE2,0x1E0.HDA_CntrlModSta) root={ROOT_DIR}"
  )

  try:
    while running:
      writer.raise_if_failed()

      msg = messaging.recv_one(can_sock)
      recv_mono_ns = time.monotonic_ns()
      wall_time_ns = time.time_ns()

      if msg is not None:
        log_mono_ns = int(msg.logMonoTime)
        for can in msg.can:
          bus = int(can.src)
          if bus not in BUS_IDS:
            continue

          address = int(can.address)
          data = bytes(can.dat)

          # Decode HDA-related raw states from the messages themselves.
          # These are logged as RAW values; no ACTIVE mapping is assumed.
          if address == 0x1EA and len(data) >= 5:
            hda_mode2_raw = data[4] & 0x07
          elif address == 0x1E0 and len(data) >= 4:
            hda_cntrl_mod_raw = (data[3] >> 6) & 0x03

          row = CanRow(
            recv_mono_ns=recv_mono_ns,
            log_mono_ns=log_mono_ns,
            wall_time_ns=wall_time_ns,
            bus=bus,
            address=address,
            data=data,
            left_bsd=cur_left,
            right_bsd=cur_right,
            scc_main_on=scc_main_on,
            cruise_enabled=cur_cruise_enabled,
            ego_speed_mps=cur_ego_speed_mps,
            hda_mode2_raw=hda_mode2_raw,
            hda_cntrl_mod_raw=hda_cntrl_mod_raw,
          )

          # RAM-only work in the receive thread.
          prebuffer.append(row)

          # One queue item per CAN row, regardless of whether L/R/SCC captures
          # overlap. The writer fans the row out to every requested capture.
          capture_ids = _active_capture_ids(bsd_captures, scc_capture)
          writer.write_row(capture_ids, row)

      # Keep only approximately the latest PRE_TRIGGER_S seconds for BSD.
      cutoff_ns = recv_mono_ns - int(PRE_TRIGGER_S * 1e9)
      while prebuffer and prebuffer[0].recv_mono_ns < cutoff_ns:
        prebuffer.popleft()

      if (recv_mono_ns - last_carstate_poll_ns) >= int(CARSTATE_POLL_INTERVAL_S * 1e9):
        last_carstate_poll_ns = recv_mono_ns
        cs_msg = messaging.recv_one_or_none(carstate_sock)
        if cs_msg is not None:
          cs = cs_msg.carState
          cur_left = bool(cs.leftBlindspot)
          cur_right = bool(cs.rightBlindspot)
          cruise_enabled = bool(cs.cruiseState.enabled)
          cur_cruise_enabled = cruise_enabled
          cur_ego_speed_mps = float(cs.vEgo)

          # BSD rising edges open independent left/right captures. Only a tuple
          # snapshot of references is created here; all CSV work is asynchronous.
          if cur_left and not prev_left and "L" not in bsd_captures:
            cap = BsdCapture("L", datetime.now(), recv_mono_ns)
            bsd_captures["L"] = cap
            writer.open_capture(cap.capture_id, cap.trigger_mono_ns, cap.paths, tuple(prebuffer))
            log_status(
              f"L BSD ON -> async capture opened: {cap.event_dir} "
              f"pre_rows={len(prebuffer)} q={writer.queue_depth()}"
            )

          if cur_right and not prev_right and "R" not in bsd_captures:
            cap = BsdCapture("R", datetime.now(), recv_mono_ns)
            bsd_captures["R"] = cap
            writer.open_capture(cap.capture_id, cap.trigger_mono_ns, cap.paths, tuple(prebuffer))
            log_status(
              f"R BSD ON -> async capture opened: {cap.event_dir} "
              f"pre_rows={len(prebuffer)} q={writer.queue_depth()}"
            )

          if "L" in bsd_captures:
            bsd_captures["L"].set_bsd_state(cur_left, recv_mono_ns)
          if "R" in bsd_captures:
            bsd_captures["R"].set_bsd_state(cur_right, recv_mono_ns)

          prev_left = cur_left
          prev_right = cur_right

          # SCC Main button: create_button_events() publishes a pressed event
          # for the physical mainCruise button. The button itself is momentary,
          # so alternate presses are treated as ON / OFF.
          main_pressed = any(
            bool(b.pressed) and b.type.raw == ButtonType.mainCruise
            for b in cs.buttonEvents
          )

          if main_pressed:
            saw_main_button_event = True
            scc_main_on = not scc_main_on
            if scc_main_on:
              if scc_capture is None:
                cap = SccCapture(datetime.now(), recv_mono_ns, "mainCruise ON")
                scc_capture = cap
                writer.open_capture(cap.capture_id, cap.trigger_mono_ns, cap.paths)
                log_status(
                  f"SCC MAIN ON -> {SCC_CAPTURE_S:.0f}s async capture opened: {cap.event_dir}"
                )
              else:
                log_status("SCC MAIN ON while SCC capture already active -> existing capture kept")
            else:
              log_status("SCC MAIN OFF -> no SCC capture started")

          # Fallback only when this car/port has never exposed mainCruise
          # ButtonEvent. This catches a clean SCC engagement rising edge.
          if carstate_initialized and (not saw_main_button_event):
            if cruise_enabled and not prev_cruise_enabled and scc_capture is None:
              cap = SccCapture(datetime.now(), recv_mono_ns, "cruiseState.enabled rising fallback")
              scc_capture = cap
              writer.open_capture(cap.capture_id, cap.trigger_mono_ns, cap.paths)
              log_status(
                f"SCC ENABLED rising fallback -> {SCC_CAPTURE_S:.0f}s async capture opened: {cap.event_dir}"
              )

          prev_cruise_enabled = cruise_enabled
          carstate_initialized = True

      # Close completed BSD captures asynchronously. Because the close command is
      # queued after all previous row commands, no already-received row is lost.
      for side, cap in list(bsd_captures.items()):
        if cap.should_close(recv_mono_ns):
          event_dir = cap.event_dir
          writer.close_capture(cap.capture_id)
          del bsd_captures[side]
          log_status(
            f"{side} BSD tail complete -> async close queued: {event_dir} q={writer.queue_depth()}"
          )

      # Close the fixed-duration SCC capture asynchronously.
      if scc_capture is not None and scc_capture.should_close(recv_mono_ns):
        event_dir = scc_capture.event_dir
        writer.close_capture(scc_capture.capture_id)
        scc_capture = None
        log_status(
          f"SCC {SCC_CAPTURE_S:.0f}s capture complete -> async close queued: "
          f"{event_dir} q={writer.queue_depth()}"
        )

      # Diagnostic only; no rows are dropped even if the queue is deep.
      qdepth = writer.queue_depth()
      if (qdepth >= WRITER_QUEUE_WARN_ITEMS and
          recv_mono_ns - last_queue_warn_ns >= int(5.0 * 1e9)):
        last_queue_warn_ns = recv_mono_ns
        log_status(f"WARNING writer queue backlog={qdepth} rows/commands; capture continues without dropping")

  finally:
    # Stop adding rows first, then queue close commands. stop_and_join() places
    # STOP after them, so every queued row is written before shutdown returns.
    for side, cap in list(bsd_captures.items()):
      writer.close_capture(cap.capture_id)
      log_status(f"{side} forced close queued: {cap.event_dir}")
    bsd_captures.clear()

    if scc_capture is not None:
      writer.close_capture(scc_capture.capture_id)
      log_status(f"SCC forced close queued: {scc_capture.event_dir}")
      scc_capture = None

    try:
      writer.stop_and_join()
      log_status(f"ASYNC WRITER STOP max_queue_depth={writer.max_queue_depth}")
    finally:
      log_status("STOP")


if __name__ == "__main__":
  main()

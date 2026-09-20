#!/usr/bin/env python3
"""BSD-triggered CAN logger for comma/openpilot/sunnypilot.

Captures raw CAN from bus 0, bus 1, and bus 2 around stock blind-spot events:
  - 5 s before BSD turns on
  - whole BSD active interval
  - 10 s after BSD turns off

Output example:
  /data/radar/2026-09-20_14-21-05/
    R_radar_bus0_14_21_05.csv
    R_radar_bus1_14_21_05.csv
    R_radar_bus2_14_21_05.csv

The directory timestamp intentionally uses '_' and '-' rather than ':' so that
files can be copied to Windows via WinSCP without filename problems.
"""

from __future__ import annotations

import csv
import os
import signal
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Deque, Dict, Optional, TextIO

from openpilot.cereal import messaging


ROOT_DIR = Path("/data/radar")
BUS_IDS = (0, 1, 2)
PRE_TRIGGER_S = 5.0
POST_TRIGGER_S = 10.0
CAN_SOCKET_TIMEOUT_MS = 20
FLUSH_INTERVAL_S = 1.0
BSD_POLL_INTERVAL_S = 0.05  # 20 Hz; enough for BSM trigger detection with lower overhead
STATUS_LOG = ROOT_DIR / "bsm_can_logger.log"


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


class Capture:
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
  ]

  def __init__(self, side: str, trigger_dt: datetime, trigger_mono_ns: int,
               prebuffer: Deque[CanRow]):
    self.side = side  # 'L' or 'R'
    self.trigger_dt = trigger_dt
    self.trigger_mono_ns = trigger_mono_ns
    self.tail_deadline_ns: Optional[int] = None
    self.last_flush_ns = trigger_mono_ns

    self.event_dir = self._make_unique_event_dir(trigger_dt, side)
    self.event_dir.mkdir(parents=True, exist_ok=True)

    hhmmss = trigger_dt.strftime("%H_%M_%S")
    self.files: Dict[int, TextIO] = {}
    self.writers: Dict[int, csv.writer] = {}

    for bus in BUS_IDS:
      path = self.event_dir / f"{side}_radar_bus{bus}_{hhmmss}.csv"
      f = open(path, "w", newline="", buffering=1024 * 1024)
      writer = csv.writer(f)
      writer.writerow(self.HEADER)
      self.files[bus] = f
      self.writers[bus] = writer

    # Snapshot the already-collected 5 second history.
    for row in prebuffer:
      self.write_row(row)

    self.flush(force=True)

  @staticmethod
  def _make_unique_event_dir(trigger_dt: datetime, side: str) -> Path:
    # Windows-safe timestamp. If another event happened in the same second,
    # add _01, _02, ... rather than overwriting it.
    base = ROOT_DIR / trigger_dt.strftime("%Y-%m-%d_%H-%M-%S")
    if not base.exists():
      return base

    # If the opposite side triggers essentially simultaneously, sharing the
    # same directory is convenient as long as our side's files do not exist.
    hhmmss = trigger_dt.strftime("%H_%M_%S")
    side_probe = base / f"{side}_radar_bus0_{hhmmss}.csv"
    if not side_probe.exists():
      return base

    for idx in range(1, 1000):
      candidate = ROOT_DIR / f"{trigger_dt.strftime('%Y-%m-%d_%H-%M-%S')}_{idx:02d}"
      if not candidate.exists():
        return candidate
    raise RuntimeError("Could not allocate a unique radar event directory")

  def write_row(self, row: CanRow) -> None:
    writer = self.writers.get(row.bus)
    if writer is None:
      return

    wall_dt = datetime.fromtimestamp(row.wall_time_ns / 1e9)
    rel_s = (row.recv_mono_ns - self.trigger_mono_ns) / 1e9
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
    ])

  def set_bsd_state(self, active: bool, now_mono_ns: int) -> None:
    if active:
      # BSD reactivated during the post-trigger tail: keep the same capture.
      self.tail_deadline_ns = None
    elif self.tail_deadline_ns is None:
      self.tail_deadline_ns = now_mono_ns + int(POST_TRIGGER_S * 1e9)

  def should_close(self, now_mono_ns: int) -> bool:
    return self.tail_deadline_ns is not None and now_mono_ns >= self.tail_deadline_ns

  def flush(self, now_mono_ns: Optional[int] = None, force: bool = False) -> None:
    if now_mono_ns is None:
      now_mono_ns = time.monotonic_ns()
    if not force and (now_mono_ns - self.last_flush_ns) < int(FLUSH_INTERVAL_S * 1e9):
      return

    for f in self.files.values():
      f.flush()
    self.last_flush_ns = now_mono_ns

  def close(self) -> None:
    for f in self.files.values():
      try:
        f.flush()
        os.fsync(f.fileno())
      except OSError:
        pass
      try:
        f.close()
      except OSError:
        pass


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


def main() -> None:
  global running

  ROOT_DIR.mkdir(parents=True, exist_ok=True)
  signal.signal(signal.SIGTERM, _signal_handler)
  signal.signal(signal.SIGINT, _signal_handler)

  # Raw CAN must not be conflated: every published CAN Event is relevant.
  can_sock = messaging.sub_sock("can", timeout=CAN_SOCKET_TIMEOUT_MS, conflate=False)

  # BSD state only needs edge detection. Read the newest carState at 20 Hz to
  # reduce subscriber/polling overhead on sunnypilot.
  carstate_sock = messaging.sub_sock("carState", conflate=True)

  prebuffer: Deque[CanRow] = deque()
  captures: Dict[str, Capture] = {}

  prev_left = False
  prev_right = False
  cur_left = False
  cur_right = False
  last_bsd_poll_ns = 0

  log_status(
    f"START buses={BUS_IDS} pre={PRE_TRIGGER_S:.1f}s post={POST_TRIGGER_S:.1f}s root={ROOT_DIR}"
  )

  try:
    while running:
      # Receive one raw CAN Event. The socket timeout lets the loop still
      # service BSD polling and capture closing if CAN traffic pauses.
      msg = messaging.recv_one(can_sock)
      recv_mono_ns = time.monotonic_ns()
      wall_time_ns = time.time_ns()

      if msg is not None:
        log_mono_ns = int(msg.logMonoTime)
        for can in msg.can:
          bus = int(can.src)
          if bus not in BUS_IDS:
            continue

          row = CanRow(
            recv_mono_ns=recv_mono_ns,
            log_mono_ns=log_mono_ns,
            wall_time_ns=wall_time_ns,
            bus=bus,
            address=int(can.address),
            data=bytes(can.dat),
            left_bsd=cur_left,
            right_bsd=cur_right,
          )

          prebuffer.append(row)
          for cap in captures.values():
            cap.write_row(row)

      # Keep only approximately the latest PRE_TRIGGER_S seconds.
      cutoff_ns = recv_mono_ns - int(PRE_TRIGGER_S * 1e9)
      while prebuffer and prebuffer[0].recv_mono_ns < cutoff_ns:
        prebuffer.popleft()

      # Poll only the newest carState at 20 Hz. A trigger detected a few tens
      # of milliseconds late is harmless because the raw-CAN prebuffer has
      # already retained the complete preceding interval.
      if (recv_mono_ns - last_bsd_poll_ns) >= int(BSD_POLL_INTERVAL_S * 1e9):
        last_bsd_poll_ns = recv_mono_ns
        cs_msg = messaging.recv_one_or_none(carstate_sock)
        if cs_msg is not None:
          cs = cs_msg.carState
          cur_left = bool(cs.leftBlindspot)
          cur_right = bool(cs.rightBlindspot)

          # Rising edges open independent left/right captures.
          if cur_left and not prev_left and "L" not in captures:
            cap = Capture("L", datetime.now(), recv_mono_ns, prebuffer)
            captures["L"] = cap
            log_status(f"L BSD ON -> capture opened: {cap.event_dir}")

          if cur_right and not prev_right and "R" not in captures:
            cap = Capture("R", datetime.now(), recv_mono_ns, prebuffer)
            captures["R"] = cap
            log_status(f"R BSD ON -> capture opened: {cap.event_dir}")

          # Active BSD cancels a pending close. Falling state starts/maintains
          # the 10 second post-trigger tail.
          if "L" in captures:
            captures["L"].set_bsd_state(cur_left, recv_mono_ns)
          if "R" in captures:
            captures["R"].set_bsd_state(cur_right, recv_mono_ns)

          prev_left = cur_left
          prev_right = cur_right

      # Flush and close completed captures.
      for side, cap in list(captures.items()):
        cap.flush(recv_mono_ns)
        if cap.should_close(recv_mono_ns):
          event_dir = cap.event_dir
          cap.close()
          del captures[side]
          log_status(f"{side} BSD tail complete -> capture closed: {event_dir}")

  finally:
    for side, cap in list(captures.items()):
      event_dir = cap.event_dir
      cap.close()
      log_status(f"{side} forced close: {event_dir}")
    log_status("STOP")


if __name__ == "__main__":
  main()

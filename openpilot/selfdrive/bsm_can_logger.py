#!/usr/bin/env python3
"""BSD + SCC-triggered CAN logger for comma/openpilot/sunnypilot.

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
  - ego_speed_mps: carState.vEgo, useful for absolute target speed/validation
  - hda_mode2_raw: raw HDA_MODE2 from CAN 0x1EA
  - hda_cntrl_mod_raw: raw HDA_CntrlModSta from CAN 0x1E0
  - raw HDA values are intentionally not mapped to ACTIVE/INACTIVE until validated on this G80

Output examples:
  /data/radar/2026-09-20_14-21-05/
    R_radar_bus0_14_21_05.csv
    R_radar_bus1_14_21_05.csv
    R_radar_bus2_14_21_05.csv

  /data/radar/2026-09-20_14-30-15_SCC/
    scc_bus0_14_30_15.csv
    scc_bus1_14_30_15.csv
    scc_bus2_14_30_15.csv

Directory timestamps intentionally avoid ':' so files copy cleanly to Windows.
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


class CsvCaptureBase:
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

  def __init__(self, trigger_mono_ns: int):
    self.trigger_mono_ns = trigger_mono_ns
    self.last_flush_ns = trigger_mono_ns
    self.files: Dict[int, TextIO] = {}
    self.writers: Dict[int, csv.writer] = {}

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
      int(row.scc_main_on),
      int(row.cruise_enabled),
      f"{row.ego_speed_mps:.3f}",
      row.hda_mode2_raw,
      row.hda_cntrl_mod_raw,
    ])

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


class BsdCapture(CsvCaptureBase):
  def __init__(self, side: str, trigger_dt: datetime, trigger_mono_ns: int,
               prebuffer: Deque[CanRow]):
    super().__init__(trigger_mono_ns)
    self.side = side  # 'L' or 'R'
    self.trigger_dt = trigger_dt
    self.tail_deadline_ns: Optional[int] = None

    self.event_dir = self._make_unique_event_dir(trigger_dt, side)
    self.event_dir.mkdir(parents=True, exist_ok=True)

    hhmmss = trigger_dt.strftime("%H_%M_%S")
    for bus in BUS_IDS:
      path = self.event_dir / f"{side}_radar_bus{bus}_{hhmmss}.csv"
      f = open(path, "w", newline="", buffering=1024 * 1024)
      writer = csv.writer(f)
      writer.writerow(self.HEADER)
      self.files[bus] = f
      self.writers[bus] = writer

    # Snapshot the already-collected PRE_TRIGGER_S history.
    for row in prebuffer:
      self.write_row(row)

    self.flush(force=True)

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


class SccCapture(CsvCaptureBase):
  def __init__(self, trigger_dt: datetime, trigger_mono_ns: int, trigger_reason: str):
    super().__init__(trigger_mono_ns)
    self.trigger_dt = trigger_dt
    self.trigger_reason = trigger_reason
    self.deadline_ns = trigger_mono_ns + int(SCC_CAPTURE_S * 1e9)

    self.event_dir = self._make_unique_event_dir(trigger_dt)
    self.event_dir.mkdir(parents=True, exist_ok=True)

    hhmmss = trigger_dt.strftime("%H_%M_%S")
    for bus in BUS_IDS:
      path = self.event_dir / f"scc_bus{bus}_{hhmmss}.csv"
      f = open(path, "w", newline="", buffering=1024 * 1024)
      writer = csv.writer(f)
      writer.writerow(self.HEADER)
      self.files[bus] = f
      self.writers[bus] = writer

    self.flush(force=True)

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


def main() -> None:
  global running

  ROOT_DIR.mkdir(parents=True, exist_ok=True)
  signal.signal(signal.SIGTERM, _signal_handler)
  signal.signal(signal.SIGINT, _signal_handler)

  # Raw CAN must not be conflated: every published CAN Event is relevant.
  can_sock = messaging.sub_sock("can", timeout=CAN_SOCKET_TIMEOUT_MS, conflate=False)

  # State only needs edge detection. Read the newest carState at 20 Hz.
  carstate_sock = messaging.sub_sock("carState", conflate=True)

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

  # mainCruise is a momentary ButtonEvent, not a latched state. Track the
  # ON/OFF toggle locally; logger normally starts with ignition/onroad and SCC Main OFF.
  scc_main_on = False
  saw_main_button_event = False

  # Fallback for ports that do not publish mainCruise ButtonEvent.
  carstate_initialized = False
  prev_cruise_enabled = False

  log_status(
    f"START buses={BUS_IDS} BSD(pre={PRE_TRIGGER_S:.1f}s post={POST_TRIGGER_S:.1f}s) "
    f"SCC={SCC_CAPTURE_S:.1f}s HDAraw=(0x1EA.HDA_MODE2,0x1E0.HDA_CntrlModSta) root={ROOT_DIR}"
  )

  try:
    while running:
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

          prebuffer.append(row)
          for cap in bsd_captures.values():
            cap.write_row(row)
          if scc_capture is not None:
            scc_capture.write_row(row)

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

          # BSD rising edges open independent left/right captures.
          if cur_left and not prev_left and "L" not in bsd_captures:
            cap = BsdCapture("L", datetime.now(), recv_mono_ns, prebuffer)
            bsd_captures["L"] = cap
            log_status(f"L BSD ON -> capture opened: {cap.event_dir}")

          if cur_right and not prev_right and "R" not in bsd_captures:
            cap = BsdCapture("R", datetime.now(), recv_mono_ns, prebuffer)
            bsd_captures["R"] = cap
            log_status(f"R BSD ON -> capture opened: {cap.event_dir}")

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
                log_status(f"SCC MAIN ON -> {SCC_CAPTURE_S:.0f}s capture opened: {cap.event_dir}")
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
              log_status(f"SCC ENABLED rising fallback -> 10s capture opened: {cap.event_dir}")

          prev_cruise_enabled = cruise_enabled
          carstate_initialized = True

      # Flush and close completed BSD captures.
      for side, cap in list(bsd_captures.items()):
        cap.flush(recv_mono_ns)
        if cap.should_close(recv_mono_ns):
          event_dir = cap.event_dir
          cap.close()
          del bsd_captures[side]
          log_status(f"{side} BSD tail complete -> capture closed: {event_dir}")

      # Flush and close the fixed-duration SCC capture.
      if scc_capture is not None:
        scc_capture.flush(recv_mono_ns)
        if scc_capture.should_close(recv_mono_ns):
          event_dir = scc_capture.event_dir
          scc_capture.close()
          scc_capture = None
          log_status(f"SCC {SCC_CAPTURE_S:.0f}s capture complete -> capture closed: {event_dir}")

  finally:
    for side, cap in list(bsd_captures.items()):
      event_dir = cap.event_dir
      cap.close()
      log_status(f"{side} forced close: {event_dir}")
    if scc_capture is not None:
      event_dir = scc_capture.event_dir
      scc_capture.close()
      log_status(f"SCC forced close: {event_dir}")
    log_status("STOP")


if __name__ == "__main__":
  main()

#!/usr/bin/env python3
"""G80 RG3 HDA2 radar logger + empirical live decoder for sunnypilot/openpilot.

Revision: 2026-09-23 v5 (Astra v2 + community cross-check fields integrated)

What this logger records
------------------------
1) Full raw CAN bus 0/1/2 for each event.
2) Compact radar-focus raw CSV.
3) Decoded 24-byte object banks on bus0:
     group A: 0x241-0x24F (left-side display teacher matches primarily y > 0)
     group B: 0x279-0x287 (right-side display teacher matches primarily y < 0)
   IMPORTANT: side is determined from decoded y sign, NOT from bank/address alone.
4) Rear 0x1EA LR/RR teacher candidates on bus1, including community-DBC lateral fields.
5) Previous FR_CMR/front-corner object candidates on bus2, including age/vy/ax fields
   cross-checked against the community Hyundai CAN-FD corner-radar DBC.
6) Raw focus coverage widened to include 0x235-0x24F for protocol-family comparison.
7) Unresolved repeated-record banks 0x270-0x277 and 0x288-0x28F for the next decode step.

Current empirical fields for 24-byte object slots
--------------------------------------------------
  counter candidate : bit 16, 8-bit uint
  object ID candidate: bit 24, 8-bit uint (bank-local, may be recycled)
  x candidate       : bit 64, 12-bit signed, 0.1 m
  y candidate       : bit 76, 11-bit signed, 0.1 m, +LEFT / -RIGHT
  vx candidate      : bit 87, 10/11-bit signed candidates, 0.1 m/s
  inactive x marker : x raw == 0x7FF

Rear teacher relation observed in the 2026-09-23 capture:
  D_LR/RR(t) ~= 2.1 - x_raw(object, t - 0.20 s)
The 2.1 m and 0.20 s values are empirical alignment terms, not calibrated vehicle
geometry or confirmed ADAS latency. This logger does NOT delay raw decoding by 0.20 s.

Still unresolved and intentionally preserved as raw data
--------------------------------------------------------
- 24-byte-bank lateral velocity vy, acceleration, class/quality/size/state fields
- full invalid/CRC rules
- exact physical sensor/ECU ownership
- 0x270-0x277 / 0x288-0x28F repeated record physical meaning
- global identity / duplicate fusion across A/B/FR_CMR banks

Event capture
-------------
BSD event:
  - 20 s pre-trigger
  - full BSD-active interval
  - 20 s post-trigger
  - independent left/right captures

SCC/HDA2 event:
  - mainCruise ON -> 60 s capture
  - fallback to cruiseState.available/enabled rising edge when needed

Output example
--------------
/data/radar/2026-09-23_21-30-00/
  L_radar_bus0_21_30_00.csv
  L_radar_bus1_21_30_00.csv
  L_radar_bus2_21_30_00.csv
  L_focus_21_30_00.csv
  L_objects_decoded_21_30_00.csv
  L_teacher_rear_21_30_00.csv
  L_front_decoded_21_30_00.csv
  L_repeated_raw_21_30_00.csv
  decoder_info.txt

This program is RECEIVE-ONLY. It never transmits CAN messages.
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


VERSION = "2026-09-23-v5-astra-v2-community-check"
ROOT_DIR = Path("/data/radar")
BUS_IDS = (0, 1, 2)
PRE_TRIGGER_S = 20.0
POST_TRIGGER_S = 20.0
SCC_CAPTURE_S = 60.0
CAN_SOCKET_TIMEOUT_MS = 20
FLUSH_INTERVAL_S = 1.0
CARSTATE_POLL_INTERVAL_S = 0.05  # 20 Hz
STATUS_LOG = ROOT_DIR / "bsm_can_logger.log"

LEFT_BANK = set(range(0x241, 0x250))
RIGHT_BANK = set(range(0x279, 0x288))
OBJECT_BANKS = LEFT_BANK | RIGHT_BANK
REPEATED_BANK = set(range(0x270, 0x278)) | set(range(0x288, 0x290))
FR_OBJECT_ADDRS = {0x180, 0x181, 0x182, 0x183, 0x184,
                   0x1B6, 0x1B7, 0x1B8, 0x1B9, 0x1FB}
RECORD_OFFSETS = (3, 8, 12, 16, 20, 24, 28)
TEACHER_OFFSET_M = 2.1
TEACHER_DELAY_S_OBSERVED = 0.20

# Keep unresolved neighbor/control frames too.
FOCUS_IDS = (
  FR_OBJECT_ADDRS |
  {0x1E0, 0x1EA} |
  set(range(0x210, 0x220)) |
  set(range(0x235, 0x250)) |
  set(range(0x270, 0x290)) |
  set(range(0x2BA, 0x2BF))
)

ButtonType = car.CarState.ButtonEvent.Type


def bits_le(data: bytes, start: int, width: int) -> int:
  return (int.from_bytes(data, "little") >> start) & ((1 << width) - 1)


def signed_value(value: int, width: int) -> int:
  return value - (1 << width) if value & (1 << (width - 1)) else value


def decode_object(address: int, data: bytes) -> Optional[dict]:
  """Decode an empirical 24-byte A/B object slot.

  Never assign geometric side from the bank alone. Both banks can contain
  objects with either y sign.
  """
  if address not in OBJECT_BANKS or len(data) != 24:
    return None

  xr = bits_le(data, 64, 12)
  if xr == 0x7FF or not any(data[3:8]):
    return None

  bank = "A_241_24F" if address in LEFT_BANK else "B_279_287"
  slot = address - (0x241 if address in LEFT_BANK else 0x279) + 1

  x = signed_value(xr, 12) * 0.1
  y = signed_value(bits_le(data, 76, 11), 11) * 0.1
  v10 = signed_value(bits_le(data, 87, 10), 10) * 0.1
  v11 = signed_value(bits_le(data, 87, 11), 11) * 0.1
  velocity_ambiguous = abs(v11 - v10) > 1e-9

  side = "LEFT" if y > 0.0 else "RIGHT" if y < 0.0 else "CENTER"
  lateral_zone = "LEFT" if y > 0.8 else "RIGHT" if y < -0.8 else "CENTER"
  xref = x - TEACHER_OFFSET_M
  longitudinal_zone = "FRONT" if xref > 0.5 else "REAR" if xref < -0.5 else "ALONGSIDE"
  primary = ((bank == "A_241_24F" and lateral_zone == "LEFT") or
             (bank == "B_279_287" and lateral_zone == "RIGHT"))

  return {
    "bank": bank,
    "slot": slot,
    "counter_raw": data[2],
    "object_id_raw": data[3],
    "bank_id_key": f"{bank}:{data[3]:02X}",
    "x_raw_candidate_m": round(x, 3),
    "y_left_candidate_m": round(y, 3),
    "vx_candidate_mps": "" if velocity_ambiguous else round(v11, 3),
    "vx_width_ambiguous": int(velocity_ambiguous),
    "vx_signed10_candidate_mps": round(v10, 3),
    "vx_signed11_candidate_mps": round(v11, 3),
    "geometric_side": side,
    "lateral_zone": lateral_zone,
    "x_teacher_reference_candidate_m": round(xref, 3),
    "longitudinal_zone_candidate": longitudinal_zone,
    "primary_for_side": int(primary),
  }


def decode_teacher_rear(data: bytes) -> list[dict]:
  """Decode bus1 0x1EA LR/RR teacher values.

  Distance/state handling preserves the Astra-v2 empirical mapping used for the
  2026-09-23 validation. Lateral fields are additionally exposed from the
  community Hyundai CAN-FD DBC for independent y-coordinate cross-checking.
  """
  if len(data) != 32:
    return []

  out = []
  for sector, dist_start, lateral_start, state_start in (
      ("LR", 139, 152, 160),
      ("RR", 163, 172, 184),
  ):
    status = bits_le(data, state_start, 3)
    distance = bits_le(data, dist_start, 8) * 0.1
    lateral = bits_le(data, lateral_start, 6) * 0.1
    out.append({
      "sector": sector,
      "status_raw": status,
      "distance_candidate_m": round(distance, 3),
      "lateral_dbc_candidate_m": round(lateral, 3),
      "teacher_usable": int(status == 1 and 0.5 <= distance <= 19.5),
      "ceiling_candidate": int(distance >= 20.0),
    })
  return out


def decode_front_pair(address: int, data: bytes) -> list[dict]:
  """Preserve earlier FR_CMR front-perception mapping for track-link study."""
  if address not in FR_OBJECT_ADDRS or len(data) != 32:
    return []

  out = []
  ordered = (0x180, 0x181, 0x182, 0x183, 0x184,
             0x1B6, 0x1B7, 0x1B8, 0x1B9, 0x1FB)
  try:
    addr_index = ordered.index(address)
  except ValueError:
    return []

  for sub in (0, 1):
    off = sub * 128
    quality = bits_le(data, off + 24, 7)
    age = bits_le(data, off + 32, 8)
    x = bits_le(data, off + 64, 13) * 0.05
    y = bits_le(data, off + 78, 12) * 0.05 - 102.4
    vx = bits_le(data, off + 91, 12) * 0.05 - 100.0
    vy = bits_le(data, off + 104, 10) * 0.05 - 25.0
    ax = signed_value(bits_le(data, off + 115, 9), 9) * 0.05
    if not (quality > 0 and 0 <= x < 180 and abs(y) < 40 and vx > -99):
      continue
    out.append({
      "slot": addr_index * 2 + sub + 1,
      "object_id_raw": bits_le(data, off + 44, 7),
      "quality": quality,
      "age": age,
      # This 3-bit field existed in the previous empirical mapping but is not
      # confirmed as an object class by the community DBC. Preserve it only
      # as a candidate for offline comparison.
      "class_id_candidate": bits_le(data, off + 60, 3),
      "x_candidate_m": round(x, 3),
      "y_left_candidate_m": round(y, 3),
      "vx_candidate_mps": round(vx, 3),
      "vy_candidate_mps": round(vy, 3),
      "ax_candidate_mps2": round(ax, 3),
      "geometric_side": "LEFT" if y > 0 else "RIGHT" if y < 0 else "CENTER",
    })
  return out


def decode_repeated(address: int, data: bytes) -> list[dict]:
  """Expose unresolved repeated records without assigning physical units."""
  if address not in REPEATED_BANK or len(data) != 32:
    return []

  out = []
  for idx, start in enumerate(RECORD_OFFSETS):
    if start + 3 >= len(data):
      continue
    out.append({
      "record": idx,
      "byte_start": start,
      "value_u16_raw": int.from_bytes(data[start:start + 2], "little"),
      "tag_raw": data[start + 2],
      "tail_raw": data[start + 3],
      "sentinel_8000_candidate": int(data[start:start + 2] == b"\x40\x1f"),
    })
  return out


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
  cruise_available: bool
  cruise_enabled: bool
  ego_speed_mps: float
  hda_mode2_raw: int
  hda_cntrl_mod_raw: int


class CsvCaptureBase:
  RAW_HEADER = [
    "wall_time", "recv_mono_ns", "log_mono_ns", "rel_trigger_s",
    "bus", "address_hex", "address_dec", "dlc", "data_hex",
    "left_bsd", "right_bsd", "scc_main_on", "cruise_available",
    "cruise_enabled", "ego_speed_mps", "hda_mode2_raw", "hda_cntrl_mod_raw",
  ]

  BASE_DECODE_HEADER = [
    "wall_time", "recv_mono_ns", "log_mono_ns", "rel_trigger_s",
    "bus", "address_hex", "address_dec", "dlc", "raw_data_hex",
    "left_bsd", "right_bsd", "scc_main_on", "cruise_available",
    "cruise_enabled", "ego_speed_mps", "hda_mode2_raw", "hda_cntrl_mod_raw",
  ]

  OBJECT_HEADER = BASE_DECODE_HEADER + [
    "bank", "slot", "counter_raw", "object_id_raw", "bank_id_key",
    "x_raw_candidate_m", "y_left_candidate_m", "vx_candidate_mps",
    "vx_width_ambiguous", "vx_signed10_candidate_mps",
    "vx_signed11_candidate_mps", "geometric_side", "lateral_zone",
    "x_teacher_reference_candidate_m", "longitudinal_zone_candidate",
    "primary_for_side",
  ]

  TEACHER_HEADER = BASE_DECODE_HEADER + [
    "sector", "status_raw", "distance_candidate_m",
    "lateral_dbc_candidate_m", "teacher_usable",
    "ceiling_candidate", "teacher_delay_s_observed",
    "teacher_offset_m_observed",
  ]

  FRONT_HEADER = BASE_DECODE_HEADER + [
    "slot", "object_id_raw", "quality", "age", "class_id_candidate",
    "x_candidate_m", "y_left_candidate_m", "vx_candidate_mps",
    "vy_candidate_mps", "ax_candidate_mps2", "geometric_side",
  ]

  REPEATED_HEADER = BASE_DECODE_HEADER + [
    "record", "byte_start", "value_u16_raw", "tag_raw", "tail_raw",
    "sentinel_8000_candidate",
  ]

  def __init__(self, trigger_mono_ns: int):
    self.trigger_mono_ns = trigger_mono_ns
    self.last_flush_ns = trigger_mono_ns

    self.files: Dict[int, TextIO] = {}
    self.writers: Dict[int, csv.writer] = {}

    self.focus_file: Optional[TextIO] = None
    self.focus_writer: Optional[csv.writer] = None

    self.objects_file: Optional[TextIO] = None
    self.objects_writer: Optional[csv.DictWriter] = None
    self.teacher_file: Optional[TextIO] = None
    self.teacher_writer: Optional[csv.DictWriter] = None
    self.front_file: Optional[TextIO] = None
    self.front_writer: Optional[csv.DictWriter] = None
    self.repeated_file: Optional[TextIO] = None
    self.repeated_writer: Optional[csv.DictWriter] = None

  def _raw_values(self, row: CanRow) -> list[object]:
    wall_dt = datetime.fromtimestamp(row.wall_time_ns / 1e9)
    rel_s = (row.recv_mono_ns - self.trigger_mono_ns) / 1e9
    return [
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
      int(row.cruise_available),
      int(row.cruise_enabled),
      f"{row.ego_speed_mps:.3f}",
      row.hda_mode2_raw,
      row.hda_cntrl_mod_raw,
    ]

  def _decode_base(self, row: CanRow) -> dict:
    wall_dt = datetime.fromtimestamp(row.wall_time_ns / 1e9)
    rel_s = (row.recv_mono_ns - self.trigger_mono_ns) / 1e9
    return {
      "wall_time": wall_dt.isoformat(timespec="milliseconds"),
      "recv_mono_ns": row.recv_mono_ns,
      "log_mono_ns": row.log_mono_ns,
      "rel_trigger_s": f"{rel_s:.6f}",
      "bus": row.bus,
      "address_hex": f"0x{row.address:X}",
      "address_dec": row.address,
      "dlc": len(row.data),
      "raw_data_hex": row.data.hex().upper(),
      "left_bsd": int(row.left_bsd),
      "right_bsd": int(row.right_bsd),
      "scc_main_on": int(row.scc_main_on),
      "cruise_available": int(row.cruise_available),
      "cruise_enabled": int(row.cruise_enabled),
      "ego_speed_mps": f"{row.ego_speed_mps:.3f}",
      "hda_mode2_raw": row.hda_mode2_raw,
      "hda_cntrl_mod_raw": row.hda_cntrl_mod_raw,
    }

  def _open_aux_files(self, event_dir: Path, prefix: str, hhmmss: str) -> None:
    focus_path = event_dir / f"{prefix}_focus_{hhmmss}.csv"
    self.focus_file = open(focus_path, "w", newline="", buffering=1024 * 1024)
    self.focus_writer = csv.writer(self.focus_file)
    self.focus_writer.writerow(self.RAW_HEADER)

    self.objects_file = open(event_dir / f"{prefix}_objects_decoded_{hhmmss}.csv",
                             "w", newline="", buffering=1024 * 1024)
    self.objects_writer = csv.DictWriter(self.objects_file, fieldnames=self.OBJECT_HEADER)
    self.objects_writer.writeheader()

    self.teacher_file = open(event_dir / f"{prefix}_teacher_rear_{hhmmss}.csv",
                             "w", newline="", buffering=1024 * 1024)
    self.teacher_writer = csv.DictWriter(self.teacher_file, fieldnames=self.TEACHER_HEADER)
    self.teacher_writer.writeheader()

    self.front_file = open(event_dir / f"{prefix}_front_decoded_{hhmmss}.csv",
                           "w", newline="", buffering=1024 * 1024)
    self.front_writer = csv.DictWriter(self.front_file, fieldnames=self.FRONT_HEADER)
    self.front_writer.writeheader()

    self.repeated_file = open(event_dir / f"{prefix}_repeated_raw_{hhmmss}.csv",
                              "w", newline="", buffering=1024 * 1024)
    self.repeated_writer = csv.DictWriter(self.repeated_file, fieldnames=self.REPEATED_HEADER)
    self.repeated_writer.writeheader()

    info = event_dir / "decoder_info.txt"
    if not info.exists():
      with open(info, "w") as f:
        f.write(f"version={VERSION}\n")
        f.write("object_bank_A=0x241-0x24F,24bytes\n")
        f.write("object_bank_B=0x279-0x287,24bytes\n")
        f.write("side_rule=decoded_y_sign_not_bank_address\n")
        f.write("x=bit64,width12,signed,scale0.1m\n")
        f.write("y=bit76,width11,signed,scale0.1m,+left\n")
        f.write("vx=bit87,width10_or_11,signed,scale0.1mps,upper_bit_unconfirmed\n")
        f.write("inactive_x_raw=0x7FF\n")
        f.write(f"teacher_offset_m_observed={TEACHER_OFFSET_M}\n")
        f.write(f"teacher_delay_s_observed={TEACHER_DELAY_S_OBSERVED}\n")
        f.write("teacher=bus1:0x1EA LR/RR empirical distance/state + community-DBC lateral\n")
        f.write("teacher_lateral=LR bit152 width6 scale0.1; RR bit172 width6 scale0.1\n")
        f.write("front_180_184=quality,age,id,x,y,vx,vy,ax community-DBC cross-check fields\n")
        f.write("front_class_id=unconfirmed candidate only\n")
        f.write("focus_extra=0x235-0x23F retained for 32-byte corner-radar family comparison\n")
        f.write("unresolved=24B_vy,class,quality,size,state,full_invalid_crc,sensor_ownership,global_track_fusion,repeated_bank_units\n")

  def write_row(self, row: CanRow) -> None:
    raw_values = self._raw_values(row)

    writer = self.writers.get(row.bus)
    if writer is not None:
      writer.writerow(raw_values)

    if self.focus_writer is not None and row.address in FOCUS_IDS:
      self.focus_writer.writerow(raw_values)

    base = None

    if row.bus == 0 and row.address in OBJECT_BANKS and self.objects_writer is not None:
      obj = decode_object(row.address, row.data)
      if obj is not None:
        base = self._decode_base(row)
        base.update(obj)
        self.objects_writer.writerow(base)

    if row.bus == 1 and row.address == 0x1EA and self.teacher_writer is not None:
      for item in decode_teacher_rear(row.data):
        d = self._decode_base(row)
        d.update(item)
        d["teacher_delay_s_observed"] = TEACHER_DELAY_S_OBSERVED
        d["teacher_offset_m_observed"] = TEACHER_OFFSET_M
        self.teacher_writer.writerow(d)

    if row.bus == 2 and row.address in FR_OBJECT_ADDRS and self.front_writer is not None:
      for item in decode_front_pair(row.address, row.data):
        d = self._decode_base(row)
        d.update(item)
        self.front_writer.writerow(d)

    if row.bus == 0 and row.address in REPEATED_BANK and self.repeated_writer is not None:
      for item in decode_repeated(row.address, row.data):
        d = self._decode_base(row)
        d.update(item)
        self.repeated_writer.writerow(d)

  def flush(self, now_mono_ns: Optional[int] = None, force: bool = False) -> None:
    if now_mono_ns is None:
      now_mono_ns = time.monotonic_ns()
    if not force and (now_mono_ns - self.last_flush_ns) < int(FLUSH_INTERVAL_S * 1e9):
      return

    for f in self._all_files():
      try:
        f.flush()
      except OSError:
        pass
    self.last_flush_ns = now_mono_ns

  def _all_files(self) -> list[TextIO]:
    result = list(self.files.values())
    for f in (self.focus_file, self.objects_file, self.teacher_file,
              self.front_file, self.repeated_file):
      if f is not None:
        result.append(f)
    return result

  def close(self) -> None:
    for f in self._all_files():
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
    self.side = side
    self.trigger_dt = trigger_dt
    self.tail_deadline_ns: Optional[int] = None

    self.event_dir = self._make_unique_event_dir(trigger_dt, side)
    self.event_dir.mkdir(parents=True, exist_ok=True)

    hhmmss = trigger_dt.strftime("%H_%M_%S")
    for bus in BUS_IDS:
      path = self.event_dir / f"{side}_radar_bus{bus}_{hhmmss}.csv"
      f = open(path, "w", newline="", buffering=1024 * 1024)
      writer = csv.writer(f)
      writer.writerow(self.RAW_HEADER)
      self.files[bus] = f
      self.writers[bus] = writer

    self._open_aux_files(self.event_dir, side, hhmmss)

    # Snapshot the already-collected PRE_TRIGGER_S history.
    for row in prebuffer:
      self.write_row(row)

    self.flush(force=True)

  @staticmethod
  def _make_unique_event_dir(trigger_dt: datetime, side: str) -> Path:
    base = ROOT_DIR / trigger_dt.strftime("%Y-%m-%d_%H-%M-%S")
    if not base.exists():
      return base

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
      writer.writerow(self.RAW_HEADER)
      self.files[bus] = f
      self.writers[bus] = writer

    self._open_aux_files(self.event_dir, "scc", hhmmss)

    with open(self.event_dir / "trigger.txt", "w") as f:
      f.write(f"version={VERSION}\n")
      f.write(f"trigger_reason={trigger_reason}\n")
      f.write(f"trigger_time={trigger_dt.isoformat(timespec='milliseconds')}\n")
      f.write(f"capture_seconds={SCC_CAPTURE_S:.1f}\n")

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


def _button_is_main_cruise_pressed(cs) -> bool:
  for b in cs.buttonEvents:
    try:
      if bool(b.pressed) and b.type.raw == ButtonType.mainCruise:
        return True
    except Exception:
      try:
        if bool(b.pressed) and str(b.type) == "mainCruise":
          return True
      except Exception:
        pass
  return False


def main() -> None:
  global running

  ROOT_DIR.mkdir(parents=True, exist_ok=True)
  signal.signal(signal.SIGTERM, _signal_handler)
  signal.signal(signal.SIGINT, _signal_handler)

  # Raw CAN must not be conflated: every published CAN Event is relevant.
  can_sock = messaging.sub_sock("can", timeout=CAN_SOCKET_TIMEOUT_MS, conflate=False)
  carstate_sock = messaging.sub_sock("carState", conflate=True)

  prebuffer: Deque[CanRow] = deque()
  bsd_captures: Dict[str, BsdCapture] = {}
  scc_capture: Optional[SccCapture] = None

  prev_left = False
  prev_right = False
  cur_left = False
  cur_right = False
  cur_cruise_available = False
  cur_cruise_enabled = False
  cur_ego_speed_mps = 0.0

  # HDA teacher/state hints must come from bus1 only.
  hda_mode2_raw = -1
  hda_cntrl_mod_raw = -1

  last_carstate_poll_ns = 0

  # mainCruise is momentary; track local toggle state.
  scc_main_on = False
  saw_main_button_event = False

  carstate_initialized = False
  prev_cruise_available = False
  prev_cruise_enabled = False

  log_status(
    f"START version={VERSION} buses={BUS_IDS} "
    f"BSD(pre={PRE_TRIGGER_S:.1f}s post={POST_TRIGGER_S:.1f}s) "
    f"SCC={SCC_CAPTURE_S:.1f}s teacher=bus1:0x1EA "
    f"object_banks=A241-24F/B279-287 root={ROOT_DIR}"
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

          # Raw HDA state values as observed in the supplied G80 logs.
          if bus == 1 and address == 0x1EA and len(data) >= 5:
            hda_mode2_raw = data[4] & 0x07
          elif bus == 1 and address == 0x1E0 and len(data) >= 4:
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
            cruise_available=cur_cruise_available,
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

      # Keep approximately PRE_TRIGGER_S seconds of full CAN history.
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
          cruise_available = bool(cs.cruiseState.available)
          cruise_enabled = bool(cs.cruiseState.enabled)
          cur_cruise_available = cruise_available
          cur_cruise_enabled = cruise_enabled
          cur_ego_speed_mps = float(cs.vEgo)

          # Independent left/right BSD captures.
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

          main_pressed = _button_is_main_cruise_pressed(cs)
          if main_pressed:
            saw_main_button_event = True
            scc_main_on = not scc_main_on
            if scc_main_on:
              if scc_capture is None:
                cap = SccCapture(datetime.now(), recv_mono_ns, "mainCruise ON")
                scc_capture = cap
                log_status(
                  f"SCC MAIN ON -> {SCC_CAPTURE_S:.0f}s capture opened: {cap.event_dir}"
                )
              else:
                log_status("SCC MAIN ON while SCC capture active -> existing capture kept")
            else:
              log_status("SCC MAIN OFF")

          # Fallback only if mainCruise ButtonEvent is unavailable on this port.
          if carstate_initialized and not saw_main_button_event and scc_capture is None:
            if cruise_available and not prev_cruise_available:
              cap = SccCapture(datetime.now(), recv_mono_ns,
                               "cruiseState.available rising fallback")
              scc_capture = cap
              log_status(
                f"SCC AVAILABLE rising fallback -> {SCC_CAPTURE_S:.0f}s capture opened: {cap.event_dir}"
              )
            elif cruise_enabled and not prev_cruise_enabled:
              cap = SccCapture(datetime.now(), recv_mono_ns,
                               "cruiseState.enabled rising fallback")
              scc_capture = cap
              log_status(
                f"SCC ENABLED rising fallback -> {SCC_CAPTURE_S:.0f}s capture opened: {cap.event_dir}"
              )

          prev_cruise_available = cruise_available
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

      # Flush and close fixed-duration SCC capture.
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

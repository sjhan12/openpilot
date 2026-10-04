#!/usr/bin/env python3
from __future__ import annotations

"""G80 V50 event-triggered ML case collector.

Design goals
------------
* NO continuous training-data writes.
* Keep only a short history in RAM.
* Fast 10 Hz training snapshot + 2 Hz diagnostic context.
* Disk/gzip work is deferred to a background writer after the post window.
* A keypad press creates one labelled case containing PRE seconds before the
  press and POST seconds after the press.
* Runs inside V50 g80radard/live_service so training data is exactly the V50
  state that a later inference module can consume.
* Collector failures must never stop V50 radar monitoring.

Default key map (Linux EV_KEY codes):
  F13 183 = LEFT SAFE
  F14 184 = LEFT CHECK
  F15 185 = LEFT DANGER
  F16 186 = RIGHT SAFE
  F17 187 = RIGHT CHECK
  F18 188 = RIGHT DANGER
"""

import uuid
import math
import csv
import glob
import gzip
import json
import os
import queue
import select
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

SCHEMA = "g80_v50_ml_case_v1"
COLLECTOR_VERSION = 5

EV_KEY = 0x01
KEY_F13 = 183
KEY_F14 = 184
KEY_F15 = 185
KEY_F16 = 186
KEY_F17 = 187
KEY_F18 = 188

DEFAULT_KEYMAP = {
  KEY_F13: ("LEFT", "SAFE"),
  KEY_F14: ("LEFT", "CHECK"),
  KEY_F15: ("LEFT", "DANGER"),
  KEY_F16: ("RIGHT", "SAFE"),
  KEY_F17: ("RIGHT", "CHECK"),
  KEY_F18: ("RIGHT", "DANGER"),
}

EGO_EXTRA_FIELDS = ('steeringRateDeg', 'steeringTorque', 'steeringPressed',
                    'yawRate', 'lateralAcceleration', 'brakePressed', 'gasPressed')
EGO_BOOL_FIELDS = {'steeringPressed', 'brakePressed', 'gasPressed'}


# Native Linux struct input_event on comma four/aarch64:
#   struct timeval { long sec; long usec; }; u16 type; u16 code; s32 value
_INPUT_EVENT = struct.Struct("@llHHi")


def _env_number(name, default, low, high):
  try:
    value = float(os.getenv(name, str(default)))
    return max(low, min(high, value)) if math.isfinite(value) else default
  except (ValueError, TypeError):
    return default


def _finite_or_none(v: Any):
  try:
    x = float(v)
    if x == x and abs(x) != float("inf"):
      return x
  except Exception:
    pass
  return None


def _plain(v: Any):
  """Convert capnp-ish/tuple values to JSON-safe plain Python values."""
  if isinstance(v, float) and not math.isfinite(v):
    return None
  if v is None or isinstance(v, (str, int, float, bool)):
    return v
  if isinstance(v, dict):
    return {str(k): _plain(x) for k, x in v.items()}
  if isinstance(v, (list, tuple, set)):
    return [_plain(x) for x in v]
  try:
    return float(v)
  except Exception:
    return str(v)


def _compact_object(o: dict) -> dict:
  """Keep V50 identity, geometry, motion and provenance fields for ML/replay."""
  # Full trajectories are intentionally retained: approaching/cut-in history is
  # valuable for training, and data is written only on labelled events.
  keys = (
    'key','source','sensor','sector','front_sector','x','y','vx','vy','ax','ay','confidence','recv_ns',
    'vehicle_id','vehicle_key','vehicle_anchor_key','canonical_id','canonical_key','canonical_valid',
    'canonical_age_frames','canonical_track_duration_s','canonical_match_reason','canonical_match_cost',
    'canonical_reacquired','canonical_reacquire_count','canonical_domains','canonical_primary_domain',
    'canonical_domain_history','canonical_source_transition','canonical_handoff_count','source_mask','source_age_ms',
    'display_state','vehicle_member_count','vehicle_cluster_sources','vehicle_span_x_m','vehicle_span_y_m',
    'vehicle_merge_reason','vehicle_latest_recv_ns','vehicle_cluster_keys','vehicle_duplicates_merged','vehicle_footprint_merged','front_link','corner_link_id','corner_fused_id',
    'camera_confirmed','camera_prob','camera_id','camera_key','camera_only','sensor_fusion','camera_match_cost',
    'camera_dx_m','camera_dy_m','camera_dv_mps','teacher_match','rear_teacher_confirmed','rear_teacher_sector',
    'rear_teacher_distance_m','rear_teacher_predicted_distance_m','scc_teacher_confirmed',
    'road_s','road_d','road_path_x','road_path_y','road_lane_index','road_lane','road_lane_source',
    'road_projection_valid','road_projection_endpoint_overshoot_m',
    'kalman_valid','kalman_track_key','kalman_age_frames','kalman_age_s','kf_reset_suspect','kf_dormant_preserved',
    'kf_x','kf_y','kf_vx','kf_vy','kf_ax','kf_ay','kf_x_sigma','kf_y_sigma','kf_vx_sigma','kf_vy_sigma',
    'kf_frenet_valid','kf_s','kf_s_dot','kf_s_ddot','kf_d','kf_d_dot','kf_d_ddot','kf_s_sigma','kf_d_sigma',
    'kf_s_dot_sigma','kf_d_dot_sigma','kf_lane_index','kf_lane','kf_ttlc_s','kf_lateral_motion','kf_motion_confident',
    'kf_lateral_candidate','kf_low_speed_lateral_candidate','kf_cutin_speed_class','kf_cutin_candidate',
    'kf_cutin_confirmed','kf_cutin_score','kf_cutin_persistence_s','kf_lateral_prediction_mode',
    'kf_lateral_prediction_limited','kalman_trajectory',
    'imm_valid','imm_api_version','imm_track_key','imm_age_frames','imm_age_s','imm_coord_source','imm_reset_suspect',
    'imm_reset_count','imm_reinit_count','imm_reinit_reason','imm_eval_age_ms','imm_interaction_relevant',
    'imm_skipped_reason','imm_s','imm_s_dot','imm_s_ddot','imm_d','imm_d_dot','imm_d_ddot','imm_s_sigma',
    'imm_d_sigma','imm_d_dot_sigma','imm_prob_cv','imm_prob_ca','imm_prob_maneuver','imm_dominant_model',
    'imm_lane_index','imm_lane','imm_ttlc_s','imm_motion_confident','imm_maneuver_candidate',
    'raw_address','raw_slot','stage_origin','stage_gate','preview_quality',
  )
  return {k: _plain(o.get(k)) for k in keys if k in o and o.get(k) is not None}


def _compact_frame(core: dict, raw_objects, filtered_objects, now_ns: int) -> dict:
  """Snapshot inputs useful to a later PC-trained V50 inference model."""
  return {
    'type': 'frame',
    'mono_ns': int(now_ns),
    'runtime_versions': _plain(core.get('runtime_versions', {})),
    'runtime_mismatch': bool(core.get('runtime_mismatch', False)),
    'coordinate_frame': _plain(core.get('coordinate_frame', {})),
    'ego_state': _plain(core.get('ego_state', {})),
    'road_model': _plain(core.get('road_model', {})),
    'sensor_fused_objects': [_compact_object(o) for o in (core.get('sensor_fused_objects', []) or [])],
    'radar_fused_objects': [_compact_object(o) for o in (core.get('radar_fused_objects', []) or [])],
    'front_sensor_objects': [_compact_object(o) for o in (core.get('front_sensor_objects', []) or [])],
    'corner_fused_objects': [_compact_object(o) for o in (core.get('corner_fused_objects', []) or [])],
    'camera_leads': [_compact_object(o) for o in (core.get('camera_leads', []) or [])],
    # V50 raw/validity-gated radar snapshots are supplied directly by live_service,
    # so this collector does not need to keep the browser awake.
    'raw_objects': [_compact_object(o) for o in (raw_objects or [])],
    'filtered_objects': [_compact_object(o) for o in (filtered_objects or [])],
    'future_gap': _plain(core.get('future_gap', {})),
    'zones': _plain(core.get('zones', {})),
    'scc_teacher': _plain(core.get('scc_teacher', {})),
    'teacher_rear': _plain(core.get('teacher_rear', [])),
    'canonical_tracker_stats': _plain(core.get('canonical_tracker_stats', {})),
    'kalman_motion_stats': _plain(core.get('kalman_motion_stats', {})),
    'imm_motion_stats': _plain(core.get('imm_motion_stats', {})),
    'camera_fusion_stats': _plain(core.get('camera_fusion_stats', {})),
    'corner_fusion_stats': _plain(core.get('corner_fusion_stats', {})),
  }


@dataclass
class KeyPress:
  mono_ns: int
  code: int
  side: str
  label: str
  device: str = ''


class _InputThread(threading.Thread):
  """Read F13..F18 from any Linux HID keyboard without python-evdev."""
  def __init__(self, out_q: queue.SimpleQueue, keymap: dict[int, tuple[str, str]], stop_evt: threading.Event):
    super().__init__(name='g80-ml-keypad', daemon=True)
    self.out_q = out_q
    self.keymap = keymap
    self.stop_evt = stop_evt
    self.fds: dict[int, tuple[Any, str]] = {}
    self.last_scan = 0.0
    self.last_press_ns: dict[int, int] = {}

  def _scan(self):
    now = time.monotonic()
    if now - self.last_scan < 2.0:
      return
    self.last_scan = now
    candidates = []
    candidates.extend(glob.glob('/dev/input/by-id/*event-kbd'))
    candidates.extend(glob.glob('/dev/input/event*'))
    existing_real = {os.path.realpath(name) for _, name in self.fds.values()}
    for name in candidates:
      real = os.path.realpath(name)
      if real in existing_real:
        continue
      try:
        fd = os.open(real, os.O_RDONLY | os.O_NONBLOCK)
        self.fds[fd] = (fd, real)
        existing_real.add(real)
      except OSError:
        pass

  def _drop(self, fd: int):
    item = self.fds.pop(fd, None)
    if item:
      try:
        os.close(fd)
      except OSError:
        pass

  def run(self):
    while not self.stop_evt.is_set():
      self._scan()
      fds = list(self.fds.keys())
      if not fds:
        self.stop_evt.wait(0.25)
        continue
      try:
        readable, _, _ = select.select(fds, [], [], 0.25)
      except Exception:
        for fd in fds:
          self._drop(fd)
        continue
      for fd in readable:
        name = self.fds.get(fd, (None, ''))[1]
        try:
          data = os.read(fd, _INPUT_EVENT.size * 32)
        except BlockingIOError:
          continue
        except OSError:
          self._drop(fd)
          continue
        if not data:
          self._drop(fd)
          continue
        off = 0
        while off + _INPUT_EVENT.size <= len(data):
          _sec, _usec, etype, code, value = _INPUT_EVENT.unpack_from(data, off)
          off += _INPUT_EVENT.size
          if etype != EV_KEY or value != 1 or code not in self.keymap:
            continue
          now_ns = time.monotonic_ns()
          # Suppress accidental switch bounce / receiver duplicate packets.
          if now_ns - self.last_press_ns.get(code, 0) < 250_000_000:
            continue
          self.last_press_ns[code] = now_ns
          side, label = self.keymap[code]
          self.out_q.put(KeyPress(now_ns, int(code), side, label, name))



def _compact_road_model(road: dict) -> dict:
  """Keep only lane/path geometry needed by the ML model at the fast rate."""
  road = road or {}
  keys = (
    'valid','fresh','age_ms','curve_direction','path','lane_lines','road_edges',
    'lane_line_probs','road_edge_stds','confident_lane_lines','path_x_min_m','path_x_max_m',
    'path_y_20m','path_y_40m','path_y_60m','lane_width_m','left_lane_width_m','right_lane_width_m',
  )
  return {k: _plain(road.get(k)) for k in keys if k in road and road.get(k) is not None}


def _compact_gap_side(side: dict) -> dict:
  side = side or {}
  evidence = side.get('fg12_evidence') or side.get('fg11_evidence') or side.get('fg10_evidence') or {}
  return {
    'target_lane': side.get('target_lane'),
    'target_lane_index': side.get('target_lane_index'),
    'incoming_count': side.get('incoming_count'),
    'stable_incoming_count': side.get('stable_incoming_count'),
    'possible_incoming_count': side.get('possible_incoming_count'),
    'min_front_clearance_during_ego_overlap_m': side.get('min_front_clearance_during_ego_overlap_m'),
    'min_rear_clearance_during_ego_overlap_m': side.get('min_rear_clearance_during_ego_overlap_m'),
    'min_boundary_clearance_during_ego_overlap_m': side.get('min_boundary_clearance_during_ego_overlap_m'),
    'min_abs_separation_during_ego_overlap_m': side.get('min_abs_separation_during_ego_overlap_m'),
    'current_front_ttc_ca_s': side.get('current_front_ttc_ca_s'),
    'current_rear_ttc_ca_s': side.get('current_rear_ttc_ca_s'),
    'decision': _plain(side.get('decision', {})),
    'bsd_state': side.get('bsd_state'),
    'lane_availability': _plain(side.get('lane_availability', {})),
    'evidence_summary': {
      k: _plain(evidence.get(k)) for k in (
        'current_front_gap_m','current_rear_gap_m','predicted_front_min_m','predicted_rear_min_m',
        'near_future_predicted_count','near_future_confirmed_count','tts3_confirmed_count',
        'tts3_candidate_count','tts5_watch_count','conflict3_confirmed_count',
        'min_confirmed_time_to_side_s','min_candidate_time_to_side_s','min_watch_time_to_side_s',
        'min_time_to_side_s','min_2d_conflict_time_s') if evidence.get(k) is not None
    },
  }


def _compact_future_gap(fg: dict) -> dict:
  fg = fg or {}
  return {
    'api_version': fg.get('api_version'),
    'mode': fg.get('mode'),
    'decision_enabled': fg.get('decision_enabled'),
    'decision_shadow_only': fg.get('decision_shadow_only'),
    'centerline_recognition_available': fg.get('centerline_recognition_available'),
    'bsd': _plain(fg.get('bsd', {})),
    'monitor_health': _plain(fg.get('monitor_health', {})),
    'driver_intent': _plain(fg.get('driver_intent', {})),
    'lane_availability': _plain(fg.get('lane_availability', {})),
    'stats': _plain(fg.get('stats', {})),
    'left': _compact_gap_side(fg.get('left', {})),
    'right': _compact_gap_side(fg.get('right', {})),
  }


def _fast_frame(core: dict, now_ns: int) -> dict:
  """10 Hz ML input snapshot. Avoid duplicate view arrays and huge FG internals."""
  return {
    'type': 'frame',
    'frame_kind': 'training_fast',
    'mono_ns': int(now_ns),
    'runtime_versions': _plain(core.get('runtime_versions', {})),
    'runtime_mismatch': bool(core.get('runtime_mismatch', False)),
    'coordinate_frame': _plain(core.get('coordinate_frame', {})),
    'ego_state': _plain(core.get('ego_state', {})),
    'road_model': _compact_road_model(core.get('road_model', {})),
    # Canonical360 is the intended ML object input. Keep the rich per-object fields/trajectories.
    'sensor_fused_objects': [_compact_object(o) for o in (core.get('sensor_fused_objects', []) or [])],
    'future_gap_summary': _compact_future_gap(core.get('future_gap', {})),
    'scc_teacher': _plain(core.get('scc_teacher', {})),
    'teacher_rear': _plain(core.get('teacher_rear', [])),
  }


def _context_detail(core: dict, raw_objects, filtered_objects) -> dict:
  """2 Hz diagnostic snapshot retained so later fusion/debug work is still possible."""
  return {
    'raw_objects': [_compact_object(o) for o in (raw_objects or [])],
    'filtered_objects': [_compact_object(o) for o in (filtered_objects or [])],
    'radar_fused_objects': [_compact_object(o) for o in (core.get('radar_fused_objects', []) or [])],
    'front_sensor_objects': [_compact_object(o) for o in (core.get('front_sensor_objects', []) or [])],
    'corner_fused_objects': [_compact_object(o) for o in (core.get('corner_fused_objects', []) or [])],
    'camera_leads': [_compact_object(o) for o in (core.get('camera_leads', []) or [])],
    'future_gap_full': _plain(core.get('future_gap', {})),
    'zones': _plain(core.get('zones', {})),
    'canonical_tracker_stats': _plain(core.get('canonical_tracker_stats', {})),
    'kalman_motion_stats': _plain(core.get('kalman_motion_stats', {})),
    'imm_motion_stats': _plain(core.get('imm_motion_stats', {})),
    'camera_fusion_stats': _plain(core.get('camera_fusion_stats', {})),
    'corner_fusion_stats': _plain(core.get('corner_fusion_stats', {})),
  }


class _PendingCase:
  """RAM-only pending case. No filesystem/gzip calls occur on the radar loop."""
  def __init__(self, press: KeyPress, start_ns: int, end_ns: int, part_path: Path, final_path: Path,
               pre_s: float, post_s: float, build_meta: dict, session_id: str):
    self.press = press
    self.start_ns = int(start_ns)
    self.end_ns = int(end_ns)
    self.part_path = part_path
    self.final_path = final_path
    self.frame_count = 0
    self.first_frame_ns = 0
    self.last_frame_ns = 0
    self.last_written_ns = 0
    self.frames: list[dict] = []
    self.meta = {
      'type': 'case_meta', 'schema': SCHEMA, 'collector_version': COLLECTOR_VERSION,
      'session_id': session_id,
      'label_scope': 'selected_side_at_key_receipt; not every frame in window',
      'label_clock': 'userspace time.monotonic_ns at key receipt; human reaction delay unknown',
      'window_usage': {
        'prediction_input': 'frames with mono_ns <= press_mono_ns only',
        'post_trigger': 'outcome_validation_only; exclude from current-time prediction inputs',
        'post_duration_sec': float(post_s),
        'outcome_is_ground_truth': False,
        'note': 'Post frames are observations; no executed lane change or safety outcome is assumed.'
      },
      'performance_profile': {
        'training_fast_hz': None,
        'diagnostic_context_hz': None,
        'writer': 'background JSON+gzip after post window; no serialization/disk I/O in radar publish loop',
        'shadow_log_required_for_training': False,
      },
      'side': press.side, 'label': press.label, 'key_code': press.code,
      'key_device': press.device, 'press_mono_ns': int(press.mono_ns),
      'pre_sec': float(pre_s), 'post_sec': float(post_s),
      'window_start_ns': int(start_ns), 'window_end_ns': int(end_ns),
      'created_wall_time': datetime.now().astimezone().isoformat(timespec='milliseconds'),
      'build': _plain(build_meta),
      'note': 'Pre/post frames are RAM buffered. File creation/compression happens on a background writer only after capture.',
    }

  def add(self, mono_ns: int, frame: dict):
    mono_ns = int(mono_ns)
    if mono_ns < self.start_ns or mono_ns > self.end_ns or mono_ns <= self.last_written_ns:
      return
    self.frames.append(frame)
    self.frame_count += 1
    if not self.first_frame_ns:
      self.first_frame_ns = mono_ns
    self.last_frame_ns = mono_ns
    self.last_written_ns = mono_ns

  def finish_record(self, truncated: bool = False) -> dict:
    return {
      'type': 'case_end', 'frame_count': int(self.frame_count), 'truncated': bool(truncated),
      'first_frame_ns': int(self.first_frame_ns), 'last_frame_ns': int(self.last_frame_ns),
      'actual_pre_sec': None if not self.first_frame_ns else round((self.press.mono_ns - self.first_frame_ns) / 1e9, 3),
      'actual_post_sec': None if not self.last_frame_ns else round((self.last_frame_ns - self.press.mono_ns) / 1e9, 3),
    }


class _CaseWriter(threading.Thread):
  """Background-only gzip/file writer so ML labels cannot block V50 rendering/fusion."""
  def __init__(self, owner):
    super().__init__(name='g80-ml-writer', daemon=True)
    self.owner = owner
    self.q: queue.Queue = queue.Queue(maxsize=16)

  def submit(self, p: _PendingCase, truncated: bool = False) -> bool:
    try:
      self.q.put_nowait((p, bool(truncated)))
      return True
    except queue.Full:
      self.owner.last_error = 'ML writer queue full; case not saved'
      return False

  def _manifest(self, p: _PendingCase):
    path = self.owner.base_dir / 'manifest.csv'
    new = not path.exists()
    self.owner.base_dir.mkdir(parents=True, exist_ok=True)
    with path.open('a', newline='', encoding='utf-8') as f:
      w = csv.writer(f)
      if new:
        w.writerow(['file','side','label','press_mono_ns','frames','actual_pre_sec','actual_post_sec'])
      pre = '' if not p.first_frame_ns else round((p.press.mono_ns - p.first_frame_ns) / 1e9, 3)
      post = '' if not p.last_frame_ns else round((p.last_frame_ns - p.press.mono_ns) / 1e9, 3)
      w.writerow([str(p.final_path.relative_to(self.owner.base_dir)), p.press.side, p.press.label,
                  p.press.mono_ns, p.frame_count, pre, post])

  def _write(self, p: _PendingCase, truncated: bool):
    try:
      p.final_path.parent.mkdir(parents=True, exist_ok=True)
      p.meta['performance_profile']['training_fast_hz'] = self.owner.sample_hz
      p.meta['performance_profile']['diagnostic_context_hz'] = self.owner.context_hz
      with gzip.open(p.part_path, 'wt', encoding='utf-8', compresslevel=1) as fp:
        fp.write(json.dumps(p.meta, separators=(',', ':')) + '\n')
        for frame in p.frames:
          fp.write(json.dumps(frame, separators=(',', ':'), allow_nan=False) + '\n')
        fp.write(json.dumps(p.finish_record(truncated), separators=(',', ':')) + '\n')
      os.replace(p.part_path, p.final_path)
      self._manifest(p)
      self.owner.saved_cases += 1
      self.owner.last_saved_file = str(p.final_path)
      print(f'[G80 ML] saved {p.final_path} ({p.frame_count} frames, bg writer)', flush=True)
    except Exception as e:
      self.owner.last_error = 'writer: ' + repr(e)
      try:
        if p.part_path.exists():
          p.part_path.unlink()
      except Exception:
        pass

  def run(self):
    while True:
      item = self.q.get()
      try:
        if item is None:
          return
        p, truncated = item
        self._write(p, truncated)
      finally:
        self.q.task_done()

  def close(self):
    try:
      self.q.put(None, timeout=0.5)
    except Exception:
      return
    self.join(timeout=3.0)


class MLCaseCollector:
  """Low-overhead RAM history + key-triggered labelled case recorder for V50."""
  def __init__(self, start_keyboard: bool = True):
    self.base_dir = Path(os.getenv('G80_ML_CASE_DIR', '/data/radar/ml_cases_v50'))
    self.disable_marker = Path(os.getenv('G80_ML_DISABLE_MARKER', '/data/radar/DISABLE_G80_ML_CASES'))
    self.pre_s = _env_number('G80_ML_PRE_SEC', 5.0, 1.0, 15.0)
    self.post_s = _env_number('G80_ML_POST_SEC', 3.0, 1.0, 15.0)
    self.sample_hz = _env_number('G80_ML_SAMPLE_HZ', 10.0, 2.0, 10.0)
    self.context_hz = _env_number('G80_ML_CONTEXT_HZ', 2.0, 0.5, 4.0)
    self.idle_hz = _env_number('G80_ML_IDLE_HZ', 2.0, 0.5, 5.0)
    self.min_interval_ns = int(1e9 / self.sample_hz)
    self.context_interval_ns = int(1e9 / self.context_hz)
    self.idle_interval_ns = int(1e9 / self.idle_hz)
    self.ring = deque()  # (mono_ns, compact plain-Python frame); RAM only
    self.last_sample_ns = 0
    self.last_context_ns = 0
    self.session_id = uuid.uuid4().hex
    self._ego_extra = {k: None for k in EGO_EXTRA_FIELDS}
    self._ego_present = {k: False for k in EGO_EXTRA_FIELDS}
    self._ego_recv_ns = 0
    self._ego_log_ns = 0
    self._ego_valid = False
    self._ego_updates = 0
    self.key_q: queue.SimpleQueue = queue.SimpleQueue()
    self.stop_evt = threading.Event()
    self.pending: list[_PendingCase] = []
    self.counter = 0
    self.saved_cases = 0
    self.last_saved_file = ''
    self.last_error = ''
    self._enabled_cache = True
    self._enabled_check_ns = 0
    self._thread = None
    if start_keyboard:
      self._thread = _InputThread(self.key_q, DEFAULT_KEYMAP, self.stop_evt)
      self._thread.start()
    self._writer = _CaseWriter(self)
    self._writer.start()

  def update_ego(self, car_state, recv_ns: int, log_ns: int = 0, valid: bool = True):
    try:
      values = {}; present = {}
      for key in EGO_EXTRA_FIELDS:
        try:
          value = car_state[key] if isinstance(car_state, dict) else getattr(car_state, key)
          present[key] = value is not None
          if key in EGO_BOOL_FIELDS:
            values[key] = bool(value) if value is not None else None
          else:
            values[key] = _finite_or_none(value)
        except Exception:
          present[key] = False
          values[key] = None
      self._ego_extra = values
      self._ego_present = present
      self._ego_recv_ns = int(recv_ns)
      self._ego_log_ns = int(log_ns)
      self._ego_valid = bool(valid)
      self._ego_updates += 1
    except Exception as e:
      self.last_error = 'update_ego: ' + repr(e)

  def _add_ego_extension(self, frame: dict, now_ns: int):
    age_ms = (now_ns-self._ego_recv_ns)/1e6 if self._ego_recv_ns else None
    fresh = bool(self._ego_valid and age_ms is not None and 0 <= age_ms <= 500)
    ego = dict(frame.get('ego_state') or {})
    ego.update(self._ego_extra)
    frame['ego_state'] = ego
    frame['ego_state_extension_meta'] = {
      'source': 'carState', 'hook_received': bool(self._ego_updates),
      'recv_mono_ns': self._ego_recv_ns or None, 'message_log_mono_ns': self._ego_log_ns or None,
      'message_valid': self._ego_valid, 'age_ms': age_ms, 'fresh': fresh,
      'schema_field_present': dict(self._ego_present),
      'sensor_support_verified': False,
      'note': 'Stored values may be stale or schema defaults. Use freshness and verify vehicle support; absent fields are null.'}

  def enabled(self, now_ns: int | None = None) -> bool:
    now_ns = int(now_ns or time.monotonic_ns())
    if now_ns - self._enabled_check_ns >= 1_000_000_000 or self._enabled_check_ns == 0:
      self._enabled_cache = not self.disable_marker.exists()
      self._enabled_check_ns = now_ns
    return self._enabled_cache

  def inject_label(self, side: str, label: str, press_ns: int | None = None):
    side = side.upper(); label = label.upper()
    code = next((k for k, v in DEFAULT_KEYMAP.items() if v == (side, label)), 0)
    if not code:
      raise ValueError('Unknown side/label')
    self.key_q.put(KeyPress(int(press_ns or time.monotonic_ns()), code, side, label, 'INJECT'))

  def _trim_ring(self, now_ns: int):
    keep_ns = int((self.pre_s + 1.0) * 1e9)
    cutoff = int(now_ns) - keep_ns
    while self.ring and self.ring[0][0] < cutoff:
      self.ring.popleft()

  def _paths(self, press: KeyPress) -> tuple[Path, Path]:
    day = datetime.now().astimezone().strftime('%Y%m%d')
    out_dir = self.base_dir / day
    wall = datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%f')[:-3]
    self.counter += 1
    stem = f'{wall}_{press.side}_{press.label}_{self.counter:04d}'
    final = out_dir / f'{stem}.jsonl.gz'
    part = out_dir / f'{stem}.jsonl.gz.part'
    return part, final

  def _start_case(self, press: KeyPress, build_meta: dict):
    if not self.enabled(press.mono_ns):
      return
    if len(self.pending) >= 8:
      self.last_error = 'Too many pending cases (limit 8)'
      return
    start_ns = press.mono_ns - int(self.pre_s * 1e9)
    end_ns = press.mono_ns + int(self.post_s * 1e9)
    part, final = self._paths(press)
    p = _PendingCase(press, start_ns, end_ns, part, final, self.pre_s, self.post_s, build_meta, self.session_id)
    for mono_ns, frame in self.ring:
      p.add(mono_ns, frame)
    self.pending.append(p)
    print(f'[G80 ML] {press.side} {press.label} -> RAM capture {self.pre_s:.1f}s before / {self.post_s:.1f}s after', flush=True)

  def _finish_due(self, now_ns: int):
    keep = []
    for p in self.pending:
      if now_ns < p.end_ns:
        keep.append(p)
        continue
      if not self._writer.submit(p, False):
        self.last_error = 'writer queue full; finished case dropped'
    self.pending = keep

  def _drain_keys(self, build_meta: dict):
    while True:
      try:
        press = self.key_q.get_nowait()
      except queue.Empty:
        break
      try:
        self._start_case(press, build_meta)
      except Exception as e:
        self.last_error = repr(e)

  def _idle_state(self, core: dict) -> bool:
    ego = core.get('ego_state', {}) or {}
    try:
      v = abs(float(ego.get('vEgo', 0.0) or 0.0))
    except Exception:
      v = 0.0
    return (v < 0.5 and not bool(ego.get('leftBlinker')) and not bool(ego.get('rightBlinker'))
            and len(core.get('sensor_fused_objects', []) or []) == 0 and not self.pending)

  def update(self, core: dict, now_ns: int, raw_objects=None, filtered_objects=None):
    """Call once per V50 publish loop. No filesystem/gzip work is done here."""
    try:
      now_ns = int(now_ns)
      self._drain_keys(core.get('runtime_versions', {}))

      if not self.enabled(now_ns) and not self.pending:
        if self.ring:
          self.ring.clear()
        return

      interval_ns = self.idle_interval_ns if self._idle_state(core) else self.min_interval_ns
      if self.last_sample_ns and now_ns - self.last_sample_ns < interval_ns * 0.85:
        self._finish_due(now_ns)
        return
      self.last_sample_ns = now_ns

      frame = _fast_frame(core, now_ns)
      frame['session_id'] = self.session_id
      self._add_ego_extension(frame, now_ns)

      # Expensive duplicate/raw/full-rule snapshots are only added at 2 Hz.
      if self.last_context_ns == 0 or now_ns - self.last_context_ns >= self.context_interval_ns * 0.85:
        frame['context_detail'] = _context_detail(core, raw_objects, filtered_objects)
        self.last_context_ns = now_ns

      # V50: no json.dumps() in the radar publish loop. Serialize only in the background writer.
      self.ring.append((now_ns, frame))
      self._trim_ring(now_ns)

      for p in self.pending:
        p.add(now_ns, frame)
      self._finish_due(now_ns)
    except Exception as e:
      self.last_error = repr(e)

  def close(self):
    self.stop_evt.set()
    if self._thread is not None:
      self._thread.join(timeout=0.6)
      for fd in list(self._thread.fds):
        self._thread._drop(fd)
    # Preserve already-labelled data on a normal shutdown, marking incomplete post windows.
    for p in self.pending:
      self._writer.submit(p, True)
    self.pending.clear()
    self._writer.close()

  def status(self) -> dict:
    return {
      'schema': SCHEMA, 'collector_version': COLLECTOR_VERSION, 'enabled': self.enabled(),
      'session_id': self.session_id, 'ego_extension_hook_received': bool(self._ego_updates),
      'ego_extension_fields_present': dict(self._ego_present),
      'keyboard_running': bool(self._thread and self._thread.is_alive()),
      'input_devices': len(self._thread.fds) if self._thread else 0,
      'pre_sec': self.pre_s, 'post_sec': self.post_s,
      'sample_hz': self.sample_hz, 'context_hz': self.context_hz, 'idle_hz': self.idle_hz,
      'ram_frames': len(self.ring), 'pending_cases': len(self.pending),
      'writer_queue': self._writer.q.qsize(),
      'saved_cases': self.saved_cases, 'last_saved_file': self.last_saved_file,
      'output_dir': str(self.base_dir), 'last_error': self.last_error,
      'disk_policy': 'labelled ML cases only; background writer',
      'serialization_policy': 'RAM dict only in radar loop; JSON+gzip only in background writer',
    }


def _key_test():
  print('G80 V50 ML keypad test. Press F13..F18; Ctrl-C to stop.')
  print('F13 LEFT SAFE | F14 LEFT CHECK | F15 LEFT DANGER | F16 RIGHT SAFE | F17 RIGHT CHECK | F18 RIGHT DANGER')
  q = queue.SimpleQueue(); stop = threading.Event(); t = _InputThread(q, DEFAULT_KEYMAP, stop); t.start()
  try:
    while True:
      try:
        p = q.get(timeout=1.0)
        print(f'{p.side:5s} {p.label:6s} code={p.code} device={p.device}', flush=True)
      except queue.Empty:
        pass
  except KeyboardInterrupt:
    stop.set()


if __name__ == '__main__':
  import sys
  if '--key-test' in sys.argv:
    _key_test()
  else:
    print('This module is integrated into V50 live_service. Use --key-test to test the two keypads.')

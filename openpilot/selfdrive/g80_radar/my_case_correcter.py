#!/usr/bin/env python3
from __future__ import annotations

"""G80 V52R2 event-triggered ML case collector.

Design goals
------------
* NO continuous training-data writes.
* Keep only a short history in RAM.
* Fast 10 Hz training snapshot + 2 Hz diagnostic context.
* Disk/gzip work is deferred to a background writer after the post window.
* A keypad press creates one labelled case containing PRE seconds before the
  press and POST seconds after the press.
* Optional AUTO-LC refined capture: when FG15 confirms a lane-change COMMIT,
  collect a separate weak-label case automatically. One physical lane-change is
  latched to one case only; re-arming requires blinker OFF + FG15 intent idle.
* AUTO_SAFE_CANDIDATE requires ACTIVE-phase evidence and no hard hazard. Partial
  or hazardous executions are AUTO_REVIEW, never silently promoted to SAFE.
* Background JSON/gzip writes are chunk-yielded to reduce GIL/CPU bursts.
* Runs inside V52R2 g80radard/live_service so training data is exactly the V52R2
  state that a later inference module can consume.
* Collector failures must never stop V52R2 radar monitoring.

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

SCHEMA = "g80_v52r1_ml_case_v1"
COLLECTOR_VERSION = 12

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

AUTO_LC_SAFE_LABEL = 'AUTO_SAFE_CANDIDATE'
AUTO_LC_REVIEW_LABEL = 'AUTO_REVIEW'


# Native Linux struct input_event on comma four/aarch64:
#   struct timeval { long sec; long usec; }; u16 type; u16 code; s32 value
_INPUT_EVENT = struct.Struct("@llHHi")


def _env_bool(name: str, default: bool = True) -> bool:
  raw = os.getenv(name)
  if raw is None:
    return bool(default)
  return str(raw).strip().lower() not in ('0', 'false', 'no', 'off', 'disable', 'disabled')


def _env_number(name, default, low, high):
  try:
    value = float(os.getenv(name, str(default)))
    return max(low, min(high, value)) if math.isfinite(value) else default
  except (ValueError, TypeError):
    return default


def _env_int(name, default, low, high):
  try:
    value = int(float(os.getenv(name, str(default))))
    return max(int(low), min(int(high), value))
  except (ValueError, TypeError):
    return int(default)


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
  """Keep V51 identity, geometry, motion and provenance fields for ML/replay."""
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


def _compact_object_fast(o: dict) -> dict:
  """10 Hz ML object snapshot: keep scalar KF/IMM state, omit bulky trajectories.

  Full trajectories remain in the 2 Hz context snapshot, so no information is
  lost from labelled cases while the realtime-ish radar loop avoids repeatedly
  deep-copying per-object prediction arrays.
  """
  out = _compact_object(o)
  out.pop('kalman_trajectory', None)
  # These lists can be large in dense traffic and are provenance/debug rather
  # than current-time model inputs. They are retained in context_detail at 2 Hz.
  out.pop('vehicle_cluster_keys', None)
  out.pop('canonical_domain_history', None)
  return out


def _compact_frame(core: dict, raw_objects, filtered_objects, now_ns: int) -> dict:
  """Snapshot inputs useful to a later PC-trained V51 inference model."""
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
    # V51 raw/validity-gated radar snapshots are supplied directly by live_service,
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
    # Canonical360 is the intended 10 Hz ML input. Keep scalar KF/IMM state at 10 Hz;
    # full trajectories/provenance remain in context_detail at 2 Hz.
    'sensor_fused_objects': [_compact_object_fast(o) for o in (core.get('sensor_fused_objects', []) or [])],
    # V51: StarPilot-derived side-window vision is a 10 Hz feature snapshot,
    # but remains SHADOW evidence and is not allowed to rewrite FG labels here.
    'side_vision': _plain(core.get('side_vision', {})),
    # V52R1: WIDE ROAD front-left/front-right ROI classifier, SHADOW only.
    'front_corner_vision': _plain(core.get('front_corner_vision', {})),
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
               pre_s: float, post_s: float, build_meta: dict, session_id: str,
               label_source: str = 'manual_key', output_root: Path | None = None,
               extra_meta: dict | None = None):
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
      'label_source': str(label_source),
      'training_default_include': bool(label_source == 'manual_key'),
      'key_device': press.device, 'press_mono_ns': int(press.mono_ns),
      'pre_sec': float(pre_s), 'post_sec': float(post_s),
      'window_start_ns': int(start_ns), 'window_end_ns': int(end_ns),
      'created_wall_time': datetime.now().astimezone().isoformat(timespec='milliseconds'),
      'build': _plain(build_meta),
      'note': 'Pre/post frames are RAM buffered. File creation/compression happens on a background writer only after capture.',
    }
    self.output_root = Path(output_root) if output_root is not None else final_path.parent
    if extra_meta:
      self.meta.update(_plain(extra_meta))

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
  """Background-only gzip/file writer so ML labels cannot block V51 rendering/fusion."""
  def __init__(self, owner):
    super().__init__(name='g80-ml-writer', daemon=True)
    self.owner = owner
    self.q: queue.Queue = queue.Queue(maxsize=16)
    self.active = False
    self.last_write_ms = 0.0
    self.max_write_ms = 0.0
    self.write_count = 0
    self.last_started_ns = 0

  def submit(self, p: _PendingCase, truncated: bool = False) -> bool:
    try:
      self.q.put_nowait((p, bool(truncated)))
      try:
        self.owner.writer_queue_peak = max(int(self.owner.writer_queue_peak), int(self.q.qsize()))
      except Exception:
        pass
      return True
    except queue.Full:
      self.owner.last_error = 'ML writer queue full; case not saved'
      return False

  def _manifest(self, p: _PendingCase):
    root = Path(p.output_root)
    path = root / 'manifest.csv'
    new = not path.exists()
    root.mkdir(parents=True, exist_ok=True)
    with path.open('a', newline='', encoding='utf-8') as f:
      w = csv.writer(f)
      if new:
        w.writerow(['file','side','label','press_mono_ns','frames','actual_pre_sec','actual_post_sec'])
      pre = '' if not p.first_frame_ns else round((p.press.mono_ns - p.first_frame_ns) / 1e9, 3)
      post = '' if not p.last_frame_ns else round((p.last_frame_ns - p.press.mono_ns) / 1e9, 3)
      w.writerow([str(p.final_path.relative_to(root)), p.press.side, p.press.label,
                  p.press.mono_ns, p.frame_count, pre, post])

  def _write(self, p: _PendingCase, truncated: bool):
    t0 = time.perf_counter_ns()
    self.active = True
    self.last_started_ns = time.monotonic_ns()
    try:
      p.final_path.parent.mkdir(parents=True, exist_ok=True)
      p.meta['performance_profile']['training_fast_hz'] = self.owner.sample_hz
      _ints = list(self.owner.capture_intervals_ns)
      _avg = (sum(_ints) / len(_ints)) if _ints else 0
      p.meta['performance_profile']['actual_capture_hz'] = round(1e9 / _avg, 3) if _avg > 0 else None
      p.meta['performance_profile']['max_frame_gap_ms'] = round(max(_ints) / 1e6, 3) if _ints else None
      p.meta['performance_profile']['capture_missed_slots'] = int(self.owner.capture_missed_slots)
      p.meta['performance_profile']['capture_policy'] = 'real V51 snapshots only; no interpolation/duplicate frames'
      p.meta['performance_profile']['diagnostic_context_hz'] = self.owner.context_hz
      p.meta['performance_profile']['writer_yield_every_frames'] = self.owner.writer_yield_every_frames
      p.meta['performance_profile']['writer_yield_ms'] = round(self.owner.writer_yield_s * 1000.0, 3)
      with gzip.open(p.part_path, 'wt', encoding='utf-8', compresslevel=1) as fp:
        fp.write(json.dumps(p.meta, separators=(',', ':')) + '\n')
        for i, frame in enumerate(p.frames, 1):
          fp.write(json.dumps(frame, separators=(',', ':'), allow_nan=False) + '\n')
          # The writer is a separate thread but shares the Python process/GIL.
          # Yield in small chunks so radar/UI scheduling wins over file throughput.
          if self.owner.writer_yield_every_frames > 0 and i % self.owner.writer_yield_every_frames == 0:
            time.sleep(self.owner.writer_yield_s)
        fp.write(json.dumps(p.finish_record(truncated), separators=(',', ':')) + '\n')
      os.replace(p.part_path, p.final_path)
      self._manifest(p)
      self.owner.saved_cases += 1
      self.owner.last_saved_file = str(p.final_path)
      print(f'[G80 ML] saved {p.final_path} ({p.frame_count} frames, yielding bg writer)', flush=True)
    except Exception as e:
      self.owner.last_error = 'writer: ' + repr(e)
      try:
        if p.part_path.exists():
          p.part_path.unlink()
      except Exception:
        pass
    finally:
      dt_ms = (time.perf_counter_ns() - t0) / 1e6
      self.last_write_ms = dt_ms
      self.max_write_ms = max(self.max_write_ms, dt_ms)
      self.write_count += 1
      self.active = False

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
  """Low-overhead RAM history + key-triggered labelled case recorder for V51."""
  def __init__(self, start_keyboard: bool = True):
    self.base_dir = Path(os.getenv('G80_ML_CASE_DIR', '/data/radar/ml_cases_v51'))
    self.auto_lc_dir = Path(os.getenv('G80_ML_AUTO_LC_DIR', '/data/radar/ml_cases_v51_auto_lanechange'))
    self.disable_marker = Path(os.getenv('G80_ML_DISABLE_MARKER', '/data/radar/DISABLE_G80_ML_CASES'))
    self.auto_lc_disable_marker = Path(os.getenv('G80_ML_AUTO_LC_DISABLE_MARKER', '/data/radar/DISABLE_G80_AUTO_LC'))
    self.auto_lc_config_enabled = _env_bool('G80_ML_AUTO_LC_ENABLE', True)
    self.auto_lc_cooldown_s = _env_number('G80_ML_AUTO_LC_COOLDOWN_SEC', 5.0, 1.0, 30.0)
    # Re-arm only after the previous maneuver is truly over: both blinkers OFF
    # and FG15 intent idle continuously for this interval. This is the primary
    # duplicate-prevention mechanism; cooldown is only a secondary guard.
    self.auto_lc_rearm_off_s = _env_number('G80_ML_AUTO_LC_REARM_OFF_SEC', 0.8, 0.3, 3.0)
    # Automatic lane-change cases need a little more post time than manual labels
    # so FG15 can reach ACTIVE and the blinker can auto-cancel after lane crossing.
    self.auto_lc_post_s = _env_number('G80_ML_AUTO_LC_POST_SEC', 4.5, 3.0, 8.0)
    self.auto_lc_min_progress_s = _env_number('G80_ML_AUTO_LC_MIN_PROGRESS_SEC', 2.4, 1.0, 3.0)
    # Background writer cooperatively yields the GIL/CPU. Defaults add only a few
    # tens of milliseconds to file completion while limiting one long CPU burst.
    self.writer_yield_every_frames = _env_int('G80_ML_WRITER_YIELD_EVERY_FRAMES', 4, 1, 32)
    self.writer_yield_s = _env_number('G80_ML_WRITER_YIELD_MS', 1.0, 0.0, 10.0) / 1000.0
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
    # Measure actual ML snapshot cadence; never synthesize duplicate frames.
    self.capture_intervals_ns = deque(maxlen=128)
    self.capture_active_intervals_ns = deque(maxlen=128)
    self.capture_samples = 0
    # Missed-slot diagnostics apply only while the collector is in ACTIVE 10 Hz mode.
    # V52R1 counted intentional 2 Hz idle gaps as four missed 10 Hz slots, which made
    # check_v52r1.sh look broken while parked/offroad even though AUTO-LC switched to
    # 10 Hz as soon as a blinker/maneuver was present.
    self.capture_missed_slots = 0
    self.capture_max_gap_ns = 0
    self.last_sample_mode = None
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
    self.auto_candidate = None
    self.auto_saved_cases = 0
    self.auto_review_cases = 0
    self.auto_rejected_cases = 0
    self.auto_last_trigger_ns = 0
    self.auto_last_completed_ns = 0
    self.auto_last_result = ''
    self.auto_armed = True
    self.auto_latched_side = None
    self.auto_rearm_since_ns = 0
    self.auto_rearm_count = 0
    self.auto_capture_last_ms = 0.0
    self.auto_capture_max_ms = 0.0
    self.auto_capture_ema_ms = 0.0
    self.writer_queue_peak = 0
    self._auto_enabled_cache = bool(self.auto_lc_config_enabled)
    self._auto_enabled_check_ns = 0
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
    p = _PendingCase(press, start_ns, end_ns, part, final, self.pre_s, self.post_s, build_meta, self.session_id,
                     label_source='manual_key', output_root=self.base_dir)
    for mono_ns, frame in self.ring:
      p.add(mono_ns, frame)
    self.pending.append(p)
    print(f'[G80 ML] {press.side} {press.label} -> RAM capture {self.pre_s:.1f}s before / {self.post_s:.1f}s after', flush=True)

  def auto_lc_enabled(self, now_ns: int | None = None) -> bool:
    now_ns = int(now_ns or time.monotonic_ns())
    if now_ns - self._auto_enabled_check_ns >= 1_000_000_000 or self._auto_enabled_check_ns == 0:
      self._auto_enabled_cache = bool(self.auto_lc_config_enabled and not self.auto_lc_disable_marker.exists())
      self._auto_enabled_check_ns = now_ns
    return self._auto_enabled_cache

  def _auto_paths(self, side: str) -> tuple[Path, Path]:
    day = datetime.now().astimezone().strftime('%Y%m%d')
    out_dir = self.auto_lc_dir / day
    wall = datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%f')[:-3]
    self.counter += 1
    stem = f'{wall}_AUTO_LC_{side.upper()}_{self.counter:04d}'
    final = out_dir / f'{stem}.jsonl.gz'
    part = out_dir / f'{stem}.jsonl.gz.part'
    return part, final

  @staticmethod
  def _matching_blinker(ego: dict, side: str) -> bool:
    if side == 'LEFT':
      return bool(ego.get('leftBlinker')) and not bool(ego.get('rightBlinker'))
    return bool(ego.get('rightBlinker')) and not bool(ego.get('leftBlinker'))

  def _update_auto_rearm(self, ego: dict, intent: dict, now_ns: int):
    """One physical lane-change may create only one auto case.

    After a trigger, do not arm again merely because the fixed capture window
    ended.  Require BOTH blinkers off and FG15 driver intent idle continuously.
    This prevents a long ACTIVE phase from re-triggering the same maneuver.
    """
    if self.auto_armed or self.auto_candidate is not None:
      return
    both_off = not bool(ego.get('leftBlinker')) and not bool(ego.get('rightBlinker'))
    intent_idle = not bool(intent.get('active')) and not bool(intent.get('committed'))
    if both_off and intent_idle:
      if not self.auto_rearm_since_ns:
        self.auto_rearm_since_ns = int(now_ns)
      elif now_ns - self.auto_rearm_since_ns >= int(self.auto_lc_rearm_off_s * 1e9):
        self.auto_armed = True
        self.auto_latched_side = None
        self.auto_rearm_since_ns = 0
        self.auto_rearm_count += 1
        self.auto_last_result = 'AUTO-LC re-armed after blinker-off + intent-idle'
    else:
      self.auto_rearm_since_ns = 0

  def _discard_auto_candidate(self, reason: str):
    if self.auto_candidate is not None:
      side = self.auto_candidate.get('side', '?')
      self.auto_rejected_cases += 1
      self.auto_last_result = f'{side} discarded: {reason}'
      self.auto_candidate = None
      self.auto_last_completed_ns = time.monotonic_ns()
      print(f'[G80 ML AUTO] {side} candidate discarded ({reason})', flush=True)

  def _start_auto_lane_change(self, core: dict, now_ns: int, side: str, intent: dict):
    side = str(side).upper()
    if side not in ('LEFT', 'RIGHT') or self.auto_candidate is not None or not self.auto_armed:
      return
    if self.auto_last_trigger_ns and now_ns - self.auto_last_trigger_ns < int(self.auto_lc_cooldown_s * 1e9):
      return
    press = KeyPress(int(now_ns), 0, side, AUTO_LC_SAFE_LABEL, 'AUTO_LANE_CHANGE_FG15_COMMIT')
    start_ns = int(now_ns) - int(self.pre_s * 1e9)
    end_ns = int(now_ns) + int(self.auto_lc_post_s * 1e9)
    part, final = self._auto_paths(side)
    trigger_phase = str(intent.get('phase') or '')
    extra = {
      'weak_label': True,
      'training_default_include': False,
      'auto_lane_change': {
        'version': 2,
        'trigger': 'FG15 driver_intent committed=True in LANE_CHANGE context',
        'interpretation': 'executed-lane-change weak-label candidate, NOT proof of objective safety',
        'review_required_before_promoting_to_manual_SAFE': True,
        'one_case_per_maneuver_latch': True,
        'rearm_policy': f'both blinkers OFF + FG15 intent idle for {self.auto_lc_rearm_off_s:.1f}s',
        'safe_candidate_requires': 'FG15 ACTIVE phase observed + no DANGER/BSD/hard-override/emergency-decel',
        'partial_execution_policy': 'REBASING without ACTIVE => AUTO_REVIEW',
        'trigger_phase': trigger_phase,
        'trigger_label': intent.get('label'),
        'trigger_state': intent.get('state'),
        'trigger_commit_age_s': intent.get('commit_age_s'),
        'lane_commit_ready': intent.get('lane_commit_ready'),
        'lane_commit_status': intent.get('lane_commit_status'),
      }
    }
    p = _PendingCase(press, start_ns, end_ns, part, final, self.pre_s, self.auto_lc_post_s,
                     core.get('runtime_versions', {}), self.session_id,
                     label_source='auto_lane_change_weak', output_root=self.auto_lc_dir, extra_meta=extra)
    for mono_ns, frame in self.ring:
      p.add(mono_ns, frame)
    self.auto_candidate = {
      'case': p, 'side': side, 'start_ns': int(now_ns), 'max_commit_age_s': 0.0,
      'saw_commit_hold': trigger_phase == 'COMMIT_HOLD',
      'saw_rebasing': trigger_phase == 'REBASING',
      'saw_active': trigger_phase == 'ACTIVE',
      'saw_blinker_off': False, 'saw_turn_context': False,
      'saw_danger': False, 'saw_bsd_block': False, 'saw_hard_override': False,
      'saw_emergency_decel': False, 'min_a_ego': 99.0, 'max_abs_steer_deg': 0.0,
      'saw_side_vision_active': False, 'max_side_vision_score': 0.0,
      'saw_front_corner_vision_active': False, 'max_front_corner_vision_score': 0.0,
      'samples': 0,
    }
    # Disarm immediately. Re-arming happens only after this whole maneuver has
    # ended, not when the fixed +4.5 s capture window happens to end.
    self.auto_armed = False
    self.auto_latched_side = side
    self.auto_rearm_since_ns = 0
    self.auto_last_trigger_ns = int(now_ns)
    self.auto_last_result = f'{side} auto candidate started'
    print(f'[G80 ML AUTO] {side} lane-change COMMIT -> candidate capture', flush=True)

  def _observe_auto_lane_change(self, core: dict, now_ns: int, frame: dict):
    fg = core.get('future_gap', {}) or {}
    intent = fg.get('driver_intent', {}) or {}
    ego = core.get('ego_state', {}) or {}

    if not self.auto_lc_enabled(now_ns):
      # If the user disables AUTO-LC while a capture is live, do not leave a
      # zombie candidate that can retain RAM forever. Do not write a partial weak label.
      if self.auto_candidate is not None:
        self._discard_auto_candidate('auto_disabled_mid_capture')
      return

    if self.auto_candidate is None:
      self._update_auto_rearm(ego, intent, now_ns)
      if not self.auto_armed:
        return
      side_l = str(intent.get('side') or '').lower()
      side = side_l.upper()
      matching = self._matching_blinker(ego, side) if side in ('LEFT', 'RIGHT') else False
      phase = str(intent.get('phase') or '')
      if (bool(intent.get('active')) and bool(intent.get('committed')) and
          str(intent.get('maneuver_context') or '') == 'LANE_CHANGE' and
          side in ('LEFT', 'RIGHT') and matching and
          phase in ('COMMIT_HOLD', 'REBASING', 'ACTIVE')):
        self._start_auto_lane_change(core, now_ns, side, intent)
      return

    c = self.auto_candidate
    p: _PendingCase = c['case']
    side = c['side']
    c['samples'] += 1
    p.add(now_ns, frame)

    phase = str(intent.get('phase') or '')
    if phase == 'COMMIT_HOLD': c['saw_commit_hold'] = True
    if phase == 'REBASING': c['saw_rebasing'] = True
    if phase == 'ACTIVE': c['saw_active'] = True
    if str(intent.get('maneuver_context') or '') == 'TURN': c['saw_turn_context'] = True
    if not self._matching_blinker(ego, side): c['saw_blinker_off'] = True

    try:
      c['max_commit_age_s'] = max(float(c['max_commit_age_s']), float(intent.get('commit_age_s') or 0.0))
    except Exception:
      pass
    try:
      c['max_abs_steer_deg'] = max(float(c['max_abs_steer_deg']), abs(float(ego.get('steeringAngleDeg') or 0.0)))
    except Exception:
      pass
    try:
      a = float(ego.get('aEgo') or 0.0)
      c['min_a_ego'] = min(float(c['min_a_ego']), a)
      if a <= -3.5:
        c['saw_emergency_decel'] = True
    except Exception:
      pass

    side_key = side.lower()
    side_state = fg.get(side_key, {}) or {}
    dec = side_state.get('decision', {}) or {}
    if str(dec.get('state') or '') == 'BLOCKED_SHADOW' or str(dec.get('label') or '').upper().startswith('DANGER'):
      c['saw_danger'] = True
    bsd = fg.get('bsd', {}) or {}
    blocked = bsd.get('blocked', {}) or {}
    if bool(blocked.get(side_key)):
      c['saw_bsd_block'] = True
    hard = intent.get('hard_override') or {}
    if isinstance(hard, dict) and bool(hard.get('confirmed')):
      c['saw_hard_override'] = True

    # V51 camera evidence is deliberately observational.  Log whether the
    # StarPilot-derived side vision saw a car during the executed maneuver, but
    # do not let an unvalidated camera model promote/demote AUTO_SAFE labels.
    sv = core.get('side_vision', {}) or {}
    sv_side = sv.get(side_key, {}) or {}
    try:
      if bool(sv.get('usable')) and bool(sv_side.get('effective_active', sv_side.get('active'))):
        c['saw_side_vision_active'] = True
      score = float(sv_side.get('score') or sv_side.get('raw_confidence') or 0.0)
      c['max_side_vision_score'] = max(float(c['max_side_vision_score']), score)
    except Exception:
      pass

    # V52R1 front-corner WIDE ROAD vision is also observational only.
    # Match lane-change side to FL/FR and record correlation; never use it to
    # promote/demote AUTO_SAFE labels until a dedicated front-corner model is validated.
    fcv = core.get('front_corner_vision', {}) or {}
    fcv_key = 'fl' if side_key == 'left' else 'fr'
    fcv_side = fcv.get(fcv_key, {}) or {}
    try:
      if bool(fcv.get('usable')) and bool(fcv_side.get('effective_active', fcv_side.get('active'))):
        c['saw_front_corner_vision_active'] = True
      score = float(fcv_side.get('score') or fcv_side.get('raw_confidence') or 0.0)
      c['max_front_corner_vision_score'] = max(float(c['max_front_corner_vision_score']), score)
    except Exception:
      pass

    if now_ns < p.end_ns:
      return

    progressed = bool(c['saw_rebasing'] or c['saw_active'])
    executed_confident = bool(c['saw_active'])
    hazard = bool(c['saw_danger'] or c['saw_bsd_block'] or c['saw_hard_override'] or c['saw_emergency_decel'])
    reject = bool(c['saw_turn_context'] or not progressed)

    auto_meta = p.meta.setdefault('auto_lane_change', {})
    if executed_confident and c['saw_blinker_off']:
      completion_confidence = 'HIGH'
    elif executed_confident:
      completion_confidence = 'MEDIUM'
    elif progressed:
      completion_confidence = 'PARTIAL'
    else:
      completion_confidence = 'LOW'
    auto_meta.update({
      'saw_commit_hold': bool(c['saw_commit_hold']),
      'saw_rebasing': bool(c['saw_rebasing']),
      'saw_active': bool(c['saw_active']),
      'saw_blinker_off': bool(c['saw_blinker_off']),
      'saw_turn_context': bool(c['saw_turn_context']),
      'saw_danger': bool(c['saw_danger']),
      'saw_bsd_block': bool(c['saw_bsd_block']),
      'saw_hard_override': bool(c['saw_hard_override']),
      'saw_emergency_decel': bool(c['saw_emergency_decel']),
      'saw_side_vision_active': bool(c['saw_side_vision_active']),
      'max_side_vision_score': round(float(c['max_side_vision_score']), 4),
      'side_vision_policy': 'logged for correlation only; excluded from AUTO_SAFE promotion in V51',
      'saw_front_corner_vision_active': bool(c['saw_front_corner_vision_active']),
      'max_front_corner_vision_score': round(float(c['max_front_corner_vision_score']), 4),
      'front_corner_vision_policy': 'WIDE ROAD FL/FR SHADOW correlation only; side V-ASM reused temporarily; excluded from AUTO_SAFE promotion in V52R1',
      'min_a_ego_mps2': None if c['min_a_ego'] > 90 else round(float(c['min_a_ego']), 3),
      'max_abs_steering_deg': round(float(c['max_abs_steer_deg']), 2),
      'max_commit_age_s': round(float(c['max_commit_age_s']), 3),
      'progressed': progressed,
      'executed_confident': executed_confident,
      'hazard_seen': hazard,
      'completion_confidence': completion_confidence,
    })

    if reject:
      self.auto_rejected_cases += 1
      self.auto_last_result = f'{side} rejected: turn_or_no_lane_change_progress'
      print(f'[G80 ML AUTO] {side} candidate discarded (turn/no lane-change progress)', flush=True)
    else:
      # A physical lane change is useful positive evidence, but it is still not
      # objective ground truth. Only a full ACTIVE progression with no observed
      # hard hazard becomes AUTO_SAFE_CANDIDATE. Partial/hazard cases go REVIEW.
      if hazard:
        p.press.label = AUTO_LC_REVIEW_LABEL
        p.meta['label'] = AUTO_LC_REVIEW_LABEL
        p.meta['training_default_include'] = False
        p.meta['auto_lane_change']['promotion_reason'] = 'hazard_or_emergency_evidence_seen'
        self.auto_review_cases += 1
      elif not executed_confident:
        p.press.label = AUTO_LC_REVIEW_LABEL
        p.meta['label'] = AUTO_LC_REVIEW_LABEL
        p.meta['training_default_include'] = False
        p.meta['auto_lane_change']['promotion_reason'] = 'partial_progress_without_FG15_ACTIVE'
        self.auto_review_cases += 1
      else:
        p.press.label = AUTO_LC_SAFE_LABEL
        p.meta['label'] = AUTO_LC_SAFE_LABEL
        p.meta['training_default_include'] = False
        p.meta['auto_lane_change']['promotion_reason'] = 'FG15_ACTIVE_execution_without_observed_hard_hazard'
        self.auto_saved_cases += 1
      if not self._writer.submit(p, False):
        self.last_error = 'writer queue full; auto lane-change case dropped'
      self.auto_last_result = f'{side} saved as {p.press.label}; waiting for maneuver release before re-arm'
      print(f'[G80 ML AUTO] {side} -> {p.press.label}', flush=True)

    self.auto_candidate = None
    self.auto_last_completed_ns = int(now_ns)
    # Deliberately remain disarmed. _update_auto_rearm() will arm only after the
    # blinker is OFF and FG15 intent is idle for the configured release interval.

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
            and len(core.get('sensor_fused_objects', []) or []) == 0 and not self.pending
            and self.auto_candidate is None)

  def update(self, core: dict, now_ns: int, raw_objects=None, filtered_objects=None):
    """Call once per V51 publish loop. No filesystem/gzip work is done here."""
    try:
      now_ns = int(now_ns)
      self._drain_keys(core.get('runtime_versions', {}))

      collector_enabled = self.enabled(now_ns)
      if not collector_enabled and self.auto_candidate is not None:
        self._discard_auto_candidate('collector_disabled_mid_capture')
      if not collector_enabled and not self.pending:
        if self.ring:
          self.ring.clear()
        return

      idle_mode = self._idle_state(core)
      interval_ns = self.idle_interval_ns if idle_mode else self.min_interval_ns
      sample_mode = 'IDLE_2HZ' if idle_mode else 'ACTIVE_10HZ'
      if self.last_sample_ns and now_ns - self.last_sample_ns < interval_ns * 0.85:
        self._finish_due(now_ns)
        return
      # Measure the real cadence delivered by live_service instead of
      # interpolating/duplicating stale frames. Intentional IDLE_2HZ gaps are not
      # counted as missing 10 Hz training snapshots.
      if self.last_sample_ns:
        gap_ns = max(0, now_ns - self.last_sample_ns)
        self.capture_intervals_ns.append(gap_ns)
        self.capture_max_gap_ns = max(self.capture_max_gap_ns, gap_ns)
        if sample_mode == 'ACTIVE_10HZ' and self.last_sample_mode == 'ACTIVE_10HZ':
          self.capture_active_intervals_ns.append(gap_ns)
          nominal_slots = max(1, int(round(gap_ns / max(self.min_interval_ns, 1))))
          if nominal_slots > 1:
            self.capture_missed_slots += nominal_slots - 1
      self.last_sample_ns = now_ns
      self.last_sample_mode = sample_mode
      self.capture_samples += 1

      frame = _fast_frame(core, now_ns)
      frame['session_id'] = self.session_id
      self._add_ego_extension(frame, now_ns)

      # Expensive duplicate/raw/full-rule snapshots are only added at 2 Hz.
      if self.last_context_ns == 0 or now_ns - self.last_context_ns >= self.context_interval_ns * 0.85:
        frame['context_detail'] = _context_detail(core, raw_objects, filtered_objects)
        self.last_context_ns = now_ns

      # V51: no json.dumps() in the radar publish loop. Serialize only in the background writer.
      self.ring.append((now_ns, frame))
      self._trim_ring(now_ns)

      # AUTO-LC detection is RAM/state-machine work only. Keep its own timing so
      # any regression is visible without conflating it with background file I/O.
      _auto_t0 = time.perf_counter_ns()
      self._observe_auto_lane_change(core, now_ns, frame)
      _auto_ms = (time.perf_counter_ns() - _auto_t0) / 1e6
      self.auto_capture_last_ms = _auto_ms
      self.auto_capture_max_ms = max(self.auto_capture_max_ms, _auto_ms)
      self.auto_capture_ema_ms = _auto_ms if self.auto_capture_ema_ms <= 0 else (0.95 * self.auto_capture_ema_ms + 0.05 * _auto_ms)

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
    # Do not promote an unfinished automatic lane change on shutdown.
    if self.auto_candidate is not None:
      self.auto_rejected_cases += 1
      self.auto_last_result = 'unfinished auto lane-change discarded on shutdown'
      self.auto_candidate = None
      self.auto_armed = False
    self._writer.close()

  def status(self) -> dict:
    intervals = list(self.capture_intervals_ns)
    if intervals:
      avg_gap_ns = sum(intervals) / len(intervals)
      actual_capture_hz = (1e9 / avg_gap_ns) if avg_gap_ns > 0 else None
      max_frame_gap_ms = max(intervals) / 1e6
    else:
      actual_capture_hz = None
      max_frame_gap_ms = None
    active_intervals = list(self.capture_active_intervals_ns)
    if active_intervals:
      active_avg_gap_ns = sum(active_intervals) / len(active_intervals)
      actual_active_hz = (1e9 / active_avg_gap_ns) if active_avg_gap_ns > 0 else None
    else:
      actual_active_hz = None
    return {
      'schema': SCHEMA, 'collector_version': COLLECTOR_VERSION, 'enabled': self.enabled(),
      'session_id': self.session_id, 'ego_extension_hook_received': bool(self._ego_updates),
      'ego_extension_fields_present': dict(self._ego_present),
      'keyboard_running': bool(self._thread and self._thread.is_alive()),
      'input_devices': len(self._thread.fds) if self._thread else 0,
      'pre_sec': self.pre_s, 'post_sec': self.post_s,
      'sample_hz': self.sample_hz, 'context_hz': self.context_hz, 'idle_hz': self.idle_hz,
      'capture_mode': self.last_sample_mode or 'WAIT',
      'actual_capture_hz': None if actual_capture_hz is None else round(actual_capture_hz, 3),
      'actual_active_hz': None if actual_active_hz is None else round(actual_active_hz, 3),
      'max_frame_gap_ms': None if max_frame_gap_ms is None else round(max_frame_gap_ms, 3),
      'capture_samples': int(self.capture_samples),
      'capture_missed_slots': int(self.capture_missed_slots),
      'capture_policy': 'real V52R2 snapshots only; ACTIVE=10Hz, intentional IDLE=2Hz; no interpolation/duplicate frames',
      'side_vision_fast_input': True,
      'front_corner_vision_fast_input': True,
      'ram_frames': len(self.ring), 'pending_cases': len(self.pending),
      'writer_queue': self._writer.q.qsize(),
      'saved_cases': self.saved_cases, 'last_saved_file': self.last_saved_file,
      'output_dir': str(self.base_dir),
      'auto_lane_change': {
        'enabled': self.auto_lc_enabled(),
        'output_dir': str(self.auto_lc_dir),
        'candidate_active': self.auto_candidate is not None,
        'armed': bool(self.auto_armed),
        'latched_side': self.auto_latched_side,
        'rearm_off_sec': self.auto_lc_rearm_off_s,
        'rearm_count': self.auto_rearm_count,
        'post_sec': self.auto_lc_post_s,
        'saved_safe_candidates': self.auto_saved_cases,
        'saved_review_cases': self.auto_review_cases,
        'rejected_cases': self.auto_rejected_cases,
        'last_result': self.auto_last_result,
        'capture_last_ms': round(self.auto_capture_last_ms, 4),
        'capture_ema_ms': round(self.auto_capture_ema_ms, 4),
        'capture_max_ms': round(self.auto_capture_max_ms, 4),
        'label_policy': 'AUTO_SAFE_CANDIDATE requires FG15 ACTIVE + no hard hazard; weak label; training_default_include=false',
      },
      'writer': {
        'active': bool(self._writer.active),
        'queue': self._writer.q.qsize(),
        'queue_peak': int(self.writer_queue_peak),
        'write_count': int(self._writer.write_count),
        'last_write_ms': round(self._writer.last_write_ms, 2),
        'max_write_ms': round(self._writer.max_write_ms, 2),
        'yield_every_frames': int(self.writer_yield_every_frames),
        'yield_ms': round(self.writer_yield_s * 1000.0, 3),
      },
      'last_error': self.last_error,
      'disk_policy': 'manual labels + separate AUTO-LC weak-label cases; cooperative-yield background writer only',
      'serialization_policy': 'RAM dict only in radar loop; JSON+gzip only in yielding background writer',
    }


def _key_test():
  print('G80 V51 ML keypad test. Press F13..F18; Ctrl-C to stop.')
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
    print('This module is integrated into V51 live_service. Use --key-test to test the two keypads.')

#!/usr/bin/env python3
from __future__ import annotations

"""G80 V49 event-triggered ML case collector.

Design goals
------------
* NO continuous training-data writes.
* Keep only a short history in RAM.
* A keypad press creates one labelled case containing PRE seconds before the
  press and POST seconds after the press.
* Runs inside V49 g80radard/live_service so training data is exactly the V49
  state that a later inference module can consume.
* Collector failures must never stop V49 radar monitoring.

Default key map (Linux EV_KEY codes):
  F13 183 = LEFT SAFE
  F14 184 = LEFT CHECK
  F15 185 = LEFT DANGER
  F16 186 = RIGHT SAFE
  F17 187 = RIGHT CHECK
  F18 188 = RIGHT DANGER
"""

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

SCHEMA = "g80_v49_ml_case_v1"
COLLECTOR_VERSION = 2

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
  """Keep V49 identity, geometry, motion and provenance fields for ML/replay."""
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
  """Snapshot inputs useful to a later PC-trained V49 inference model."""
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
    # V49 raw/validity-gated radar snapshots are supplied directly by live_service,
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


class _PendingCase:
  def __init__(self, press: KeyPress, start_ns: int, end_ns: int, part_path: Path, final_path: Path,
               pre_s: float, post_s: float, build_meta: dict):
    self.press = press
    self.start_ns = int(start_ns)
    self.end_ns = int(end_ns)
    self.part_path = part_path
    self.final_path = final_path
    self.fp = gzip.open(part_path, 'wt', encoding='utf-8', compresslevel=3)
    self.frame_count = 0
    self.first_frame_ns = 0
    self.last_frame_ns = 0
    self.last_written_ns = 0
    meta = {
      'type': 'case_meta', 'schema': SCHEMA, 'collector_version': COLLECTOR_VERSION,
      'side': press.side, 'label': press.label, 'key_code': press.code,
      'key_device': press.device, 'press_mono_ns': int(press.mono_ns),
      'pre_sec': float(pre_s), 'post_sec': float(post_s),
      'window_start_ns': int(start_ns), 'window_end_ns': int(end_ns),
      'created_wall_time': datetime.now().astimezone().isoformat(timespec='milliseconds'),
      'build': _plain(build_meta),
      'note': 'Frames before press came from RAM ring buffer; disk file is created only after a label key press.',
    }
    self.fp.write(json.dumps(meta, separators=(',', ':')) + '\n')

  def add(self, mono_ns: int, line: str):
    mono_ns = int(mono_ns)
    if mono_ns < self.start_ns or mono_ns > self.end_ns or mono_ns <= self.last_written_ns:
      return
    self.fp.write(line + '\n')
    self.frame_count += 1
    if not self.first_frame_ns:
      self.first_frame_ns = mono_ns
    self.last_frame_ns = mono_ns
    self.last_written_ns = mono_ns

  def finish(self):
    end = {
      'type': 'case_end', 'frame_count': int(self.frame_count),
      'first_frame_ns': int(self.first_frame_ns), 'last_frame_ns': int(self.last_frame_ns),
      'actual_pre_sec': None if not self.first_frame_ns else round((self.press.mono_ns - self.first_frame_ns) / 1e9, 3),
      'actual_post_sec': None if not self.last_frame_ns else round((self.last_frame_ns - self.press.mono_ns) / 1e9, 3),
    }
    self.fp.write(json.dumps(end, separators=(',', ':')) + '\n')
    self.fp.close()
    os.replace(self.part_path, self.final_path)

  def abort(self):
    try:
      self.fp.close()
    except Exception:
      pass


class MLCaseCollector:
  """RAM-history + key-triggered case recorder for V49."""
  def __init__(self, start_keyboard: bool = True):
    self.base_dir = Path(os.getenv('G80_ML_CASE_DIR', '/data/radar/ml_cases_v49'))
    self.disable_marker = Path(os.getenv('G80_ML_DISABLE_MARKER', '/data/radar/DISABLE_G80_ML_CASES'))
    self.pre_s = _env_number('G80_ML_PRE_SEC', 5.0, 1.0, 15.0)
    self.post_s = _env_number('G80_ML_POST_SEC', 3.0, 1.0, 15.0)
    self.sample_hz = _env_number('G80_ML_SAMPLE_HZ', 10.0, 2.0, 10.0)
    self.min_interval_ns = int(1e9 / self.sample_hz)
    self.ring = deque()  # (mono_ns, serialized frame); RAM only
    self.last_sample_ns = 0
    self.key_q: queue.SimpleQueue = queue.SimpleQueue()
    self.stop_evt = threading.Event()
    self.pending: list[_PendingCase] = []
    self.counter = 0
    self.saved_cases = 0
    self.last_saved_file = ''
    self.last_error = ''
    self._thread = None
    if start_keyboard:
      self._thread = _InputThread(self.key_q, DEFAULT_KEYMAP, self.stop_evt)
      self._thread.start()

  def enabled(self) -> bool:
    return not self.disable_marker.exists()

  def inject_label(self, side: str, label: str, press_ns: int | None = None):
    """Test hook; not used by normal driving."""
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
    out_dir.mkdir(parents=True, exist_ok=True)
    wall = datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%f')[:-3]
    self.counter += 1
    stem = f'{wall}_{press.side}_{press.label}_{self.counter:04d}'
    final = out_dir / f'{stem}.jsonl.gz'
    part = out_dir / f'{stem}.jsonl.gz.part'
    return part, final

  def _manifest(self, p: _PendingCase):
    path = self.base_dir / 'manifest.csv'
    new = not path.exists()
    self.base_dir.mkdir(parents=True, exist_ok=True)
    with path.open('a', newline='', encoding='utf-8') as f:
      w = csv.writer(f)
      if new:
        w.writerow(['file','side','label','press_mono_ns','frames','actual_pre_sec','actual_post_sec'])
      pre = '' if not p.first_frame_ns else round((p.press.mono_ns - p.first_frame_ns) / 1e9, 3)
      post = '' if not p.last_frame_ns else round((p.last_frame_ns - p.press.mono_ns) / 1e9, 3)
      w.writerow([str(p.final_path.relative_to(self.base_dir)), p.press.side, p.press.label,
                  p.press.mono_ns, p.frame_count, pre, post])

  def _start_case(self, press: KeyPress, build_meta: dict):
    if not self.enabled():
      return
    if len(self.pending) >= 8:
      self.last_error = "Too many pending cases (limit 8)"
      return
    start_ns = press.mono_ns - int(self.pre_s * 1e9)
    end_ns = press.mono_ns + int(self.post_s * 1e9)
    part, final = self._paths(press)
    p = _PendingCase(press, start_ns, end_ns, part, final, self.pre_s, self.post_s, build_meta)
    try:
      for mono_ns, line in self.ring:
        p.add(mono_ns, line)
    except Exception:
      p.abort()
      raise
    self.pending.append(p)
    print(f'[G80 ML] {press.side} {press.label} -> capture {self.pre_s:.1f}s before / {self.post_s:.1f}s after', flush=True)

  def _finish_due(self, now_ns: int):
    keep = []
    for p in self.pending:
      if now_ns < p.end_ns:
        keep.append(p)
        continue
      try:
        p.finish()
        self._manifest(p)
        self.saved_cases += 1
        self.last_saved_file = str(p.final_path)
        print(f'[G80 ML] saved {p.final_path} ({p.frame_count} frames)', flush=True)
      except Exception as e:
        self.last_error = repr(e)
        p.abort()
    self.pending = keep

  def update(self, core: dict, now_ns: int, raw_objects=None, filtered_objects=None):
    """Call once per V49 publish loop (10 Hz). Never raises into live_service."""
    try:
      now_ns = int(now_ns)
      # Even when disabled, finish already-triggered cases cleanly; do not start new ones.
      if self.last_sample_ns and now_ns - self.last_sample_ns < self.min_interval_ns * 0.85:
        self._drain_keys(core.get('runtime_versions', {}))
        self._finish_due(now_ns)
        return
      self.last_sample_ns = now_ns

      frame = _compact_frame(core, raw_objects, filtered_objects, now_ns)
      line = json.dumps(frame, separators=(',', ':'), allow_nan=False)
      self.ring.append((now_ns, line))
      self._trim_ring(now_ns)

      # Existing cases receive the current post-trigger frame.
      healthy = []
      for p in self.pending:
        try:
          p.add(now_ns, line)
          healthy.append(p)
        except Exception as e:
          self.last_error = repr(e)
          p.abort()
      self.pending = healthy

      self._drain_keys(core.get('runtime_versions', {}))
      self._finish_due(now_ns)
    except Exception as e:
      self.last_error = repr(e)
      # This module is monitor-only. V49 must continue even if storage/HID fails.

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

  def close(self):
    self.stop_evt.set()
    if self._thread is not None:
      self._thread.join(timeout=0.6)
      for fd in list(self._thread.fds):
        self._thread._drop(fd)
    for p in self.pending:
      p.abort()
    self.pending.clear()

  def status(self) -> dict:
    return {
      'schema': SCHEMA, 'collector_version': COLLECTOR_VERSION, 'enabled': self.enabled(),
      'keyboard_running': bool(self._thread and self._thread.is_alive()),
      'input_devices': len(self._thread.fds) if self._thread else 0, 'pre_sec': self.pre_s, 'post_sec': self.post_s,
      'sample_hz': self.sample_hz, 'ram_frames': len(self.ring), 'pending_cases': len(self.pending),
      'saved_cases': self.saved_cases, 'last_saved_file': self.last_saved_file,
      'output_dir': str(self.base_dir), 'last_error': self.last_error,
    }


def _key_test():
  print('G80 V49 ML keypad test. Press F13..F18; Ctrl-C to stop.')
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
    print('This module is integrated into V49 live_service. Use --key-test to test the two keypads.')

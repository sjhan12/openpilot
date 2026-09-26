#!/usr/bin/env python3
"""
Persistent G80 shadow-evaluation logger.

Default:
  enabled
  /data/radar/shadow_YYYYMMDD_HHMMSS.jsonl.gz
  5 Hz periodic sampling
  immediate extra sample on L1/L2/CUT-IN state changes
  30 minute rotation
  64 MB approximate uncompressed rotation
  1 second gzip flush

The logger stores evaluation data only. It never publishes radarState or CAN.
"""
from __future__ import annotations
from datetime import datetime
from pathlib import Path
import atexit
import gzip
import json
import os
import time


def _env_bool(name: str, default: bool) -> bool:
  v = os.getenv(name)
  if v is None:
    return default
  return v.strip().lower() not in ('0', 'false', 'no', 'off', '')


def _compact_obj(o: dict) -> dict:
  keys = (
    'key','source','x','y','vx','front_sector','sector','corner_fused_id',
    'member_count','teacher_match','scc_teacher_confirmed','front_link',
    'corner_link_id','camera_confirmed','camera_prob','camera_id','camera_key',
    'camera_only','sensor_fusion','camera_match_cost','camera_dx_m',
    'camera_dy_m','camera_dv_mps','recv_ns','log_ns',
    'vehicle_id','vehicle_key','vehicle_anchor_key','vehicle_member_count','vehicle_duplicates_merged','vehicle_footprint_merged',
    'vehicle_span_x_m','vehicle_span_y_m','vehicle_cluster_keys','vehicle_cluster_sources',
    'camera_hypothesis_keys','camera_hypothesis_count','camera_hypotheses_merged',
    'road_s','road_d','road_path_x','road_path_y','road_lane_index','road_lane','road_lane_source','road_projection_valid',
    'preview_quality','kalman_valid','kalman_track_key','kalman_age_frames','kalman_age_s',
    'kf_x','kf_y','kf_vx','kf_vy','kf_ax','kf_ay','kf_x_sigma','kf_y_sigma','kf_vx_sigma','kf_vy_sigma','kf_frenet_valid','kf_s','kf_s_dot','kf_s_ddot',
    'kf_d','kf_d_dot','kf_d_ddot','kf_s_sigma','kf_d_sigma','kf_s_dot_sigma','kf_d_dot_sigma','kf_lane_index','kf_lane','kf_ttlc_s','kf_lateral_motion','kf_motion_confident','kf_cutin_candidate',
    'kalman_trajectory'
  )
  return {k:o.get(k) for k in keys if k in o and o.get(k) is not None}


class ShadowLogger:
  def __init__(self,
               log_dir: str | None = None,
               enabled: bool | None = None,
               hz: float | None = None,
               rotate_min: float | None = None,
               max_mb: float | None = None,
               flush_sec: float | None = None):
    self.log_dir = Path(log_dir or os.getenv('G80_SHADOW_LOG_DIR', '/data/radar'))
    self.enabled = _env_bool('G80_SHADOW_LOG', True) if enabled is None else bool(enabled)
    self.hz = max(0.2, float(os.getenv('G80_SHADOW_LOG_HZ', '5.0')) if hz is None else float(hz))
    self.rotate_min = max(1.0, float(os.getenv('G80_SHADOW_LOG_ROTATE_MIN', '30')) if rotate_min is None else float(rotate_min))
    self.max_bytes = int(max(1.0, float(os.getenv('G80_SHADOW_LOG_MAX_MB', '64')) if max_mb is None else float(max_mb)) * 1024 * 1024)
    self.flush_sec = max(0.2, float(os.getenv('G80_SHADOW_LOG_FLUSH_SEC', '1.0')) if flush_sec is None else float(flush_sec))

    self.fp = None
    self.path: Path | None = None
    self.final_path: Path | None = None
    self.file_start_mono = 0.0
    self.next_periodic_mono = 0.0
    self.next_flush_mono = 0.0
    self.uncompressed_bytes = 0
    self.records = 0
    self.event_records = 0
    self.files_created = 0
    self.last_error = ''
    self.last_signature = None
    self.last_write_ns = 0

    atexit.register(self.close)

  def _filename(self) -> Path:
    stamp = datetime.now().astimezone().strftime('%Y%m%d_%H%M%S')
    base = self.log_dir / f'shadow_{stamp}.jsonl.gz'
    if not base.exists() and not Path(str(base) + '.part').exists():
      return base
    i = 1
    while True:
      p = self.log_dir / f'shadow_{stamp}_{i:02d}.jsonl.gz'
      if not p.exists() and not Path(str(p) + '.part').exists():
        return p
      i += 1

  def _open(self, now_mono: float):
    if not self.enabled or self.fp is not None:
      return
    try:
      self.log_dir.mkdir(parents=True, exist_ok=True)
      self.final_path = self._filename()
      self.path = Path(str(self.final_path) + '.part')
      self.fp = gzip.open(self.path, 'at', encoding='utf-8', compresslevel=5)
      self.file_start_mono = now_mono
      self.next_flush_mono = now_mono + self.flush_sec
      self.uncompressed_bytes = 0
      self.files_created += 1
      header = {
        'type':'header',
        'format':'g80_shadow_log',
        'format_version':5,
        'service_version':22,
        'created':datetime.now().astimezone().isoformat(timespec='seconds'),
        'config':{
          'hz':self.hz,
          'rotate_min':self.rotate_min,
          'max_mb':round(self.max_bytes/1024/1024,1),
          'flush_sec':self.flush_sec,
          'log_dir':str(self.log_dir),
          'vehicle_footprint_m':[4.8,2.1],
          'vehicle_vrel_gate_mps':3.0,
          'kalman_model':'CA-6D Cartesian + CA-6D Frenet',
          'kalman_horizons_s':[0.5,1.0,2.0,3.0],
        },
        'control_connected':False,
        'publishes_radarState':False,
        'can_tx':False,
      }
      self._write_line(header)
      self.fp.flush()
    except Exception as e:
      self.last_error = f'open: {type(e).__name__}: {e}'
      self.fp = None
      self.path = None

  def _write_line(self, record: dict):
    if self.fp is None:
      return
    s = json.dumps(record, separators=(',',':'), ensure_ascii=False)
    self.fp.write(s + '\n')
    self.uncompressed_bytes += len(s.encode('utf-8')) + 1

  def _signature(self, shadow: dict, kalman_stats: dict | None = None):
    l1 = shadow.get('leadOne', {}) or {}
    l2 = shadow.get('leadTwo', {}) or {}
    st = shadow.get('stats', {}) or {}
    return (
      bool(l1.get('status')), l1.get('key'), l1.get('reason'),
      bool(l2.get('status')), l2.get('key'), l2.get('reason'),
      int(st.get('confirmed_cutin_count', 0) or 0),
      int(st.get('stationary_supported_count', 0) or 0),
      bool(st.get('path_valid')), bool(st.get('v_ego_valid')),
      int((kalman_stats or {}).get('cutin_candidates', 0) or 0),
    )

  def _rotate_due(self, now_mono: float) -> bool:
    return (
      self.fp is not None
      and ((now_mono - self.file_start_mono) >= self.rotate_min * 60.0
           or self.uncompressed_bytes >= self.max_bytes)
    )

  def _build_record(self, core: dict, model_path, v_ego, model_path_recv_ns, v_ego_recv_ns,
                    diag: dict | None, now_ns: int, event: bool) -> dict:
    shadow = core.get('shadow_leads', {}) or {}
    return {
      'type':'sample',
      'wall_time':datetime.now().astimezone().isoformat(timespec='milliseconds'),
      'mono_ns':int(now_ns),
      'event':bool(event),
      'v_ego_mps':float(v_ego),
      'v_ego_recv_ns':int(v_ego_recv_ns or 0),
      'model_path_recv_ns':int(model_path_recv_ns or 0),
      'model_path':[[round(float(x),3),round(float(y),3)] for x,y in (model_path or [])],
      'road_model_summary':{
        'fresh':(core.get('road_model',{}) or {}).get('fresh'),
        'age_ms':(core.get('road_model',{}) or {}).get('age_ms'),
        'curve_direction':(core.get('road_model',{}) or {}).get('curve_direction'),
        'path_x_min_m':(core.get('road_model',{}) or {}).get('path_x_min_m'),
        'path_x_max_m':(core.get('road_model',{}) or {}).get('path_x_max_m'),
        'path_y_20m':(core.get('road_model',{}) or {}).get('path_y_20m'),
        'path_y_40m':(core.get('road_model',{}) or {}).get('path_y_40m'),
        'path_y_60m':(core.get('road_model',{}) or {}).get('path_y_60m'),
        'lane_line_probs':(core.get('road_model',{}) or {}).get('lane_line_probs',[]),
        'confident_lane_lines':(core.get('road_model',{}) or {}).get('confident_lane_lines'),
      },
      'shadow':shadow,
      'sensor_fused_objects':[_compact_obj(o) for o in core.get('sensor_fused_objects',[])],
      'radar_fused_objects':[_compact_obj(o) for o in core.get('radar_fused_objects',[])],
      'corner_fused_objects':[_compact_obj(o) for o in core.get('corner_fused_objects',[])],
      'front_sensor_objects':[_compact_obj(o) for o in core.get('front_sensor_objects',[])],
      'standard_front_preview':core.get('standard_front_preview',[]),
      'standard_front_preview_stats':core.get('standard_front_preview_stats',{}),
      'kalman_motion_stats':core.get('kalman_motion_stats',{}),
      'corner_kalman_motion_stats':core.get('corner_kalman_motion_stats',{}),
      'front_kalman_motion_stats':core.get('front_kalman_motion_stats',{}),
      'camera_leads':[_compact_obj(o) for o in core.get('camera_leads',[])],
      'camera_matches':core.get('camera_fusion_matches',[]),
      'camera_fusion_stats':core.get('camera_fusion_stats',{}),
      'corner_fusion_stats':core.get('corner_fusion_stats',{}),
      'scc_teacher':core.get('scc_teacher',{}),
      'rear_teacher':core.get('teacher_rear',[]),
      'corner_front_associations':core.get('corner_front_associations',[]),
      'zones':core.get('zones',{}),
      'diag':{
        k:(diag or {}).get(k) for k in (
          'corner_A_frames','corner_B_frames','corner_decoded_total',
          'corner_rear_total','corner_front_total','model_frames',
          'camera_leads_latest','model_transport_lag_ms','last_transport_lag_ms'
        ) if k in (diag or {})
      },
    }

  def maybe_write(self, core: dict, model_path, v_ego: float,
                  model_path_recv_ns: int, v_ego_recv_ns: int,
                  diag: dict | None, now_ns: int) -> bool:
    if not self.enabled:
      return False

    now_mono = time.monotonic()
    shadow = core.get('shadow_leads', {}) or {}
    sig = self._signature(shadow, core.get('kalman_motion_stats', {}))
    event = self.last_signature is not None and sig != self.last_signature
    self.last_signature = sig

    periodic = now_mono >= self.next_periodic_mono
    if not periodic and not event:
      return False

    # Avoid endless empty off-road records, but preserve transitions as event samples.
    stats = shadow.get('stats', {}) or {}
    production_valid = bool((shadow.get('comparison', {}) or {}).get('production_valid'))
    meaningful = (
      int(stats.get('fresh_object_count', 0) or 0) > 0
      or production_valid
      or len(core.get('camera_leads', [])) > 0
    )
    if not meaningful and not event:
      self.next_periodic_mono = now_mono + 1.0 / self.hz
      return False

    if self._rotate_due(now_mono):
      self.close()

    self._open(now_mono)
    if self.fp is None:
      return False

    try:
      rec = self._build_record(core, model_path, v_ego, model_path_recv_ns,
                               v_ego_recv_ns, diag, now_ns, event)
      self._write_line(rec)
      self.records += 1
      if event:
        self.event_records += 1
      self.last_write_ns = int(now_ns)
      self.next_periodic_mono = now_mono + 1.0 / self.hz
      if now_mono >= self.next_flush_mono:
        self.fp.flush()
        self.next_flush_mono = now_mono + self.flush_sec
      return True
    except Exception as e:
      self.last_error = f'write: {type(e).__name__}: {e}'
      try:
        self.close()
      except Exception:
        pass
      return False

  def status(self) -> dict:
    return {
      'enabled':self.enabled,
      'directory':str(self.log_dir),
      'path':str(self.path) if self.path else '',
      'filename':self.path.name if self.path else '',
      'hz':self.hz,
      'records':self.records,
      'event_records':self.event_records,
      'files_created':self.files_created,
      'approx_uncompressed_mb':round(self.uncompressed_bytes/1024/1024,3),
      'last_write_ns':self.last_write_ns,
      'last_error':self.last_error,
    }

  def close(self):
    fp = self.fp
    part = self.path
    final = self.final_path
    self.fp = None
    if fp is not None:
      try:
        fp.flush()
      except Exception:
        pass
      try:
        fp.close()
      except Exception:
        pass
      # Only a cleanly closed gzip stream receives the final .jsonl.gz name.
      # Abrupt power loss leaves *.part, clearly marking a possibly truncated log.
      if part is not None and final is not None:
        try:
          part.replace(final)
          self.path = final
        except Exception as e:
          self.last_error = f'finalize: {type(e).__name__}: {e}'

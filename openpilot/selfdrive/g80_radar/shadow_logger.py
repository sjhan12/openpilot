#!/usr/bin/env python3
"""
Persistent G80 shadow-evaluation logger.

Default:
  enabled
  /data/radar/shadow_YYYYMMDD_HHMMSS.jsonl.gz
  2 Hz periodic sampling (V38R4 source-trace default)
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

from openpilot.selfdrive.g80_radar.build_info import BUILD_VERSION, BUILD_TAG, LOGGER_FORMAT_VERSION

LOGGER_SERVICE_VERSION = BUILD_VERSION


def _env_bool(name: str, default: bool) -> bool:
  v = os.getenv(name)
  if v is None:
    return default
  return v.strip().lower() not in ('0', 'false', 'no', 'off', '')


def _compact_obj(o: dict) -> dict:
  keys = (
    'key','source','x','y','vx','front_sector','sector','corner_fused_id',
    'member_count','teacher_match','rear_teacher_confirmed','teacher_error_m','rear_teacher_sector','rear_teacher_distance_m','rear_teacher_predicted_distance_m','rear_teacher_match_gate_m','scc_teacher_confirmed','front_link',
    'corner_link_id','camera_confirmed','camera_prob','camera_id','camera_key',
    'camera_only','sensor_fusion','camera_match_cost','camera_dx_m',
    'camera_dy_m','camera_dv_mps','recv_ns','log_ns',
    'vehicle_id','vehicle_key','canonical_id','canonical_key','canonical_valid','canonical_age_frames','canonical_track_duration_s','canonical_match_reason','canonical_match_cost','canonical_alias_overlap','canonical_gap_ms','canonical_reacquired','canonical_reacquire_count','canonical_domains','canonical_primary_domain','canonical_domain_history','canonical_source_transition','canonical_handoff_count','canonical_alias_count','canonical_candidate_count','vehicle_anchor_key','vehicle_member_count','vehicle_duplicates_merged','vehicle_footprint_merged',
    'vehicle_span_x_m','vehicle_span_y_m','vehicle_cluster_keys','vehicle_cluster_sources','vehicle_merge_reason','source_mask','source_age_ms','display_state','trace_local_key','trace_match_method','trace_unmatched','trace_canonical_domains','corner_debug_role',
    'camera_hypothesis_keys','camera_hypothesis_count','camera_hypotheses_merged',
    'road_s','road_d','road_path_x','road_path_y','road_lane_index','road_lane','road_lane_source','road_projection_valid','road_projection_endpoint_overshoot_m',
    'preview_quality','kalman_valid','kalman_track_key','kalman_age_frames','kalman_age_s','kf_canonical_key_match','kf_reset_suspect','kf_dormant_preserved',
    'kf_x','kf_y','kf_vx','kf_vy','kf_ax','kf_ay','kf_x_sigma','kf_y_sigma','kf_vx_sigma','kf_vy_sigma','kf_frenet_valid','kf_s','kf_s_dot','kf_s_ddot',
    'kf_d','kf_d_dot','kf_d_ddot','kf_s_sigma','kf_d_sigma','kf_s_dot_sigma','kf_d_dot_sigma','kf_lane_index','kf_lane','kf_ttlc_s','kf_lateral_motion','kf_motion_confident',
    'kf_lateral_candidate','kf_low_speed_lateral_candidate','kf_cutin_speed_class','kf_cutin_candidate','kf_cutin_confirmed','kf_cutin_score','kf_cutin_persistence_s',
    'kf_lateral_prediction_mode','kf_lateral_prediction_limited',
    'imm_valid','imm_api_version','imm_track_key','imm_age_frames','imm_age_s','imm_coord_source','imm_reset_suspect','imm_reset_count','imm_reinit_count','imm_reinit_reason','imm_eval_age_ms','imm_interaction_relevant','imm_skipped_reason',
    'imm_s','imm_s_dot','imm_s_ddot','imm_d','imm_d_dot','imm_d_ddot','imm_s_sigma','imm_d_sigma','imm_d_dot_sigma',
    'imm_prob_cv','imm_prob_ca','imm_prob_maneuver','imm_dominant_model','imm_lane_index','imm_lane','imm_ttlc_s','imm_motion_confident','imm_maneuver_candidate'
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
    self.hz = max(0.2, float(os.getenv('G80_SHADOW_LOG_HZ', '2.0')) if hz is None else float(hz))
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
    # V44: identify each logger/service instance so overlapping logs can be diagnosed.
    self.pid = os.getpid()
    self.instance_id = f'{self.pid}-{time.monotonic_ns()}'

    atexit.register(self.close)

  def _filename(self) -> Path:
    stamp = datetime.now().astimezone().strftime('%Y%m%d_%H%M%S')
    base = self.log_dir / f'shadow_v{BUILD_VERSION}_{stamp}.jsonl.gz'
    if not base.exists() and not Path(str(base) + '.part').exists():
      return base
    i = 1
    while True:
      p = self.log_dir / f'shadow_v{BUILD_VERSION}_{stamp}_{i:02d}.jsonl.gz'
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
      self.fp = gzip.open(self.path, 'at', encoding='utf-8', compresslevel=1)
      self.file_start_mono = now_mono
      self.next_flush_mono = now_mono + self.flush_sec
      self.uncompressed_bytes = 0
      self.files_created += 1
      header = {
        'type':'header',
        'format':'g80_shadow_log',
        'format_version':LOGGER_FORMAT_VERSION,
        'service_version':LOGGER_SERVICE_VERSION,
        'build_tag':BUILD_TAG,
        'created':datetime.now().astimezone().isoformat(timespec='seconds'),
        'pid':self.pid,
        'instance_id':self.instance_id,
        'config':{
          'hz':self.hz,
          'rotate_min':self.rotate_min,
          'max_mb':round(self.max_bytes/1024/1024,1),
          'flush_sec':self.flush_sec,
          'log_dir':str(self.log_dir),
          'vehicle_footprint_m':[4.8,2.1],
          'vehicle_vrel_gate_mps':2.5,
          'kalman_model':'KF4: selective CA longitudinal + bounded lateral; dormant Canonical identity preserved',
          'imm_model':'IMM3: CV + CA + MANEUVER; 3Hz selective ROI, max 8, cached between ticks',
          'kalman_horizons_s':[0.5,1.0,2.0,3.0],
          'coordinate_x_origin':'ego_front_bumper_display_reference',
          'decoded_object_x_adjustment_m':0.0,
          'rear_teacher_one_to_one':True,
          'rear_teacher_match_gate_m':0.8,
          'highway_cutin_min_v_ego_mps':5.0,
          'c4_path_projection_margin_m':0.75,
          'canonical360_identity_authority':True,
          'canonical360_ttl_s':1.5,
          'canonical360_identity_safety':'tight cluster aliases + 650ms reacquire diagnostic',
          'scc_teacher_policy':'path-aware final-object match; adjacent-lane streak cannot confirm; SCC+CAM strong L1 handoff',
          'performance_policy':'V44: browser-only seven-stage audit at UI 8Hz or idle 2Hz; legacy V40 road/vehicle visual; KF4 max10; IMM3 2.5Hz/max6; sparse immediate shadow events',
          'future_gap_evaluator':True,
          'future_gap_policy':'V44 FG12: same-key longitudinal+lateral time windows must overlap <=3s for DANGER; incoming outside lane needs persistence; raw precommit decision latch; FG8 TURN/rebase retained',
          'traffic_signal_probe':'monitor-only E2E heuristic: path/action + sunnypilot green alert + modelDataV2SP turn path; scores are not probabilities',
          'web_stage_contract':'V44 seven read-only stages; V40 legacy road/vehicle visual; diagnostic validity-only raw filter and selected L1/L2',
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

  def _signature(self, shadow: dict, kalman_stats: dict | None = None, canonical_stats: dict | None = None, imm_stats: dict | None = None, future_gap: dict | None = None, ego_state: dict | None = None, traffic_signal: dict | None = None):
    """Sparse event signature.

    V30 included per-frame new/reacquire/handoff counts, so ~75-99% of records
    became "events" and gzip/JSON work ran almost every publish.  V31 reserves
    immediate records for semantically important lead/cut-in transitions; the
    full canonical/IMM/FG7/DEC3 state is still captured by the 2 Hz periodic stream.
    """
    # V38R4: immediate records are reserved for actual driver/maneuver events.
    # Passive left/right preview decisions, path-valid toggles, lead identity churn and
    # generic signal-probe state are captured by the 2 Hz periodic stream instead.
    st = shadow.get('stats', {}) or {}
    fg = future_gap or {}
    di = fg.get('driver_intent', {}) or {}
    ego = ego_state or {}
    tl = traffic_signal or {}
    return (
      di.get('state'), di.get('side'),
      bool(ego.get('leftBlinker')), bool(ego.get('rightBlinker')),
      bool(st.get('leadOne_strong_handoff')),
      bool(int(st.get('confirmed_cutin_count',0) or 0) > 0),
      bool(int((kalman_stats or {}).get('cutin_confirmed',0) or 0) > 0),
      bool(tl.get('green_confirmed')), bool(tl.get('sunnypilot_green_alert')),
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
      'runtime_versions':core.get('runtime_versions',{}),
      'runtime_mismatch':bool(core.get('runtime_mismatch',False)),
      'coordinate_frame':core.get('coordinate_frame',{}),
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
      'canonical_tracker_stats':core.get('canonical_tracker_stats',{}),
      'view_consistency_stats':core.get('view_consistency_stats',{}),
      'web_stage_stats':core.get('web_stage_stats',{}),
      'web_stage_errors':core.get('web_stage_errors',[]),
      'radar_fused_objects':[_compact_obj(o) for o in core.get('radar_fused_objects',[])],
      'corner_fused_objects':[_compact_obj(o) for o in core.get('corner_fused_objects',[])],
      'corner_candidate_objects':[_compact_obj(o) for o in core.get('corner_candidate_objects',[])[:24]],
      'front_sensor_objects':[_compact_obj(o) for o in core.get('front_sensor_objects',[])],
      'standard_front_preview':core.get('standard_front_preview',[]),
      'standard_front_preview_stats':core.get('standard_front_preview_stats',{}),
      'kalman_motion_stats':core.get('kalman_motion_stats',{}),
      'imm_motion_stats':core.get('imm_motion_stats',{}),
      'future_gap':core.get('future_gap',{}),
      'traffic_signal_probe':core.get('traffic_signal_probe',{}),
      'ego_state':core.get('ego_state',{}),
      'performance_stats':core.get('performance_stats',{}),
      'corner_kalman_motion_stats':core.get('corner_kalman_motion_stats',{}),
      'front_kalman_motion_stats':core.get('front_kalman_motion_stats',{}),
      'camera_leads':[_compact_obj(o) for o in core.get('camera_leads',[])],
      'camera_matches':core.get('camera_fusion_matches',[]),
      'camera_fusion_stats':core.get('camera_fusion_stats',{}),
      'corner_fusion_stats':core.get('corner_fusion_stats',{}),
      'scc_teacher':core.get('scc_teacher',{}),
      'rear_teacher':core.get('teacher_rear',[]),
      'rear_teacher_match_stats':core.get('rear_teacher_match_stats',{}),
      'corner_front_associations':core.get('corner_front_associations',[]),
      'zones':core.get('zones',{}),
      'validation_summary':self._validation_summary(core),
      'diag':{
        k:(diag or {}).get(k) for k in (
          'corner_A_frames','corner_B_frames','corner_decoded_total',
          'corner_rear_total','corner_front_total','model_frames',
          'camera_leads_latest','model_transport_lag_ms','last_transport_lag_ms'
        ) if k in (diag or {})
      },
    }

  def _validation_summary(self, core: dict) -> dict:
    shadow = core.get('shadow_leads', {}) or {}
    l1 = shadow.get('leadOne', {}) or {}
    teacher = core.get('scc_teacher', {}) or {}
    objs = core.get('sensor_fused_objects', []) or []
    canon = core.get('canonical_tracker_stats', {}) or {}
    kf_valid = sum(1 for o in objs if o.get('kalman_valid'))
    kf_frenet = sum(1 for o in objs if o.get('kf_frenet_valid'))
    kf_cutin = sum(1 for o in objs if o.get('kf_cutin_candidate'))
    kf_cutin_confirmed = sum(1 for o in objs if o.get('kf_cutin_confirmed'))
    imm_valid = sum(1 for o in objs if o.get('imm_valid'))
    imm_man = sum(1 for o in objs if o.get('imm_maneuver_candidate'))
    imm_reset = sum(1 for o in objs if o.get('imm_reset_suspect'))
    out = {
      'canonical_visible_tracks':int(canon.get('visible_tracks',0) or 0),
      'canonical_active_tracks':int(canon.get('active_tracks',0) or 0),
      'canonical_new_tracks':int(canon.get('new_tracks',0) or 0),
      'canonical_reacquired_tracks':int(canon.get('reacquired_tracks',0) or 0),
      'canonical_source_handoffs':int(canon.get('source_handoffs',0) or 0),
      'canonical_ambiguous_objects':int(canon.get('ambiguous_objects',0) or 0),
      'canonical_continuity_ratio':canon.get('continuity_ratio'),
      'kalman_objects':kf_valid,
      'kalman_frenet_objects':kf_frenet,
      'kalman_cutin_candidates':kf_cutin,
      'kalman_cutin_confirmed':kf_cutin_confirmed,
      'imm_objects':imm_valid,
      'imm_maneuver_candidates':imm_man,
      'imm_reset_suspects':imm_reset,
      'future_gap_left_incoming':int((((core.get('future_gap',{}) or {}).get('left',{}) or {}).get('incoming_count',0) or 0)),
      'future_gap_right_incoming':int((((core.get('future_gap',{}) or {}).get('right',{}) or {}).get('incoming_count',0) or 0)),
      'scc_teacher_usable':bool(teacher.get('teacher_usable')),
      'shadow_l1_present':bool(l1.get('status')),
      'rear_teacher_usable_count':sum(1 for t in (core.get('teacher_rear',[]) or []) if t.get('teacher_usable')),
      'rear_teacher_matched_count':int((core.get('rear_teacher_match_stats',{}) or {}).get('matched_count',0) or 0),
    }
    rear_errors=(core.get('rear_teacher_match_stats',{}) or {}).get('errors_m',[]) or []
    if rear_errors:
      try:
        out['rear_teacher_abs_error_max_m']=round(max(abs(float(v)) for v in rear_errors),3)
      except Exception:
        pass
    if teacher.get('teacher_usable') and l1.get('status'):
      try:
        de = float(l1.get('dRel')) - float(teacher.get('distance_m'))
        ve = float(l1.get('vRel')) - float(teacher.get('rel_speed_mps'))
        out.update({
          'scc_l1_d_error_m':round(de,3),
          'scc_l1_v_error_mps':round(ve,3),
          'scc_l1_disagreement':abs(de) > 3.0 or abs(ve) > 3.0,
          'scc_l1_camera_confirmed':bool(l1.get('cameraConfirmed')),
          'scc_l1_scc_confirmed':bool(l1.get('sccConfirmed')),
        })
      except Exception:
        pass
    return out

  def maybe_write(self, core: dict, model_path, v_ego: float,
                  model_path_recv_ns: int, v_ego_recv_ns: int,
                  diag: dict | None, now_ns: int) -> bool:
    if not self.enabled:
      return False

    now_mono = time.monotonic()
    shadow = core.get('shadow_leads', {}) or {}
    sig = self._signature(shadow, core.get('kalman_motion_stats', {}), core.get('canonical_tracker_stats', {}), core.get('imm_motion_stats', {}), core.get('future_gap', {}), core.get('ego_state', {}), core.get('traffic_signal_probe', {}))
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
      or bool((core.get('traffic_signal_probe', {}) or {}).get('context_active'))
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
      'service_version':LOGGER_SERVICE_VERSION,
      'format_version':LOGGER_FORMAT_VERSION,
      'build_tag':BUILD_TAG,
      'directory':str(self.log_dir),
      'path':str(self.path) if self.path else '',
      'filename':self.path.name if self.path else '',
      'hz':self.hz,
      'records':self.records,
      'event_records':self.event_records,
      'files_created':self.files_created,
      'approx_uncompressed_mb':round(self.uncompressed_bytes/1024/1024,3),
      'last_write_ns':self.last_write_ns,
      'pid':self.pid,
      'instance_id':self.instance_id,
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

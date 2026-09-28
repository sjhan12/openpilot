#!/usr/bin/env python3
from __future__ import annotations
from openpilot.selfdrive.g80_radar.build_info import ANDROID_PROTOCOL_VERSION
PROTOCOL_VERSION = ANDROID_PROTOCOL_VERSION

def _sector_color_key(o: dict) -> str | None:
  src = str(o.get('source',''))
  sources = o.get('vehicle_cluster_sources') or []
  has_corner = src.startswith('corner') or o.get('corner_link_id') is not None or any(str(v).startswith('corner') for v in sources)
  if not has_corner:
    return None
  sec = o.get('sector')
  if sec in ('FL','FR','RL','RR'):
    return sec
  try:
    x, y = float(o.get('x')), float(o.get('y'))
    if x > 0.5 and y > 1.2: return 'FL'
    if x > 0.5 and y < -1.2: return 'FR'
    if x < -0.5 and y > 1.2: return 'RL'
    if x < -0.5 and y < -1.2: return 'RR'
  except Exception:
    pass
  return None

def _obj(o: dict) -> dict:
  d={'id':o.get('canonical_key') or o.get('vehicle_key') or o.get('key'),'x':o.get('x'),'y':o.get('y'),'vx':o.get('vx'),'source':o.get('source')}
  ck=_sector_color_key(o)
  if ck is not None:d['corner_color_key']=ck
  for k in ('sector','front_sector','corner_fused_id','member_count','teacher_match','rear_teacher_confirmed','teacher_error_m','rear_teacher_sector','rear_teacher_distance_m','rear_teacher_predicted_distance_m','rear_teacher_match_gate_m','scc_teacher_confirmed',
            'front_link','corner_link_id','confidence','camera_confirmed','camera_prob','camera_id','camera_key',
            'camera_only','sensor_fusion','camera_match_cost','camera_dx_m','camera_dy_m','camera_dv_mps',
            'vehicle_id','vehicle_key','canonical_id','canonical_key','canonical_valid','canonical_age_frames','canonical_track_duration_s','canonical_match_reason','canonical_match_cost','canonical_alias_overlap','canonical_gap_ms','canonical_reacquired','canonical_reacquire_count','canonical_domains','canonical_primary_domain','canonical_domain_history','canonical_source_transition','canonical_handoff_count','canonical_alias_count','canonical_candidate_count','vehicle_anchor_key','vehicle_member_count','vehicle_duplicates_merged','vehicle_footprint_merged',
            'vehicle_span_x_m','vehicle_span_y_m','vehicle_cluster_sources','vehicle_merge_reason',
            'camera_hypothesis_count','camera_hypotheses_merged',
            'road_s','road_d','road_path_x','road_path_y','road_lane_index','road_lane','road_lane_source','road_projection_valid','road_projection_endpoint_overshoot_m','preview_quality',
            'kalman_valid','kalman_track_key','kalman_age_frames','kalman_age_s','kf_canonical_key_match','kf_reset_suspect','kf_x','kf_y','kf_vx','kf_vy','kf_ax','kf_ay','kf_x_sigma','kf_y_sigma','kf_vx_sigma','kf_vy_sigma',
            'kf_frenet_valid','kf_s','kf_s_dot','kf_s_ddot','kf_d','kf_d_dot','kf_d_ddot','kf_s_sigma','kf_d_sigma','kf_s_dot_sigma','kf_d_dot_sigma','kf_lane_index','kf_lane',
            'kf_ttlc_s','kf_lateral_motion','kf_motion_confident','kf_lateral_candidate','kf_low_speed_lateral_candidate','kf_cutin_speed_class',
            'kf_cutin_candidate','kf_cutin_confirmed','kf_cutin_score','kf_cutin_persistence_s','kf_lateral_prediction_mode','kf_lateral_prediction_limited','kalman_trajectory',
            'imm_valid','imm_api_version','imm_track_key','imm_age_frames','imm_age_s','imm_coord_source','imm_reset_suspect','imm_reset_count','imm_reinit_count','imm_reinit_reason','imm_eval_age_ms','imm_interaction_relevant','imm_skipped_reason',
            'imm_s','imm_s_dot','imm_s_ddot','imm_d','imm_d_dot','imm_d_ddot','imm_s_sigma','imm_d_sigma','imm_d_dot_sigma',
            'imm_prob_cv','imm_prob_ca','imm_prob_maneuver','imm_dominant_model','imm_lane_index','imm_lane','imm_ttlc_s','imm_motion_confident','imm_maneuver_candidate'):
    if k in o and o.get(k) is not None:d[k]=o.get(k)
  return d

def build_render_packet(state: dict) -> dict:
  final_all=state.get('sensor_fused_objects',state.get('all_fused_objects',[]))
  final_front=state.get('front_sensor_objects',state.get('front_objects',[]))
  return {
    'protocol':'g80_radar_render','protocol_version':PROTOCOL_VERSION,
    'state_version':state.get('version',0),'mono_ns':state.get('mono_ns',0),
    'runtime_versions':state.get('runtime_versions',{}),'runtime_mismatch':state.get('runtime_mismatch',False),
    'coordinate_frame':state.get('coordinate_frame',{}),
    'all':[_obj(o) for o in final_all],
    'front':[_obj(o) for o in final_front],
    'corner':[_obj(o) for o in state.get('corner_fused_objects',[])],
    'radar_all':[_obj(o) for o in state.get('radar_fused_objects',[])],
    'radar_front':[_obj(o) for o in state.get('front_objects',[])],
    'camera':[_obj(o) for o in state.get('camera_leads',[])],
    'camera_matches':state.get('camera_fusion_matches',[]),
    'camera_fusion_stats':state.get('camera_fusion_stats',{}),
    'shadow_leads':state.get('shadow_leads',{}),
    'shadow_logger':state.get('shadow_logger',{}),
    'zones':state.get('zones',{}),
    'rear_teacher':state.get('teacher_rear',[]),
    'rear_teacher_match_stats':state.get('rear_teacher_match_stats',{}),
    'scc_teacher':state.get('scc_teacher',{}),
    'scc_front_match':state.get('scc_front_match',{}),
    'road_model':state.get('road_model',{}),
    'canonical_tracker_stats':state.get('canonical_tracker_stats',{}),
    'kalman_motion_stats':state.get('kalman_motion_stats',{}),
    'imm_motion_stats':state.get('imm_motion_stats',{}),
    'performance_stats':state.get('performance_stats',{}),
    'scc_teacher_by_bus':state.get('scc_teacher_by_bus',{}),
    'associations':state.get('corner_front_associations',[]),
    'fusion_stats':state.get('corner_fusion_stats',{}),
    'view_presets':{
      'short':{'rear_m':-15,'front_m':35,'metric_1to1':True},
      'long':{'rear_m':-50,'front_m':100,'metric_1to1':True},
      'wide':{'rear_m':-30,'front_m':70,'lateral_m':12.6,'metric_1to1':False},
      'drive':{'rear_m':-12,'front_m':40,'lateral_m':7.2,'lanes_total':3,'metric_1to1':False,'perspective':True},
    },
  }

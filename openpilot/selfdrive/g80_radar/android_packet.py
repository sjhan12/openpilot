#!/usr/bin/env python3
from __future__ import annotations
PROTOCOL_VERSION = 8

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
  d={'id':o.get('key'),'x':o.get('x'),'y':o.get('y'),'vx':o.get('vx'),'source':o.get('source')}
  ck=_sector_color_key(o)
  if ck is not None:d['corner_color_key']=ck
  for k in ('sector','front_sector','corner_fused_id','member_count','teacher_match','scc_teacher_confirmed',
            'front_link','corner_link_id','confidence','camera_confirmed','camera_prob','camera_id','camera_key',
            'camera_only','sensor_fusion','camera_match_cost','camera_dx_m','camera_dy_m','camera_dv_mps',
            'vehicle_id','vehicle_key','vehicle_anchor_key','vehicle_member_count','vehicle_duplicates_merged','vehicle_footprint_merged',
            'vehicle_span_x_m','vehicle_span_y_m','vehicle_cluster_keys','vehicle_cluster_sources',
            'camera_hypothesis_keys','camera_hypothesis_count','camera_hypotheses_merged',
            'road_s','road_d','road_path_x','road_path_y','road_lane_index','road_lane','road_lane_source','road_projection_valid','preview_quality',
            'kalman_valid','kalman_track_key','kalman_age_frames','kalman_age_s','kf_x','kf_y','kf_vx','kf_vy','kf_ax','kf_ay','kf_x_sigma','kf_y_sigma','kf_vx_sigma','kf_vy_sigma',
            'kf_frenet_valid','kf_s','kf_s_dot','kf_s_ddot','kf_d','kf_d_dot','kf_d_ddot','kf_s_sigma','kf_d_sigma','kf_s_dot_sigma','kf_d_dot_sigma','kf_lane_index','kf_lane',
            'kf_ttlc_s','kf_lateral_motion','kf_motion_confident','kf_cutin_candidate','kalman_trajectory'):
    if k in o and o.get(k) is not None:d[k]=o.get(k)
  return d

def build_render_packet(state: dict) -> dict:
  final_all=state.get('sensor_fused_objects',state.get('all_fused_objects',[]))
  final_front=state.get('front_sensor_objects',state.get('front_objects',[]))
  return {
    'protocol':'g80_radar_render','protocol_version':PROTOCOL_VERSION,
    'state_version':state.get('version',0),'mono_ns':state.get('mono_ns',0),
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
    'scc_teacher':state.get('scc_teacher',{}),
    'scc_front_match':state.get('scc_front_match',{}),
    'road_model':state.get('road_model',{}),
    'kalman_motion_stats':state.get('kalman_motion_stats',{}),
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

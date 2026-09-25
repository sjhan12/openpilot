#!/usr/bin/env python3
from __future__ import annotations
PROTOCOL_VERSION = 1

def _obj(o: dict) -> dict:
  d = {'id':o.get('key'),'x':o.get('x'),'y':o.get('y'),'vx':o.get('vx'),'source':o.get('source')}
  for k in ('sector','front_sector','corner_fused_id','member_count','teacher_match','scc_teacher_confirmed','front_link','corner_link_id','confidence'):
    if k in o and o.get(k) is not None: d[k]=o.get(k)
  return d

def build_render_packet(state: dict) -> dict:
  return {
    'protocol':'g80_radar_render','protocol_version':PROTOCOL_VERSION,'state_version':state.get('version',0),'mono_ns':state.get('mono_ns',0),
    'corner':[_obj(o) for o in state.get('corner_fused_objects',[])],
    'front':[_obj(o) for o in state.get('front_objects',[])],
    'all':[_obj(o) for o in state.get('all_fused_objects',[])],
    'zones':state.get('zones',{}),'rear_teacher':state.get('teacher_rear',[]),'scc_teacher':state.get('scc_teacher',{}),
    'scc_front_match':state.get('scc_front_match',{}),'scc_teacher_by_bus':state.get('scc_teacher_by_bus',{}),'associations':state.get('corner_front_associations',[]),'fusion_stats':state.get('corner_fusion_stats',{}),'view_presets':{'short':{'rear_m':-15,'front_m':35,'metric_1to1':True},'long':{'rear_m':-50,'front_m':100,'metric_1to1':True},'wide':{'rear_m':-30,'front_m':70,'lateral_m':12.6,'metric_1to1':False},'drive':{'rear_m':-15,'front_m':45,'lateral_m':5.8,'lanes_total':3,'metric_1to1':False}}}

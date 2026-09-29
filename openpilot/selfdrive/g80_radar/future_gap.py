#!/usr/bin/env python3
"""V35 shadow Future Gap + Target-Lane Occupancy evaluator.

Consumes Canonical360 vehicles after KF4/IMM3 and computes diagnostic target-lane
geometry for NOW/0.5/1/2/3 s.  V35 keeps the V34 geometry separation and adds temporal incoming + DEC1 diagnostic:
- core occupants: object centre is inside the target lane;
- boundary overlaps: only the object footprint overlaps the target lane;
- confirmed incoming: predicted centre enters and persists in the target lane;
- possible incoming: footprint/one-horizon entry only.

It also exposes diagnostic what-if braking scenarios and shadow-only
SAFE_SHADOW/CAUTION_SHADOW/BLOCKED_SHADOW. These are diagnostic labels only and
are not connected to planner/control.
"""
from __future__ import annotations

import math

from openpilot.selfdrive.g80_radar.road_geometry import LANE_W_M, lane_index_from_d, lane_name

FUTURE_GAP_API_VERSION = 3
HORIZONS_S = (0.0, 0.5, 1.0, 2.0, 3.0)
LANE_CHANGE_DURATION_S = 3.0
EGO_HALF_WIDTH_M = 1.05
OBJECT_HALF_WIDTH_M = 1.05
OBJECT_LONG_MARGIN_M = 2.4
MAX_LAT_SIGMA_MARGIN_M = 1.5
MAX_LONG_SIGMA_MARGIN_M = 5.0
MAX_TTC_S = 20.0

# Diagnostic what-if assumptions only.  These are not safety thresholds.
TARGET_BRAKE_ASSUMPTION_MPS2 = -3.0
EGO_BRAKE_ASSUMPTION_MPS2 = -3.0
INCOMING_EGO_MIN_MANEUVER_PROB = 0.35
INCOMING_STABLE_S = 0.25
INCOMING_FORGET_S = 0.80

# DEC1 diagnostic thresholds. These are shadow-only engineering gates, not
# control limits and not connected to planner/CAN.
DEC_BLOCK_CLEARANCE_M = 5.0
DEC_CAUTION_CLEARANCE_M = 12.0
DEC_BLOCK_BOUNDARY_M = 2.0
DEC_CAUTION_BOUNDARY_M = 5.0
DEC_BLOCK_TTC_S = 3.0
DEC_CAUTION_TTC_S = 5.0
DEC_BLOCK_INCOMING_ETA_S = 2.5
DEC_CAUTION_INCOMING_ETA_S = 3.0
DEC_BLOCK_INCOMING_ABS_S_M = 25.0
DEC_CAUTION_INCOMING_ABS_S_M = 40.0


def _finite(v, default=None):
  try:
    x = float(v)
  except Exception:
    return default
  return x if math.isfinite(x) else default


def _key(o: dict) -> str:
  return str(o.get('canonical_key') or o.get('vehicle_key') or o.get('key') or '')


def _find_traj_point(o: dict, t: float) -> tuple[dict | None, str]:
  if t <= 1e-9:
    return None, 'current'
  if o.get('imm_valid'):
    for p in o.get('imm_trajectory') or []:
      if abs(float(p.get('t', -999.0)) - t) < 1e-6:
        return p, 'IMM'
  if o.get('kalman_valid'):
    for p in o.get('kalman_trajectory') or []:
      if abs(float(p.get('t', -999.0)) - t) < 1e-6:
        return p, 'KF3'
  return None, 'CV_FALLBACK'


def _state_at(o: dict, t: float) -> dict | None:
  if t <= 1e-9:
    if o.get('imm_valid'):
      s = _finite(o.get('imm_s')); d = _finite(o.get('imm_d'))
      vs = _finite(o.get('imm_s_dot'), _finite(o.get('vx'), 0.0))
      acc = _finite(o.get('imm_s_ddot'), 0.0)
      ss = _finite(o.get('imm_s_sigma'), 0.0); ds = _finite(o.get('imm_d_sigma'), 0.0)
      src = 'IMM'
    elif o.get('kf_frenet_valid'):
      s = _finite(o.get('kf_s')); d = _finite(o.get('kf_d'))
      vs = _finite(o.get('kf_s_dot'), _finite(o.get('vx'), 0.0))
      acc = _finite(o.get('kf_s_ddot'), 0.0)
      ss = _finite(o.get('kf_s_sigma'), 0.0); ds = _finite(o.get('kf_d_sigma'), 0.0)
      src = 'KF3'
    else:
      s = _finite(o.get('x')); d = _finite(o.get('road_d'), _finite(o.get('y')))
      vs = _finite(o.get('vx'), 0.0); acc = 0.0
      ss = _finite(o.get('kf_x_sigma'), 0.0); ds = _finite(o.get('kf_y_sigma'), 0.0)
      src = 'RADAR'
    if s is None or d is None:
      return None
    return {'s':s, 'd':d, 's_dot':vs or 0.0, 's_ddot':acc or 0.0,
            's_sigma':max(0.0, ss or 0.0), 'd_sigma':max(0.0, ds or 0.0),
            'source':src, 'lane_index':lane_index_from_d(d), 'lane':lane_name(lane_index_from_d(d))}

  p, src = _find_traj_point(o, t)
  if p is not None:
    s = _finite(p.get('s'), _finite(p.get('x')))
    d = _finite(p.get('d'), _finite(p.get('y')))
    if s is None or d is None:
      return None
    ss = _finite(p.get('s_sigma'), _finite(o.get('imm_s_sigma'), _finite(o.get('kf_s_sigma'), 0.0))) or 0.0
    ds = _finite(p.get('d_sigma'), _finite(o.get('imm_d_sigma'), _finite(o.get('kf_d_sigma'), 0.0))) or 0.0
    return {'s':s, 'd':d,
            's_dot':_finite(p.get('s_dot'), _finite(o.get('imm_s_dot'), _finite(o.get('kf_s_dot'), _finite(o.get('vx'),0.0)))) or 0.0,
            's_ddot':_finite(o.get('imm_s_ddot'), _finite(o.get('kf_s_ddot'),0.0)) or 0.0,
            's_sigma':max(0.0,ss), 'd_sigma':max(0.0,ds), 'source':src,
            'lane_index':int(p.get('lane_index')) if p.get('lane_index') is not None else lane_index_from_d(d),
            'lane':p.get('lane') or lane_name(lane_index_from_d(d))}

  s0 = _finite(o.get('kf_s'), _finite(o.get('kf_x'), _finite(o.get('x'))))
  d0 = _finite(o.get('kf_d'), _finite(o.get('kf_y'), _finite(o.get('road_d'), _finite(o.get('y')))))
  if s0 is None or d0 is None:
    return None
  v = _finite(o.get('kf_s_dot'), _finite(o.get('kf_vx'), _finite(o.get('vx'),0.0))) or 0.0
  a = _finite(o.get('kf_s_ddot'), 0.0) or 0.0
  dd = _finite(o.get('kf_d_dot'), _finite(o.get('kf_vy'),0.0)) or 0.0
  s = s0 + v*t + 0.5*a*t*t
  d = d0 + dd*t
  return {'s':s,'d':d,'s_dot':v+a*t,'s_ddot':a,
          's_sigma':max(0.0,_finite(o.get('kf_s_sigma'),0.0) or 0.0),
          'd_sigma':max(0.0,_finite(o.get('kf_d_sigma'),0.0) or 0.0),
          'source':'KF_CV_FALLBACK','lane_index':lane_index_from_d(d),'lane':lane_name(lane_index_from_d(d))}


def _target_bounds(target_idx: int) -> tuple[float,float,float]:
  c = float(target_idx) * LANE_W_M
  return c - LANE_W_M/2.0, c + LANE_W_M/2.0, c


def _center_in_target(st: dict, target_idx: int) -> bool:
  lo,hi,_ = _target_bounds(target_idx)
  d=float(st['d'])
  return lo <= d <= hi


def _overlaps_target(st: dict, target_idx: int, conservative: bool=False) -> bool:
  lo,hi,_ = _target_bounds(target_idx)
  sigma = min(MAX_LAT_SIGMA_MARGIN_M, max(0.0,float(st.get('d_sigma',0.0)))) if conservative else 0.0
  half = OBJECT_HALF_WIDTH_M + sigma
  return (float(st['d']) + half) >= lo and (float(st['d']) - half) <= hi


def _ego_d_at(t: float, target_idx: int) -> float:
  u = min(1.0, max(0.0, float(t)/LANE_CHANGE_DURATION_S))
  smooth = u*u*(3.0-2.0*u)
  return float(target_idx) * LANE_W_M * smooth


def _ego_overlaps_target(t: float, target_idx: int) -> bool:
  lo,hi,_ = _target_bounds(target_idx)
  d = _ego_d_at(t,target_idx)
  return (d+EGO_HALF_WIDTH_M)>=lo and (d-EGO_HALF_WIDTH_M)<=hi


def _clearance(st: dict) -> float:
  s = abs(float(st['s']))
  sig = min(MAX_LONG_SIGMA_MARGIN_M, max(0.0,float(st.get('s_sigma',0.0))))
  return max(0.0, s - OBJECT_LONG_MARGIN_M - sig)


def _scenario_clearance(s: float, s_dot: float, a_rel: float, t: float) -> tuple[float,float]:
  sp=float(s)+float(s_dot)*float(t)+0.5*float(a_rel)*float(t)*float(t)
  return sp, max(0.0, abs(sp)-OBJECT_LONG_MARGIN_M)


def _ttc_ca(s: float, v: float, a: float) -> float | None:
  s=float(s); v=float(v); a=float(a)
  if abs(a) < 1e-5:
    if abs(v) < 1e-5:
      return None
    t=-s/v
    return t if 0.0 < t <= MAX_TTC_S else None
  disc=v*v-2.0*a*s
  if disc < 0.0:
    return None
  root=math.sqrt(max(0.0,disc))
  vals=[(-v-root)/a,(-v+root)/a]
  vals=[t for t in vals if 0.0 < t <= MAX_TTC_S]
  return min(vals) if vals else None


def _obj_summary(o: dict, st: dict, current_core: bool, current_overlap: bool) -> dict:
  return {
    'key':_key(o),'s':round(float(st['s']),3),'d':round(float(st['d']),3),
    'clearance_m':round(_clearance(st),3),'lane':st.get('lane'),'lane_index':st.get('lane_index'),
    'prediction_source':st.get('source'),'current_target_core':bool(current_core),
    'current_target_overlap':bool(current_overlap),
    'imm_prob_maneuver':o.get('imm_prob_maneuver'),'imm_dominant_model':o.get('imm_dominant_model'),
    'canonical_age_s':o.get('canonical_track_duration_s'),
  }


class FutureGapEvaluator:
  def __init__(self):
    self.api_version=FUTURE_GAP_API_VERSION
    self.incoming_hist: dict[tuple[int,str], dict] = {}

  def _stabilize_incoming(self, side: dict, target_idx: int, now_ns: int) -> dict:
    now_ns=int(now_ns)
    present=set()
    stable=[]
    for item in side.get('incoming') or []:
      key=str(item.get('key') or '')
      if not key:
        continue
      hk=(int(target_idx),key); present.add(hk)
      h=self.incoming_hist.get(hk)
      if h is None or now_ns-int(h.get('last_ns',0)) > int(INCOMING_FORGET_S*1e9):
        h={'since_ns':now_ns,'last_ns':now_ns,'count':1}
      else:
        h['last_ns']=now_ns; h['count']=int(h.get('count',0))+1
      self.incoming_hist[hk]=h
      age_s=max(0.0,(now_ns-int(h['since_ns']))/1e9)
      item['temporal_age_s']=round(age_s,3)
      item['temporal_count']=int(h['count'])
      item['temporal_confirmed']=bool(age_s>=INCOMING_STABLE_S or h['count']>=2)
      if item['temporal_confirmed']:
        stable.append(dict(item))
    stale=[hk for hk,h in self.incoming_hist.items()
           if now_ns-int(h.get('last_ns',0)) > int(INCOMING_FORGET_S*1e9)]
    for hk in stale:
      self.incoming_hist.pop(hk,None)
    side['stable_incoming']=stable[:12]
    side['stable_incoming_count']=len(stable)
    return side

  @staticmethod
  def _decision(side: dict) -> dict:
    reasons=[]
    level='SAFE_SHADOW'
    front=side.get('min_front_clearance_during_ego_overlap_m')
    rear=side.get('min_rear_clearance_during_ego_overlap_m')
    boundary=side.get('min_boundary_clearance_during_ego_overlap_m')
    ft=side.get('current_front_ttc_ca_s'); rt=side.get('current_rear_ttc_ca_s')
    stable=side.get('stable_incoming') or []
    possible=side.get('possible_incoming') or []
    sc=side.get('scenarios') or {}
    fb=(sc.get('front_target_brake') or {}).get('min_clearance_m')
    rb=(sc.get('rear_ego_brake') or {}).get('min_clearance_m')

    def le(v,thr):
      return v is not None and float(v) <= float(thr)
    if le(front,DEC_BLOCK_CLEARANCE_M):
      reasons.append(f'front_gap<={DEC_BLOCK_CLEARANCE_M:.0f}m')
    if le(rear,DEC_BLOCK_CLEARANCE_M):
      reasons.append(f'rear_gap<={DEC_BLOCK_CLEARANCE_M:.0f}m')
    if le(boundary,DEC_BLOCK_BOUNDARY_M):
      reasons.append(f'boundary<={DEC_BLOCK_BOUNDARY_M:.0f}m')
    if le(ft,DEC_BLOCK_TTC_S) or le(rt,DEC_BLOCK_TTC_S):
      reasons.append(f'TTC<={DEC_BLOCK_TTC_S:.0f}s')
    if le(fb,3.0):
      reasons.append('front_brake_scenario<=3m')
    if le(rb,3.0):
      reasons.append('ego_brake_rear<=3m')
    for x in stable:
      if float(x.get('entry_eta_s',99))<=DEC_BLOCK_INCOMING_ETA_S and abs(float(x.get('entry_s_m',999)))<=DEC_BLOCK_INCOMING_ABS_S_M:
        reasons.append('stable_incoming_near')
        break
    if reasons:
      level='BLOCKED_SHADOW'
    else:
      caution=[]
      if le(front,DEC_CAUTION_CLEARANCE_M): caution.append(f'front_gap<={DEC_CAUTION_CLEARANCE_M:.0f}m')
      if le(rear,DEC_CAUTION_CLEARANCE_M): caution.append(f'rear_gap<={DEC_CAUTION_CLEARANCE_M:.0f}m')
      if le(boundary,DEC_CAUTION_BOUNDARY_M): caution.append(f'boundary<={DEC_CAUTION_BOUNDARY_M:.0f}m')
      if le(ft,DEC_CAUTION_TTC_S) or le(rt,DEC_CAUTION_TTC_S): caution.append(f'TTC<={DEC_CAUTION_TTC_S:.0f}s')
      if le(fb,8.0): caution.append('front_brake_scenario<=8m')
      if le(rb,8.0): caution.append('ego_brake_rear<=8m')
      for x in stable:
        if float(x.get('entry_eta_s',99))<=DEC_CAUTION_INCOMING_ETA_S and abs(float(x.get('entry_s_m',999)))<=DEC_CAUTION_INCOMING_ABS_S_M:
          caution.append('stable_incoming')
          break
      if not caution:
        for x in possible:
          if float(x.get('entry_eta_s',99))<=1.5 and abs(float(x.get('entry_s_m',999)))<=25.0:
            caution.append('possible_incoming_near')
            break
      if caution:
        level='CAUTION_SHADOW'; reasons=caution
    return {'state':level,'reasons':reasons[:6],
            'policy':'DEC1 diagnostic only; never connected to planner/control'}

  def _side(self, objects: list[dict], target_idx: int, a_ego: float) -> dict:
    states_by_key: dict[str,dict[float,dict]]={}
    current_core: dict[str,bool]={}
    current_overlap: dict[str,bool]={}
    origin_lane: dict[str,str|None]={}
    obj_by_key={}
    for o in objects:
      k=_key(o)
      if not k:
        continue
      st0=_state_at(o,0.0)
      if st0 is None:
        continue
      obj_by_key[k]=o
      states_by_key[k]={0.0:st0}
      current_core[k]=_center_in_target(st0,target_idx)
      current_overlap[k]=_overlaps_target(st0,target_idx,False)
      origin_lane[k]=st0.get('lane')
      for h in HORIZONS_S[1:]:
        st=_state_at(o,h)
        if st is not None:
          states_by_key[k][h]=st

    confirmed_incoming=[]
    possible_incoming=[]
    relevant_origins = ('left2','ego') if target_idx > 0 else ('right2','ego')
    _,_,target_center=_target_bounds(target_idx)
    for k,o in obj_by_key.items():
      if current_core.get(k) or origin_lane.get(k) not in relevant_origins:
        continue
      states=states_by_key[k]
      center_hits=[]
      overlap_hits=[]
      for h in HORIZONS_S[1:]:
        st=states.get(h)
        if st is None:
          continue
        if _center_in_target(st,target_idx):
          center_hits.append((h,st))
        if _overlaps_target(st,target_idx,False):
          overlap_hits.append((h,st))
      if not center_hits and not overlap_hits:
        continue
      first_h,first_st=(center_hits[0] if center_hits else overlap_hits[0])
      persists=False
      if center_hits:
        first_i=HORIZONS_S.index(first_h)
        for h2 in HORIZONS_S[first_i+1:]:
          st2=states.get(h2)
          if st2 is not None and _center_in_target(st2,target_idx):
            persists=True
            break
      d0=float(states[0.0]['d']); d1=float(first_st['d'])
      toward_prediction=abs(d1-target_center) <= max(0.0,abs(d0-target_center)-0.15)
      pman=float(o.get('imm_prob_maneuver') or 0.0)
      man_candidate=bool(o.get('imm_maneuver_candidate'))
      if origin_lane.get(k)=='ego':
        confirmed=bool(center_hits and persists and toward_prediction and (pman>=INCOMING_EGO_MIN_MANEUVER_PROB or man_candidate))
      else:
        confirmed=bool(center_hits and persists and toward_prediction)
      item={
        'key':k,'origin_lane':origin_lane.get(k),'target_lane':lane_name(target_idx),
        'entry_eta_s':float(first_h),'entry_s_m':round(float(first_st['s']),3),'entry_d_m':round(float(first_st['d']),3),
        'centre_entry':bool(center_hits),'persists':bool(persists),'toward_target':bool(toward_prediction),
        'imm_prob_maneuver':o.get('imm_prob_maneuver'),'imm_dominant_model':o.get('imm_dominant_model'),
        'imm_ttlc_s':o.get('imm_ttlc_s'),'prediction_source':first_st.get('source')
      }
      (confirmed_incoming if confirmed else possible_incoming).append(item)

    confirmed_incoming.sort(key=lambda z:(z['entry_eta_s'],abs(z['entry_s_m'])))
    possible_incoming.sort(key=lambda z:(z['entry_eta_s'],abs(z['entry_s_m'])))

    horizons=[]
    min_front=None; min_rear=None; min_abs=None; min_boundary=None
    nearest_front_key=None; nearest_rear_key=None
    for h in HORIZONS_S:
      core=[]; boundary=[]; uncertain_only=[]
      for k,o in obj_by_key.items():
        st=states_by_key.get(k,{}).get(h)
        if st is None:
          continue
        if _center_in_target(st,target_idx):
          core.append((o,st,current_core.get(k,False),current_overlap.get(k,False)))
        elif _overlaps_target(st,target_idx,False):
          boundary.append((o,st,current_core.get(k,False),current_overlap.get(k,False)))
        elif _overlaps_target(st,target_idx,True):
          uncertain_only.append((o,st,current_core.get(k,False),current_overlap.get(k,False)))
      front=[x for x in core if float(x[1]['s'])>=0.0]
      rear=[x for x in core if float(x[1]['s'])<0.0]
      front.sort(key=lambda x:float(x[1]['s']))
      rear.sort(key=lambda x:abs(float(x[1]['s'])))
      f=front[0] if front else None; r=rear[0] if rear else None
      fsum=_obj_summary(f[0],f[1],f[2],f[3]) if f else None
      rsum=_obj_summary(r[0],r[1],r[2],r[3]) if r else None

      bfront=[x for x in boundary if float(x[1]['s'])>=0.0]
      brear=[x for x in boundary if float(x[1]['s'])<0.0]
      bfront.sort(key=lambda x:float(x[1]['s'])); brear.sort(key=lambda x:abs(float(x[1]['s'])))
      bf=_obj_summary(*bfront[0]) if bfront else None
      br=_obj_summary(*brear[0]) if brear else None

      ego_overlap=_ego_overlaps_target(h,target_idx)
      row={
        't':h,'ego_target_d_m':round(_ego_d_at(h,target_idx),3),'ego_target_overlap':ego_overlap,
        'core_occupant_count':len(core),'boundary_overlap_count':len(boundary),
        'uncertain_only_count':len(uncertain_only),'front':fsum,'rear':rsum,
        'boundary_front':bf,'boundary_rear':br,
      }
      horizons.append(row)
      if ego_overlap:
        if fsum is not None and (min_front is None or fsum['clearance_m']<min_front):
          min_front=fsum['clearance_m']; nearest_front_key=fsum['key']
        if rsum is not None and (min_rear is None or rsum['clearance_m']<min_rear):
          min_rear=rsum['clearance_m']; nearest_rear_key=rsum['key']
        vals=[abs(float(x[1]['s'])) for x in core]
        if vals:
          v=min(vals); min_abs=v if min_abs is None else min(min_abs,v)
        bvals=[_clearance(x[1]) for x in boundary]
        if bvals:
          bv=min(bvals); min_boundary=bv if min_boundary is None else min(min_boundary,bv)

    current=horizons[0]
    ft=rt=None
    current_front_obj=current_rear_obj=None
    if current['front'] is not None:
      current_front_obj=obj_by_key.get(current['front']['key'])
      st=states_by_key[current['front']['key']][0.0]
      ft=_ttc_ca(st['s'],st['s_dot'],st['s_ddot'])
    if current['rear'] is not None:
      current_rear_obj=obj_by_key.get(current['rear']['key'])
      st=states_by_key[current['rear']['key']][0.0]
      rt=_ttc_ca(st['s'],st['s_dot'],st['s_ddot'])

    scenarios={'target_brake_assumption_mps2':TARGET_BRAKE_ASSUMPTION_MPS2,
               'ego_brake_assumption_mps2':EGO_BRAKE_ASSUMPTION_MPS2,
               'front_target_brake':None,'rear_ego_brake':None}
    if current_front_obj is not None:
      st=states_by_key[_key(current_front_obj)][0.0]
      a_rel=TARGET_BRAKE_ASSUMPTION_MPS2-float(a_ego)
      series=[]
      for h in HORIZONS_S:
        sp,clr=_scenario_clearance(st['s'],st['s_dot'],a_rel,h)
        series.append({'t':h,'s_m':round(sp,3),'clearance_m':round(clr,3)})
      scenarios['front_target_brake']={'key':_key(current_front_obj),'relative_accel_mps2':round(a_rel,3),'horizons':series,
                                       'min_clearance_m':round(min(x['clearance_m'] for x in series),3)}
    if current_rear_obj is not None:
      st=states_by_key[_key(current_rear_obj)][0.0]
      rear_abs_accel=float(st.get('s_ddot',0.0))+float(a_ego)
      a_rel=rear_abs_accel-EGO_BRAKE_ASSUMPTION_MPS2
      series=[]
      for h in HORIZONS_S:
        sp,clr=_scenario_clearance(st['s'],st['s_dot'],a_rel,h)
        series.append({'t':h,'s_m':round(sp,3),'clearance_m':round(clr,3)})
      scenarios['rear_ego_brake']={'key':_key(current_rear_obj),'estimated_rear_abs_accel_mps2':round(rear_abs_accel,3),
                                   'relative_accel_mps2':round(a_rel,3),'horizons':series,
                                   'min_clearance_m':round(min(x['clearance_m'] for x in series),3)}

    return {
      'target_lane':lane_name(target_idx),'target_lane_index':target_idx,
      'horizons':horizons,
      'incoming':confirmed_incoming[:12],'incoming_count':len(confirmed_incoming),
      'possible_incoming':possible_incoming[:12],'possible_incoming_count':len(possible_incoming),
      'min_front_clearance_during_ego_overlap_m':None if min_front is None else round(float(min_front),3),
      'min_rear_clearance_during_ego_overlap_m':None if min_rear is None else round(float(min_rear),3),
      'min_boundary_clearance_during_ego_overlap_m':None if min_boundary is None else round(float(min_boundary),3),
      'min_abs_separation_during_ego_overlap_m':None if min_abs is None else round(float(min_abs),3),
      'min_front_key':nearest_front_key,'min_rear_key':nearest_rear_key,
      'current_front_ttc_ca_s':None if ft is None else round(float(ft),3),
      'current_rear_ttc_ca_s':None if rt is None else round(float(rt),3),
      'scenarios':scenarios,
    }

  def update(self, objects: list[dict], v_ego: float=0.0, a_ego: float=0.0, left_blinker: bool=False, right_blinker: bool=False, now_ns: int=0) -> dict:
    if not now_ns:
      import time
      now_ns=time.monotonic_ns()
    left=self._stabilize_incoming(self._side(objects,+1,a_ego),+1,now_ns)
    right=self._stabilize_incoming(self._side(objects,-1,a_ego),-1,now_ns)
    left['decision']=self._decision(left); right['decision']=self._decision(right)
    active='left' if left_blinker and not right_blinker else ('right' if right_blinker and not left_blinker else None)
    return {
      'api_version':FUTURE_GAP_API_VERSION,
      'mode':'SHADOW_GEOMETRY_FG3_DEC1',
      'decision_enabled':True,
      'safe_caution_blocked_enabled':True,
      'decision_shadow_only':True,
      'active_target':active,
      'horizons_s':list(HORIZONS_S),
      'lane_change_duration_s':LANE_CHANGE_DURATION_S,
      'vehicle_long_margin_m':OBJECT_LONG_MARGIN_M,
      'vehicle_half_width_m':OBJECT_HALF_WIDTH_M,
      'ego':{'v_ego_mps':round(float(v_ego),3),'a_ego_mps2':round(float(a_ego),3),
             'left_blinker':bool(left_blinker),'right_blinker':bool(right_blinker)},
      'left':left,'right':right,
      'stats':{
        'visible_objects':len(objects),
        'left_incoming_confirmed':left['incoming_count'],'right_incoming_confirmed':right['incoming_count'],
        'left_incoming_stable':left['stable_incoming_count'],'right_incoming_stable':right['stable_incoming_count'],
        'left_incoming_possible':left['possible_incoming_count'],'right_incoming_possible':right['possible_incoming_count'],
        'left_current_core':left['horizons'][0]['core_occupant_count'],
        'right_current_core':right['horizons'][0]['core_occupant_count'],
        'left_current_boundary':left['horizons'][0]['boundary_overlap_count'],
        'right_current_boundary':right['horizons'][0]['boundary_overlap_count'],
      },
      'note':'FG3 + DEC1 shadow only: temporal incoming stability + core/boundary gaps + braking what-if + SAFE/CAUTION/BLOCKED diagnostic; no planner/CAN control.',
    }

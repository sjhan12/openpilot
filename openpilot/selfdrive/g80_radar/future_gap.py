#!/usr/bin/env python3
"""V33 shadow Future Gap + Target-Lane Occupancy evaluator.

This module consumes the authoritative Canonical360 objects after KF3/IMM2 and
computes *diagnostic* left/right target-lane geometry for NOW/0.5/1/2/3 s.
It does not output SAFE/CAUTION/BLOCKED and is not connected to planner/control.

Design goals:
- use IMM trajectory when available, KF3 trajectory otherwise;
- retain L2->L1 / R2->R1 incoming vehicles by testing vehicle footprint overlap
  with the target lane, rather than only looking at integer lane labels;
- expose raw longitudinal separation and a conservative clearance estimate;
- expose CA TTC estimates for front/rear closing objects;
- model a hypothetical smooth 3 s ego lane-change trajectory for *both* sides
  only to define when ego begins to occupy the target lane.
"""
from __future__ import annotations

import math
from typing import Any

from openpilot.selfdrive.g80_radar.road_geometry import LANE_W_M, lane_index_from_d, lane_name

FUTURE_GAP_API_VERSION = 1
HORIZONS_S = (0.0, 0.5, 1.0, 2.0, 3.0)
LANE_CHANGE_DURATION_S = 3.0
EGO_HALF_WIDTH_M = 1.05
OBJECT_HALF_WIDTH_M = 1.05
# Decoded x/s is empirically calibrated and is not a rigorously defined object
# centre.  Therefore this is deliberately called a conservative *margin*, not
# a bumper-to-bumper geometry correction.
OBJECT_LONG_MARGIN_M = 2.4
MAX_LAT_SIGMA_MARGIN_M = 1.5
MAX_LONG_SIGMA_MARGIN_M = 5.0
MAX_TTC_S = 20.0


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
    pts = o.get('imm_trajectory') or []
    for p in pts:
      if abs(float(p.get('t', -999.0)) - t) < 1e-6:
        return p, 'IMM'
  if o.get('kalman_valid'):
    pts = o.get('kalman_trajectory') or []
    for p in pts:
      if abs(float(p.get('t', -999.0)) - t) < 1e-6:
        return p, 'KF3'
  return None, 'CV_FALLBACK'


def _state_at(o: dict, t: float) -> dict | None:
  """Return local relative [s,d] state for a horizon."""
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

  # Fallback keeps all canonical tracks usable even if they are outside IMM ROI.
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


def _occupies_target(st: dict, target_idx: int, conservative: bool=False) -> bool:
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


def _ttc_ca(s: float, v: float, a: float) -> float | None:
  """Smallest positive solution to s + v*t + .5*a*t^2 = 0."""
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


def _obj_summary(o: dict, st: dict, target_idx: int, current_occ: bool) -> dict:
  return {
    'key':_key(o),'s':round(float(st['s']),3),'d':round(float(st['d']),3),
    'clearance_m':round(_clearance(st),3),'lane':st.get('lane'),'lane_index':st.get('lane_index'),
    'prediction_source':st.get('source'),'current_target_occupant':bool(current_occ),
    'imm_prob_maneuver':o.get('imm_prob_maneuver'),'imm_dominant_model':o.get('imm_dominant_model'),
    'canonical_age_s':o.get('canonical_track_duration_s'),
  }


class FutureGapEvaluator:
  def __init__(self):
    self.api_version=FUTURE_GAP_API_VERSION

  def _side(self, objects: list[dict], target_idx: int) -> dict:
    states_by_key: dict[str,dict[float,dict]]={}
    current_occ: dict[str,bool]={}
    origin_lane: dict[str,str|None]={}
    for o in objects:
      k=_key(o)
      if not k:
        continue
      st0=_state_at(o,0.0)
      if st0 is None:
        continue
      states_by_key[k]={0.0:st0}; current_occ[k]=_occupies_target(st0,target_idx,False); origin_lane[k]=st0.get('lane')
      for h in HORIZONS_S[1:]:
        st=_state_at(o,h)
        if st is not None:
          states_by_key[k][h]=st

    incoming=[]
    relevant_origins = ('left2','ego') if target_idx > 0 else ('right2','ego')
    for o in objects:
      k=_key(o)
      if k not in states_by_key or current_occ.get(k) or origin_lane.get(k) not in relevant_origins:
        continue
      eta=None; entry_state=None
      for h in HORIZONS_S[1:]:
        st=states_by_key[k].get(h)
        if st is not None and _occupies_target(st,target_idx,False):
          eta=h; entry_state=st; break
      if eta is not None:
        incoming.append({
          'key':k,'origin_lane':origin_lane.get(k),'target_lane':lane_name(target_idx),'entry_eta_s':eta,
          'entry_s_m':round(float(entry_state['s']),3),'entry_d_m':round(float(entry_state['d']),3),
          'imm_prob_maneuver':o.get('imm_prob_maneuver'),'imm_dominant_model':o.get('imm_dominant_model'),
          'imm_ttlc_s':o.get('imm_ttlc_s'),'prediction_source':entry_state.get('source')
        })
    incoming.sort(key=lambda z:(z['entry_eta_s'],abs(z['entry_s_m'])))

    horizons=[]
    min_front=None; min_rear=None; min_abs=None
    nearest_front_key=None; nearest_rear_key=None
    for h in HORIZONS_S:
      occ=[]; uncertain_only=[]
      for o in objects:
        k=_key(o); st=states_by_key.get(k,{}).get(h)
        if st is None:
          continue
        if _occupies_target(st,target_idx,False):
          occ.append((o,st,current_occ.get(k,False)))
        elif _occupies_target(st,target_idx,True):
          uncertain_only.append((o,st,current_occ.get(k,False)))
      front=[x for x in occ if float(x[1]['s'])>=0.0]
      rear=[x for x in occ if float(x[1]['s'])<0.0]
      front.sort(key=lambda x:float(x[1]['s']))
      rear.sort(key=lambda x:abs(float(x[1]['s'])))
      f=front[0] if front else None; r=rear[0] if rear else None
      fsum=_obj_summary(f[0],f[1],target_idx,f[2]) if f else None
      rsum=_obj_summary(r[0],r[1],target_idx,r[2]) if r else None
      ego_overlap=_ego_overlaps_target(h,target_idx)
      row={
        't':h,'ego_target_d_m':round(_ego_d_at(h,target_idx),3),'ego_target_overlap':ego_overlap,
        'occupant_count':len(occ),'uncertain_only_count':len(uncertain_only),'front':fsum,'rear':rsum,
      }
      horizons.append(row)
      if ego_overlap:
        if fsum is not None and (min_front is None or fsum['clearance_m']<min_front):
          min_front=fsum['clearance_m']; nearest_front_key=fsum['key']
        if rsum is not None and (min_rear is None or rsum['clearance_m']<min_rear):
          min_rear=rsum['clearance_m']; nearest_rear_key=rsum['key']
        vals=[abs(float(x[1]['s'])) for x in occ]
        if vals:
          v=min(vals); min_abs=v if min_abs is None else min(min_abs,v)

    # Current closest target-lane TTC diagnostics, acceleration-aware.
    current=horizons[0]
    ft=rt=None
    if current['front'] is not None:
      o=next((x for x in objects if _key(x)==current['front']['key']),None)
      st=states_by_key[current['front']['key']][0.0]
      ft=_ttc_ca(st['s'],st['s_dot'],st['s_ddot'])
    if current['rear'] is not None:
      st=states_by_key[current['rear']['key']][0.0]
      rt=_ttc_ca(st['s'],st['s_dot'],st['s_ddot'])

    return {
      'target_lane':lane_name(target_idx),'target_lane_index':target_idx,
      'horizons':horizons,'incoming':incoming[:12],'incoming_count':len(incoming),
      'min_front_clearance_during_ego_overlap_m':None if min_front is None else round(float(min_front),3),
      'min_rear_clearance_during_ego_overlap_m':None if min_rear is None else round(float(min_rear),3),
      'min_abs_separation_during_ego_overlap_m':None if min_abs is None else round(float(min_abs),3),
      'min_front_key':nearest_front_key,'min_rear_key':nearest_rear_key,
      'current_front_ttc_ca_s':None if ft is None else round(float(ft),3),
      'current_rear_ttc_ca_s':None if rt is None else round(float(rt),3),
    }

  def update(self, objects: list[dict], v_ego: float=0.0, a_ego: float=0.0, left_blinker: bool=False, right_blinker: bool=False) -> dict:
    left=self._side(objects,+1); right=self._side(objects,-1)
    return {
      'api_version':FUTURE_GAP_API_VERSION,
      'mode':'SHADOW_GEOMETRY_ONLY',
      'decision_enabled':False,
      'safe_caution_blocked_enabled':False,
      'horizons_s':list(HORIZONS_S),
      'lane_change_duration_s':LANE_CHANGE_DURATION_S,
      'vehicle_long_margin_m':OBJECT_LONG_MARGIN_M,
      'vehicle_half_width_m':OBJECT_HALF_WIDTH_M,
      'ego':{'v_ego_mps':round(float(v_ego),3),'a_ego_mps2':round(float(a_ego),3),
             'left_blinker':bool(left_blinker),'right_blinker':bool(right_blinker)},
      'left':left,'right':right,
      'stats':{
        'visible_objects':len(objects),
        'left_incoming':left['incoming_count'],'right_incoming':right['incoming_count'],
        'left_current_occupants':left['horizons'][0]['occupant_count'],
        'right_current_occupants':right['horizons'][0]['occupant_count'],
        'left_current_uncertain_only':left['horizons'][0]['uncertain_only_count'],
        'right_current_uncertain_only':right['horizons'][0]['uncertain_only_count'],
      },
      'note':'Future-gap/target-lane occupancy only; no SAFE/CAUTION/BLOCKED and no control output.',
    }

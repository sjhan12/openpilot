#!/usr/bin/env python3
"""V38R1 shadow Future Gap + Target-Lane Occupancy evaluator.

Consumes Canonical360 vehicles after KF4/IMM3 and computes diagnostic target-lane
geometry for NOW/0.5/1/2/3 s.  V37 keeps FG4 geometry and adds DEC3 maneuver-context handling:
- distinguish likely intersection turns from lane changes using road curvature, speed and steering angle;
- latch the pre-commit lane-change assessment briefly after steering commitment;
- suppress coordinate-recenter false DANGER during the commit/rebase window while retaining a hard TTC override;
- expose both-side preview decisions continuously for arrow HUD rendering:
- core occupants: object centre is inside the target lane;
- boundary overlaps: only the object footprint overlaps the target lane;
- confirmed incoming: predicted centre enters and persists in the target lane;
- possible incoming: footprint/one-horizon entry only.

It also exposes diagnostic what-if braking scenarios and shadow-only
SAFE_SHADOW/CAUTION_SHADOW/BLOCKED_SHADOW. Driver HUD maps lane-change states to SAFE/CHECK?/DANGER and intersection turns to TURN.
These are diagnostic labels only and are not connected to planner/control.
"""
from __future__ import annotations

import math

from openpilot.selfdrive.g80_radar.road_geometry import LANE_W_M, lane_index_from_d, lane_name, adjacent_lane_availability

FUTURE_GAP_API_VERSION = 6
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

DISPLAY_DANGER_RELEASE_S = 0.35
DISPLAY_SAFE_ENTRY_S = 0.60

# V37 maneuver-context / lane-change commit handling.
TURN_CURVE_MATCH_MAX_SPEED_MPS = 8.5      # ~30.6 km/h
TURN_STEERING_MIN_DEG = 35.0
TURN_STEERING_MAX_SPEED_MPS = 14.0        # allow a brisk intersection turn to be classified
LANE_CHANGE_COMMIT_MIN_SPEED_MPS = 5.0
LANE_CHANGE_COMMIT_STEER_DEG = 4.0
LANE_CHANGE_COMMIT_MIN_BLINKER_S = 0.15
LANE_CHANGE_DECISION_HOLD_S = 1.50
LANE_CHANGE_REBASE_S = 0.85
LANE_CHANGE_SESSION_TIMEOUT_S = 4.5
HARD_OVERRIDE_CLEARANCE_M = 1.5
HARD_OVERRIDE_TTC_S = 1.5

# DEC3 diagnostic thresholds. These are shadow-only engineering gates, not
# control limits and not connected to planner/CAN. V37 preserves actual geometry/TTC
# hazards from hypothetical braking scenarios.
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
    self.display_hist = {+1:{'state':'CAUTION_SHADOW','raw':None,'raw_since_ns':0}, -1:{'state':'CAUTION_SHADOW','raw':None,'raw_since_ns':0}}
    self.intent_hist = {
      'side': None, 'blinker_since_ns': 0, 'committed': False, 'commit_ns': 0,
      'latched_decision': None, 'precommit_decision': None, 'last_active_ns': 0, 'last_context': 'STANDBY'
    }

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
    def le(v,thr): return v is not None and float(v) <= float(thr)
    if le(front,DEC_BLOCK_CLEARANCE_M): reasons.append(f'front_gap<={DEC_BLOCK_CLEARANCE_M:.0f}m')
    if le(rear,DEC_BLOCK_CLEARANCE_M): reasons.append(f'rear_gap<={DEC_BLOCK_CLEARANCE_M:.0f}m')
    if le(boundary,DEC_BLOCK_BOUNDARY_M): reasons.append(f'boundary<={DEC_BLOCK_BOUNDARY_M:.0f}m')
    if le(ft,DEC_BLOCK_TTC_S) or le(rt,DEC_BLOCK_TTC_S): reasons.append(f'TTC<={DEC_BLOCK_TTC_S:.0f}s')
    for x in stable:
      if float(x.get('entry_eta_s',99))<=DEC_BLOCK_INCOMING_ETA_S and abs(float(x.get('entry_s_m',999)))<=DEC_BLOCK_INCOMING_ABS_S_M:
        reasons.append('stable_incoming_near'); break
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
          caution.append('stable_incoming'); break
      if not caution:
        for x in possible:
          if float(x.get('entry_eta_s',99))<=1.5 and abs(float(x.get('entry_s_m',999)))<=25.0:
            caution.append('possible_incoming_near'); break
      if caution: level='CAUTION_SHADOW'; reasons=caution
    return {'state':level,'reasons':reasons[:6],'policy':'DEC3 base shadow; what-if braking alone is advisory CHECK only'}

  @staticmethod
  def _apply_road_lane_gate(raw: dict, availability: dict | None) -> dict:
    """Never report SAFE from empty radar space alone.

    Vehicle hazards keep priority.  The road gate only converts an otherwise
    SAFE decision to NO LANE / CHECK ROAD when target-lane geometry is absent
    or not confirmed by the fresh C4 road model.
    """
    out = dict(raw or {})
    if str(out.get('state')) != 'SAFE_SHADOW':
      return out
    av = availability or {}
    status = str(av.get('status') or 'UNCERTAIN').upper()
    if status == 'CONFIRMED':
      return out
    out['state'] = 'CAUTION_SHADOW'
    if status == 'ABSENT':
      out['label_override'] = 'NO LANE'
      reason = 'target_lane_absent'
    else:
      out['label_override'] = 'CHECK ROAD'
      reason = 'target_lane_unconfirmed'
    out['reasons'] = [reason] + list(out.get('reasons') or [])[:5]
    out['road_gate'] = status
    return out

  def _stabilize_decision(self, side: dict, target_idx: int, now_ns: int) -> dict:
    raw=dict(side.get('decision_raw') or self._decision(side))
    h=self.display_hist[int(target_idx)]
    raw_state=str(raw.get('state') or 'CAUTION_SHADOW')
    if h.get('raw') != raw_state:
      h['raw']=raw_state; h['raw_since_ns']=int(now_ns)
    age_s=max(0.0,(int(now_ns)-int(h.get('raw_since_ns') or now_ns))/1e9)
    cur=str(h.get('state') or 'CAUTION_SHADOW')
    if raw_state == 'BLOCKED_SHADOW': cur='BLOCKED_SHADOW'
    elif raw_state == 'CAUTION_SHADOW':
      if cur == 'BLOCKED_SHADOW':
        if age_s >= DISPLAY_DANGER_RELEASE_S: cur='CAUTION_SHADOW'
      else: cur='CAUTION_SHADOW'
    else:
      if cur == 'BLOCKED_SHADOW':
        if age_s >= DISPLAY_DANGER_RELEASE_S: cur='CAUTION_SHADOW'
      elif cur == 'CAUTION_SHADOW':
        if age_s >= DISPLAY_SAFE_ENTRY_S: cur='SAFE_SHADOW'
      else: cur='SAFE_SHADOW'
    h['state']=cur; self.display_hist[int(target_idx)]=h
    label='DANGER' if cur=='BLOCKED_SHADOW' else ('CHECK ?' if cur=='CAUTION_SHADOW' else 'SAFE')
    if cur=='CAUTION_SHADOW' and raw_state=='CAUTION_SHADOW' and raw.get('label_override'):
      label=str(raw.get('label_override'))
    reasons=raw.get('reasons',[])[:6] if cur==raw_state else ['display_hysteresis']+raw.get('reasons',[])[:5]
    return {'state':cur,'label':label,'raw_state':raw_state,'raw_reasons':raw.get('reasons',[])[:6],'reasons':reasons,'raw_age_s':round(age_s,3),'policy':'DEC3 display hysteresis; comparison only'}

  @staticmethod
  def _context(active: str | None, v_ego: float, steering_angle_deg: float,
               road_curve_direction: str | None) -> dict:
    if active not in ('left','right'):
      return {'kind':'STANDBY','curve_match':False,'turn_by_curve':False,'turn_by_steer':False}
    curve = str(road_curve_direction or 'UNKNOWN').upper()
    want = 'LEFT' if active == 'left' else 'RIGHT'
    curve_match = curve == want
    steer = abs(float(steering_angle_deg or 0.0))
    speed = max(0.0, float(v_ego or 0.0))
    turn_by_curve = bool(curve_match and speed <= TURN_CURVE_MATCH_MAX_SPEED_MPS)
    turn_by_steer = bool(steer >= TURN_STEERING_MIN_DEG and speed <= TURN_STEERING_MAX_SPEED_MPS)
    kind = 'TURN' if (turn_by_curve or turn_by_steer) else 'LANE_CHANGE'
    return {'kind':kind,'curve_match':curve_match,'turn_by_curve':turn_by_curve,
            'turn_by_steer':turn_by_steer,'curve_direction':curve,'steering_abs_deg':round(steer,2)}

  @staticmethod
  def _hard_override(side: dict) -> bool:
    front=side.get('min_front_clearance_during_ego_overlap_m')
    rear=side.get('min_rear_clearance_during_ego_overlap_m')
    ft=side.get('current_front_ttc_ca_s'); rt=side.get('current_rear_ttc_ca_s')
    def le(v,t): return v is not None and float(v) <= float(t)
    # During lane-change coordinate recentering a zero gap can be synthetic, so
    # require both a very small physical clearance and a very short TTC.
    return bool((le(front,HARD_OVERRIDE_CLEARANCE_M) and le(ft,HARD_OVERRIDE_TTC_S)) or
                (le(rear,HARD_OVERRIDE_CLEARANCE_M) and le(rt,HARD_OVERRIDE_TTC_S)))

  def _apply_intent_state(self, active: str | None, active_side: dict | None,
                          v_ego: float, steering_angle_deg: float,
                          road_curve_direction: str | None, now_ns: int) -> dict:
    base={'active':bool(active_side is not None),'side':active,'label':'STANDBY','state':'STANDBY',
          'raw_state':None,'reasons':[],'front_clearance_m':None,'rear_clearance_m':None,
          'boundary_clearance_m':None,'front_ttc_s':None,'rear_ttc_s':None,'shadow_only':True,
          'maneuver_context':'STANDBY','phase':'STANDBY','committed':False,
          'commit_age_s':None,'hold_remaining_s':None,'steering_angle_deg':round(float(steering_angle_deg or 0.0),2),
          'road_curve_direction':str(road_curve_direction or 'UNKNOWN').upper(),
          'lane_availability':None}
    h=self.intent_hist

    if active_side is None or active not in ('left','right'):
      if h.get('last_active_ns') and int(now_ns)-int(h.get('last_active_ns',0)) > int(0.8e9):
        self.intent_hist={'side':None,'blinker_since_ns':0,'committed':False,'commit_ns':0,
                          'latched_decision':None,'precommit_decision':None,'last_active_ns':0,'last_context':'STANDBY'}
      return base

    if h.get('side') != active or (int(now_ns)-int(h.get('last_active_ns',0) or 0) > int(LANE_CHANGE_SESSION_TIMEOUT_S*1e9)):
      h={'side':active,'blinker_since_ns':int(now_ns),'committed':False,'commit_ns':0,
         'latched_decision':None,'precommit_decision':None,'last_active_ns':int(now_ns),'last_context':'STANDBY'}
      self.intent_hist=h
    else:
      h['last_active_ns']=int(now_ns)

    ctx=self._context(active,v_ego,steering_angle_deg,road_curve_direction)
    h['last_context']=ctx['kind']
    base['maneuver_context']=ctx['kind']
    base['context_detail']=ctx
    base['blinker_age_s']=round(max(0.0,(int(now_ns)-int(h.get('blinker_since_ns',now_ns)))/1e9),3)

    dec=dict(active_side.get('decision') or {})
    base.update({'label':dec.get('label','CHECK ?'),'state':dec.get('state','CAUTION_SHADOW'),
                 'raw_state':dec.get('raw_state'),'reasons':dec.get('reasons',[])[:6],
                 'front_clearance_m':active_side.get('min_front_clearance_during_ego_overlap_m'),
                 'rear_clearance_m':active_side.get('min_rear_clearance_during_ego_overlap_m'),
                 'boundary_clearance_m':active_side.get('min_boundary_clearance_during_ego_overlap_m'),
                 'front_ttc_s':active_side.get('current_front_ttc_ca_s'),
                 'rear_ttc_s':active_side.get('current_rear_ttc_ca_s'),
                 'lane_availability':dict(active_side.get('lane_availability') or {})})

    if ctx['kind']=='TURN':
      # A blinker during a low-speed, same-direction road turn should not be
      # presented as a lane-change permission decision.
      h['committed']=False; h['commit_ns']=0; h['latched_decision']=None; h['precommit_decision']=None
      base.update({'label':'TURN','state':'TURN','phase':'TURNING','committed':False,
                   'reasons':['intersection_turn_context']})
      return base

    # Lane-change context.
    blink_age=max(0.0,(int(now_ns)-int(h.get('blinker_since_ns',now_ns)))/1e9)
    steer=abs(float(steering_angle_deg or 0.0))
    if (not h.get('committed') and float(v_ego)>=LANE_CHANGE_COMMIT_MIN_SPEED_MPS and
        blink_age>=LANE_CHANGE_COMMIT_MIN_BLINKER_S and steer>=LANE_CHANGE_COMMIT_STEER_DEG):
      h['committed']=True; h['commit_ns']=int(now_ns)
      h['latched_decision']=dict(h.get('precommit_decision') or dec)

    if not h.get('committed'):
      h['precommit_decision']=dict(dec)
      base['phase']='PRECHECK'
      return base

    age=max(0.0,(int(now_ns)-int(h.get('commit_ns',now_ns)))/1e9)
    base['committed']=True; base['commit_age_s']=round(age,3)
    lat=dict(h.get('latched_decision') or dec)

    if age < LANE_CHANGE_DECISION_HOLD_S:
      base['phase']='COMMIT_HOLD'
      base['hold_remaining_s']=round(max(0.0,LANE_CHANGE_DECISION_HOLD_S-age),3)
      if self._hard_override(active_side):
        base['reasons']=['commit_hold_hard_ttc_override']+(dec.get('reasons') or [])[:5]
      else:
        base['state']=lat.get('state','CAUTION_SHADOW')
        base['label']=lat.get('label','CHECK ?')
        base['reasons']=['lane_change_commit_hold']+(lat.get('reasons') or [])[:5]
      return base

    if age < LANE_CHANGE_DECISION_HOLD_S + LANE_CHANGE_REBASE_S:
      base['phase']='REBASING'
      base['hold_remaining_s']=0.0
      # Re-centering into the target lane can make the old target-lane coordinate
      # system report zero gap/boundary overlap.  During this short window keep
      # such geometry-only DANGER at CHECK, but do not mask a hard TTC conflict.
      if base.get('state')=='BLOCKED_SHADOW' and not self._hard_override(active_side):
        rr=set(base.get('reasons') or [])
        benign_prefixes=('front_gap<=','rear_gap<=','boundary<=')
        benign_exact={'display_hysteresis','possible_incoming_near','front_brake_scenario<=8m','ego_brake_rear<=8m'}
        geometry_only=all((any(x.startswith(p) for p in benign_prefixes) or x in benign_exact) for x in rr) if rr else True
        if geometry_only:
          base['state']='CAUTION_SHADOW'; base['label']='CHECK ?'
          base['reasons']=['lane_change_rebase']+list(base.get('reasons') or [])[:5]
      return base

    base['phase']='ACTIVE'
    return base

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

  def update(self, objects: list[dict], v_ego: float=0.0, a_ego: float=0.0,
             left_blinker: bool=False, right_blinker: bool=False, now_ns: int=0,
             steering_angle_deg: float=0.0, road_curve_direction: str | None=None,
             road_model: dict | None=None) -> dict:
    if not now_ns:
      import time
      now_ns=time.monotonic_ns()
    left=self._stabilize_incoming(self._side(objects,+1,a_ego),+1,now_ns)
    right=self._stabilize_incoming(self._side(objects,-1,a_ego),-1,now_ns)
    lane_availability=adjacent_lane_availability(road_model)
    left['lane_availability']=dict(lane_availability.get('left') or {})
    right['lane_availability']=dict(lane_availability.get('right') or {})
    left['decision_raw']=self._apply_road_lane_gate(self._decision(left), left['lane_availability'])
    right['decision_raw']=self._apply_road_lane_gate(self._decision(right), right['lane_availability'])
    left['decision']=self._stabilize_decision(left,+1,now_ns); right['decision']=self._stabilize_decision(right,-1,now_ns)
    active=None; active_side=None
    if left_blinker and not right_blinker:
      active='left'; active_side=left
    elif right_blinker and not left_blinker:
      active='right'; active_side=right

    if left_blinker and right_blinker:
      intent={'active':False,'side':None,'label':'HAZARD','state':'HAZARD','raw_state':None,
              'reasons':['both_blinkers'],'front_clearance_m':None,'rear_clearance_m':None,
              'boundary_clearance_m':None,'front_ttc_s':None,'rear_ttc_s':None,'shadow_only':True,
              'maneuver_context':'HAZARD','phase':'HAZARD','committed':False,'commit_age_s':None,
              'hold_remaining_s':None,'steering_angle_deg':round(float(steering_angle_deg or 0.0),2),
              'road_curve_direction':str(road_curve_direction or 'UNKNOWN').upper()}
    else:
      intent=self._apply_intent_state(active,active_side,v_ego,steering_angle_deg,road_curve_direction,now_ns)

    return {
      'api_version':FUTURE_GAP_API_VERSION,'mode':'SHADOW_GEOMETRY_FG6_DEC3_ROAD_GATE',
      'decision_enabled':True,'safe_caution_blocked_enabled':True,'decision_shadow_only':True,
      'active_target':active,'driver_intent':intent,'horizons_s':list(HORIZONS_S),
      'lane_change_duration_s':LANE_CHANGE_DURATION_S,'vehicle_long_margin_m':OBJECT_LONG_MARGIN_M,
      'vehicle_half_width_m':OBJECT_HALF_WIDTH_M,
      'ego':{'v_ego_mps':round(float(v_ego),3),'a_ego_mps2':round(float(a_ego),3),
             'left_blinker':bool(left_blinker),'right_blinker':bool(right_blinker),
             'steering_angle_deg':round(float(steering_angle_deg or 0.0),2),
             'road_curve_direction':str(road_curve_direction or 'UNKNOWN').upper()},
      'left':left,'right':right,'lane_availability':lane_availability,
      'stats':{'visible_objects':len(objects),'left_incoming_confirmed':left['incoming_count'],
               'right_incoming_confirmed':right['incoming_count'],'left_incoming_stable':left['stable_incoming_count'],
               'right_incoming_stable':right['stable_incoming_count'],'left_incoming_possible':left['possible_incoming_count'],
               'right_incoming_possible':right['possible_incoming_count'],'left_current_core':left['horizons'][0]['core_occupant_count'],
               'right_current_core':right['horizons'][0]['core_occupant_count'],'left_current_boundary':left['horizons'][0]['boundary_overlap_count'],
               'right_current_boundary':right['horizons'][0]['boundary_overlap_count']},
      'note':'FG6 + DEC3 shadow only: target-lane road-geometry gate + turn/lane-change context + commit hold/rebase + dual-side preview HUD; no planner/CAN control.'
    }


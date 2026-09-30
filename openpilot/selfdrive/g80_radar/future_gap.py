#!/usr/bin/env python3
"""V39R1 shadow Future Gap + Target-Lane Occupancy evaluator.

Consumes Canonical360 vehicles after KF4/IMM3 and computes diagnostic target-lane
geometry for NOW/0.5/1/2/3 s.  V40 retains FG8 maneuver context and adds origin-aware FG9 risk evidence to DEC3 maneuver-context handling:
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

FUTURE_GAP_API_VERSION = 9
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

# V40 retains FG8 maneuver-context / lane-change commit handling.
TURN_CURVE_MATCH_MAX_SPEED_MPS = 8.5      # ~30.6 km/h
TURN_STEERING_MIN_DEG = 35.0
TURN_STEERING_MAX_SPEED_MPS = 14.0        # allow a brisk intersection turn to be classified
TURN_LOW_SPEED_STEERING_MIN_DEG = 18.0    # catch intersection turns before 35 deg is reached
TURN_LOW_SPEED_MAX_MPS = 6.0              # ~21.6 km/h
TURN_EXIT_HOLD_S = 1.80                    # normal release hold after explicit TURN evidence
TURN_LOW_SPEED_LATCH_MPS = 5.0               # once TURN is seen, keep it while creeping/stopped
TURN_WAIT_MAX_SPEED_MPS = 2.5                 # blinker + unconfirmed lane at crawl => TURN? instead of lane-change
TURN_NO_LANE_APPROACH_MAX_MPS = 12.0          # target lane absent at urban speed => likely intersection/road turn
LANE_CHANGE_COMMIT_MIN_SPEED_MPS = 5.0
LANE_CHANGE_COMMIT_STEER_DEG = 4.0
LANE_CHANGE_COMMIT_MIN_BLINKER_S = 0.15
LANE_CHANGE_DECISION_HOLD_S = 1.50
LANE_CHANGE_REBASE_S = 1.50
LANE_CHANGE_SESSION_TIMEOUT_S = 4.5
HARD_OVERRIDE_CLEARANCE_M = 1.5
HARD_OVERRIDE_TTC_S = 1.5
HARD_OVERRIDE_CONFIRM_S = 0.20                # identity-stable hard conflict before breaking commit/rebase hold
HARD_OVERRIDE_CONFIRM_COUNT = 2
LANE_CHANGE_COMMIT_LANE_STABLE_S = 0.35

# Road/lane geometry is noisy frame-to-frame.  Stabilize the semantic lane state
# before it reaches the driver arrows.  These are display/shadow-only timers.
LANE_GATE_ENTER_S = 0.55
LANE_GATE_SWITCH_S = 0.75
LANE_GATE_UNCERTAIN_HOLD_S = 1.20

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

# FG9 separates the *current* physical gap from a future-horizon extrapolation.
# These are diagnostic/UI-only gates.  In particular, a current FRONT target
# must never be relabelled as a REAR vehicle just because its predicted s crosses 0.
FG9_PREDICTED_CONFIRM_S = 0.25
FG9_PREDICTED_MIN_COUNT = 2
FG9_PREDICTED_FORGET_S = 0.85
FG9_MAX_SOURCE_AGE_MS = 350.0
FG9_CROSSING_TTC_S = 3.0
FG9_URGENT_TTC_S = 1.5


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
      'latched_decision': None, 'precommit_decision': None, 'last_active_ns': 0, 'last_context': 'STANDBY',
      'last_turn_evidence_ns': 0
    }
    self.lane_gate_hist = {
      +1:{'stable':'UNCERTAIN','candidate':'UNCERTAIN','candidate_since_ns':0,'last_strong_ns':0},
      -1:{'stable':'UNCERTAIN','candidate':'UNCERTAIN','candidate_since_ns':0,'last_strong_ns':0},
    }
    self.hard_override_hist = {'side':None,'key':None,'since_ns':0,'last_ns':0,'count':0}
    self.fg9_prediction_hist: dict[tuple[int,str,str],dict] = {}

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
  def _decision_legacy(side: dict) -> dict:
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


  def _fg9_build_evidence(self, side: dict, target_idx: int, now_ns: int) -> dict:
    """Link each predicted closest pass to THAT SAME vehicle's observed state.

    Old DEC3 aggregated current nearest TTC and min future gap independently;
    both may belong to different keys.  In logs 23/25 rear-gap warnings during
    signalling were actually vehicles originally AHEAD of the ego, extrapolated
    through s=0 and subsequently named 'rear'.  Here origin is immutable.
    """
    obj=side.get('fg9_object_states') or {}
    horizon=side.get('horizons') or []
    stable={str(t.get('key')) for t in side.get('stable_incoming') or []}
    near=[]
    have=set()
    live_hist=set()
    for key,x in obj.items():
      s0=_finite(x.get('s')); v0=_finite(x.get('s_dot')); age=_finite(x.get('source_age_ms'),9999.0)
      if s0 is None or v0 is None:continue
      if not x.get('current_core') and not x.get('current_boundary') and key not in stable and not x.get('forecast_near'):
        continue
      origin='front' if s0>=0 else 'rear'
      closing=bool((s0>0 and v0 < -0.25) or (s0<0 and v0>0.25))
      ttc=abs(s0/v0) if closing else None
      # CA TTC can be informative diagnostically, but acceleration alone is not
      # sufficient to label a distant target DANGER if its present motion is away.
      cur_clearance=max(0.0,abs(s0)-OBJECT_LONG_MARGIN_M-min(MAX_LONG_SIGMA_MARGIN_M,max(0.0,_finite(x.get('s_sigma'),0.0))))
      predicted=[]
      pred_core=[]; pred_boundary=[]
      for h in horizon:
        if not h.get('ego_target_overlap'):continue
        hsec=_finite(h.get('t'),0.0)
        for typ in ('front','rear','boundary_front','boundary_rear'):
          item=h.get(typ) or {}
          if str(item.get('key') or '')!=key:continue
          c=_finite(item.get('clearance_m'))
          if c is None:continue
          predicted.append((c,hsec,typ,_finite(item.get('s'))))
          if typ.startswith('boundary'):pred_boundary.append((c,hsec,typ))
          else:pred_core.append((c,hsec,typ))
      # Only measured point at t=0 can be an *immediate* hazard.  A path point
      # at t=2 s can be a prediction rather than an actual nearby vehicle.
      current_core=bool(x.get('current_core'))
      current_boundary=bool(x.get('current_boundary'))
      immediate=(current_core and cur_clearance<=DEC_BLOCK_CLEARANCE_M) or (current_boundary and cur_clearance<=DEC_BLOCK_BOUNDARY_M)
      nearest=min(predicted,default=None,key=lambda a:a[0])
      pmin=nearest[0] if nearest else None
      ptime=nearest[1] if nearest else None
      # Strong imminent closing evidence, including approaching rear vehicles.
      # Only enforce this when the source is recent, lanes align, AND the
      # identified object is stable over time; one-frame predictions stay CHECK.
      recent=age is not None and age <= FG9_MAX_SOURCE_AGE_MS
      future_severe=bool(pmin is not None and pmin <= DEC_BLOCK_CLEARANCE_M)
      lane_corrob=bool(current_core or key in stable)
      imminent=bool(closing and ttc is not None and 0.5 <= ttc <= FG9_CROSSING_TTC_S)
      pred_key=(int(target_idx),key,origin)
      active=bool(future_severe and recent and imminent and lane_corrob)
      if active:
        live_hist.add(pred_key)
        prev=self.fg9_prediction_hist.get(pred_key)
        if prev is None or int(now_ns)-int(prev.get('last_ns',0))>int(FG9_PREDICTED_FORGET_S*1e9):
          prev={'since_ns':int(now_ns),'count':1,'last_ns':int(now_ns)}
        else:
          prev['count']=int(prev.get('count',0))+1;prev['last_ns']=int(now_ns)
        self.fg9_prediction_hist[pred_key]=prev
        stable_s=max(0.0,(int(now_ns)-int(prev['since_ns']))/1e9)
        persistent=bool(prev['count']>=FG9_PREDICTED_MIN_COUNT and stable_s>=FG9_PREDICTED_CONFIRM_S)
      else:
        persistent=False;stable_s=0.0
      # Real current overlap can be urgent before a two-sample predictor gate.
      confirmed=bool(immediate or (future_severe and imminent and lane_corrob and recent and persistent))
      item={'key':key,'origin':origin,'s_now_m':round(s0,2),'d_now_m':round(float(x.get('d') or 0.0),2),'current_gap_m':round(cur_clearance,2),
            'current_core':current_core,'current_boundary':current_boundary,
            'relative_s_dot_mps':round(v0,2),'closing':closing,'ttc_linear_s':round(ttc,2) if ttc is not None else None,
            'source_age_ms':round(age,1) if age is not None else None,'source_mask':x.get('source_mask') or [],
            'track_duration_s':round(float(x.get('track_duration_s') or 0.0),2),
            'future_min_m':round(pmin,2) if pmin is not None else None,
            'future_min_t_s':ptime,'future_min_kind':nearest[2] if nearest else None,
            'forecast_core':bool(pred_core),'stable_incoming':key in stable,
            'prediction_persistence_s':round(stable_s,3),'prediction_persistent':persistent,
            'immediate':immediate,'predicted_severe':future_severe,'confirmed':confirmed,
            'fresh':recent,'uncertain_prediction':bool(future_severe and not confirmed),
            'front_crossed_into_predicted_rear':bool(origin=='front' and any(c<=5 and typ=='rear' for c,t,typ,_ in predicted)),
            'rear_closed_into_predicted_front':bool(origin=='rear' and any(c<=5 and typ=='front' for c,t,typ,_ in predicted))}
      near.append(item)
    # Bound the history and published evidence.  Active risk keys stay cached;
    # a dropped/reacquired identity must not inherit another vehicle's proof.
    for k,h in list(self.fg9_prediction_hist.items()):
      if int(now_ns)-int(h.get('last_ns',0))>int(FG9_PREDICTED_FORGET_S*1e9):
        del self.fg9_prediction_hist[k]
    near.sort(key=lambda x:(not x['confirmed'],x['future_min_m'] if x['future_min_m'] is not None else 999))
    all_now=[x for x in near if x['current_core']]
    front=[x['current_gap_m'] for x in all_now if x['origin']=='front']
    rear=[x['current_gap_m'] for x in all_now if x['origin']=='rear']
    out={'observations':near[:30],
         'current_front_gap_m':min(front,default=None),'current_rear_gap_m':min(rear,default=None),
         'predicted_front_min_m':side.get('min_front_clearance_during_ego_overlap_m'),
         'predicted_rear_min_m':side.get('min_rear_clearance_during_ego_overlap_m'),
         'near_future_predicted_count':sum(bool(x['predicted_severe']) for x in near),
         'near_future_confirmed_count':sum(bool(x['confirmed']) for x in near),
         'front_to_rear_predictions':sum(bool(x['front_crossed_into_predicted_rear']) for x in near),
         'origin_aware':True}
    side['fg9_evidence']=out
    return out

  @staticmethod
  def _decision(side: dict) -> dict:
    """FG9 risk tiers: immediate physical hazard / corroborated approaching /
    prediction-only CHECK.  Never use the min future REAR gap of a FRONT car as
    a measurement of rear-approach danger.
    """
    e=side.get('fg9_evidence') or {}
    items=e.get('observations') or []
    hard=[]; caution=[]
    def add(lst,value):
      if value not in lst:lst.append(value)
    for x in items:
      gap=x.get('current_gap_m')
      ttc=x.get('ttc_linear_s')
      if x.get('immediate'):
        add(hard,'current_'+('front' if x['origin']=='front' else 'rear')+'_close')
      elif gap is not None and x.get('current_core') and gap<=DEC_CAUTION_CLEARANCE_M:
        add(caution,'current_'+x['origin']+'_watch')
      if x.get('confirmed') and not x.get('immediate'):
        if x.get('front_crossed_into_predicted_rear'):
          add(hard,'confirmed_FRONT_crossover')
        else:
          add(hard,'confirmed_'+x['origin']+'_closing')
      elif x.get('uncertain_prediction'):
        add(caution,'unconfirmed_'+('FRONT_crossover' if x.get('front_crossed_into_predicted_rear') else x['origin']+'_forecast'))
      if x.get('current_core') and x.get('closing') and ttc is not None:
        if ttc<=FG9_URGENT_TTC_S and x.get('fresh'):
          add(hard,'immediate_'+x['origin']+'_TTC')
        elif ttc<=DEC_CAUTION_TTC_S:
          add(caution,x['origin']+'_TTC_watch')
    for x in side.get('stable_incoming') or []:
      k=str(x.get('key') or '')
      if (float(x.get('entry_eta_s',99))<=DEC_BLOCK_INCOMING_ETA_S and
          abs(float(x.get('entry_s_m',999)))<=DEC_BLOCK_INCOMING_ABS_S_M):
        # Incoming is already temporally confirmed by _stabilize_incoming.
        # Corroborate with a fresh canonical object before declaring DANGER.
        obs=next((r for r in items if r['key']==k),None)
        if obs is not None and obs['fresh']:
          add(hard,'stable_incoming_near')
        else:add(caution,'incoming_source_uncertain')
      elif float(x.get('entry_eta_s',99))<=DEC_CAUTION_INCOMING_ETA_S and abs(float(x.get('entry_s_m',999)))<=DEC_CAUTION_INCOMING_ABS_S_M:
        add(caution,'stable_incoming')
    if not hard:
      for x in side.get('possible_incoming') or []:
        if float(x.get('entry_eta_s',99))<=1.5 and abs(float(x.get('entry_s_m',999)))<=25.0:
          add(caution,'possible_incoming_near');break
    # Hypothetical braking scenarios only justify CHECK, not hard danger.
    sc=side.get('scenarios') or {}
    if any(((sc.get(name) or {}).get('min_clearance_m') is not None and (sc.get(name) or {}).get('min_clearance_m')<=8.0)
           for name in ('front_target_brake','rear_ego_brake')):
      add(caution,'what_if_braking')
    # Bound FG9 against DEC3: V40 must NOT manufacture additional red warnings
    # from an instantaneous sensor close pass that FG8 did not flag.  Equally,
    # missing corroboration changes a legacy red to yellow, NEVER to green.
    legacy=side.get('legacy_decision_raw') or FutureGapEvaluator._decision_legacy(side)
    old_state=legacy.get('state')
    if hard and old_state!='BLOCKED_SHADOW':
      hard=[]
      add(caution,'extra_hard_evidence_watch_only')
    if not hard and old_state=='BLOCKED_SHADOW':
      add(caution,'legacy_block_unconfirmed')
    elif not hard and not caution and old_state=='CAUTION_SHADOW':
      add(caution,'legacy_watch_unverified')
    # If there is not enough information to confirm a lane, road gate will
    # override the *display* separately; no prediction-only downgrade to green.
    state='BLOCKED_SHADOW' if hard else ('CAUTION_SHADOW' if caution else 'SAFE_SHADOW')
    return {'state':state,'reasons':(hard if hard else caution)[:6],
            'policy':'FG9 origin-aware same-key current gap + stable future evidence; display/shadow only',
            'predicted_only':bool(caution and not hard),
            'diagnostic_not_permission':True}

  def _stabilize_lane_availability(self, availability: dict | None, target_idx: int, now_ns: int) -> dict:
    """Debounce noisy C4 lane-line/road-edge geometry for the driver preview.

    Logs from V38R3 showed ~800 CONFIRMED/UNCERTAIN/ABSENT transitions in ~23 min.
    Keep the last strong state briefly through uncertain frames, and require
    persistence before switching between CONFIRMED and ABSENT.
    """
    av=dict(availability or {})
    raw=str(av.get('status') or 'UNCERTAIN').upper()
    if raw not in ('CONFIRMED','ABSENT','UNCERTAIN'):
      raw='UNCERTAIN'
    h=self.lane_gate_hist[int(target_idx)]
    now_ns=int(now_ns)
    if h.get('candidate') != raw:
      h['candidate']=raw; h['candidate_since_ns']=now_ns
    cand_age=max(0.0,(now_ns-int(h.get('candidate_since_ns',now_ns) or now_ns))/1e9)
    stable=str(h.get('stable') or 'UNCERTAIN')
    if raw in ('CONFIRMED','ABSENT'):
      h['last_strong_ns']=now_ns
      need=LANE_GATE_ENTER_S if stable=='UNCERTAIN' else LANE_GATE_SWITCH_S
      if raw==stable or cand_age>=need:
        stable=raw
    else:
      # One or several weak frames should not make a confirmed/absent lane blink.
      last_strong=int(h.get('last_strong_ns',0) or 0)
      strong_age=(now_ns-last_strong)/1e9 if last_strong else 1e9
      if stable=='UNCERTAIN' or (cand_age>=LANE_GATE_ENTER_S and strong_age>=LANE_GATE_UNCERTAIN_HOLD_S):
        stable='UNCERTAIN'
    h['stable']=stable; self.lane_gate_hist[int(target_idx)]=h
    av['raw_status']=raw
    av['status']=stable
    av['stabilized']=True
    av['candidate_age_s']=round(cand_age,3)
    av['hold_policy']='V38R4 lane geometry temporal hysteresis'
    return av

  def _apply_road_lane_gate(self, raw: dict, availability: dict | None, side: dict | None = None) -> dict:
    """Road semantics take precedence over passive preview noise.

    ABSENT always means NO LANE. UNCERTAIN means CHECK ROAD unless a fresh hard
    TTC/clearance conflict exists. CONFIRMED passes the vehicle-gap decision through.
    This affects shadow/UI labels only and never planner/CAN.
    """
    out=dict(raw or {})
    av=availability or {}
    status=str(av.get('status') or 'UNCERTAIN').upper()
    out['road_gate']=status
    if status=='CONFIRMED':
      return out
    if status=='ABSENT':
      out['state']='CAUTION_SHADOW'; out['label_override']='NO LANE'
      out['reasons']=['target_lane_absent'] + [r for r in list(out.get('reasons') or []) if r!='target_lane_absent'][:5]
      return out
    # UNCERTAIN: retain only a genuinely hard immediate hazard as DANGER.
    hard=self._hard_override(side or {})
    # UNCERTAIN road geometry: being near an *adjacent* car is not the same
    # as a current ego-footprint overlap.  If there's direct physical overlap,
    # keep red; otherwise unknown road stays CHECK ROAD until lane confirmed.
    direct=any(bool(x.get('immediate') and x.get('fresh') and
                    abs(float(x.get('d_now_m') if x.get('d_now_m') is not None else 999))<=EGO_HALF_WIDTH_M+OBJECT_HALF_WIDTH_M+0.2 and
                    float(x.get('current_gap_m') if x.get('current_gap_m') is not None else 999)<=HARD_OVERRIDE_CLEARANCE_M)
               for x in ((side or {}).get('fg9_evidence') or {}).get('observations',[]))
    if str(out.get('state'))=='BLOCKED_SHADOW' and (hard or direct):
      out['reasons']=['road_uncertain_hard_hazard'] + list(out.get('reasons') or [])[:5]
      return out
    out['state']='CAUTION_SHADOW'; out['label_override']='CHECK ROAD'
    out['reasons']=['target_lane_unconfirmed'] + [r for r in list(out.get('reasons') or []) if r!='target_lane_unconfirmed'][:5]
    return out

  def _stabilize_decision(self, side: dict, target_idx: int, now_ns: int) -> dict:
    raw=dict(side.get('decision_raw') or self._decision(side))
    h=self.display_hist[int(target_idx)]
    raw_state=str(raw.get('state') or 'CAUTION_SHADOW')
    if h.get('raw') != raw_state:
      h['raw']=raw_state; h['raw_since_ns']=int(now_ns)
    age_s=max(0.0,(int(now_ns)-int(h.get('raw_since_ns') or now_ns))/1e9)
    cur=str(h.get('state') or 'CAUTION_SHADOW')
    semantic_override=str(raw.get('label_override') or '')
    # Road semantics (NO LANE / CHECK ROAD) are a different condition from a
    # vehicle-gap DANGER. Do not let the previous red state linger when the
    # stabilized road gate says the target lane itself is absent/unconfirmed.
    if raw_state == 'CAUTION_SHADOW' and semantic_override in ('NO LANE','CHECK ROAD'):
      cur='CAUTION_SHADOW'
    elif raw_state == 'BLOCKED_SHADOW': cur='BLOCKED_SHADOW'
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
    steer_signed = float(steering_angle_deg or 0.0)
    steer = abs(steer_signed)
    steer_direction_match = bool((active=='left' and steer_signed>0.0) or (active=='right' and steer_signed<0.0))
    speed = max(0.0, float(v_ego or 0.0))
    turn_by_curve = bool(curve_match and speed <= TURN_CURVE_MATCH_MAX_SPEED_MPS)
    turn_by_steer = bool(steer_direction_match and steer >= TURN_STEERING_MIN_DEG and speed <= TURN_STEERING_MAX_SPEED_MPS)
    turn_by_low_speed_steer = bool(steer_direction_match and steer >= TURN_LOW_SPEED_STEERING_MIN_DEG and speed <= TURN_LOW_SPEED_MAX_MPS)
    kind = 'TURN' if (turn_by_curve or turn_by_steer or turn_by_low_speed_steer) else 'LANE_CHANGE'
    return {'kind':kind,'curve_match':curve_match,'turn_by_curve':turn_by_curve,
            'turn_by_steer':turn_by_steer,'turn_by_low_speed_steer':turn_by_low_speed_steer,
            'steer_direction_match':steer_direction_match,'steering_signed_deg':round(steer_signed,2),
            'curve_direction':curve,'steering_abs_deg':round(steer,2)}

  @staticmethod
  def _hard_override(side: dict) -> bool:
    """Also fixes a pre-existing FG8 AttributeError in UNCERTAIN road gate.

    Never mix current TTC of vehicle A with predicted clearance of vehicle B.
    """
    return FutureGapEvaluator._hard_override_candidate(side) is not None

  @staticmethod
  def _hard_override_candidate(side: dict) -> dict | None:
    """FG9: hard override requires SAME keyed object for TTC and min clearance.

    V39 combined nearest current TTC and nearest future gap for unrelated cars.
    This could produce a synthetic hard conflict during rebase/identity churn.
    """
    obs=(side.get('fg9_evidence') or {}).get('observations') or []
    can=[]
    for x in obs:
      gap=x.get('future_min_m')
      ttc=x.get('ttc_linear_s')
      if not (x.get('confirmed') and x.get('fresh') and x.get('closing')):
        continue
      best=min([v for v in (gap,x.get('current_gap_m') if x.get('immediate') else None) if v is not None],default=None)
      if best is None or ttc is None or best>HARD_OVERRIDE_CLEARANCE_M or ttc>HARD_OVERRIDE_TTC_S:
        continue
      can.append({'side':x['origin'],'key':x['key'],'clearance_m':best,'ttc_s':ttc,'same_key':True})
    return min(can,key=lambda x:(x['ttc_s'],x['clearance_m'])) if can else None

  def _stable_hard_override(self, active: str | None, side: dict, now_ns: int) -> tuple[bool, dict | None]:
    cand=self._hard_override_candidate(side)
    h=self.hard_override_hist
    if cand is None:
      self.hard_override_hist={'side':active,'key':None,'since_ns':0,'last_ns':int(now_ns),'count':0}
      return False,None
    key=str(cand.get('key') or '')
    continuous=(h.get('side')==active and h.get('key')==key and int(now_ns)-int(h.get('last_ns',0) or 0) <= int(0.9e9))
    if continuous:
      h['last_ns']=int(now_ns); h['count']=int(h.get('count',0))+1
    else:
      h={'side':active,'key':key,'since_ns':int(now_ns),'last_ns':int(now_ns),'count':1}
      self.hard_override_hist=h
    age=max(0.0,(int(now_ns)-int(h.get('since_ns',now_ns)))/1e9)
    confirmed=bool(int(h.get('count',0))>=HARD_OVERRIDE_CONFIRM_COUNT and age>=HARD_OVERRIDE_CONFIRM_S)
    diag=dict(cand); diag.update({'age_s':round(age,3),'count':int(h.get('count',0)),'confirmed':confirmed})
    return confirmed,diag

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
                          'latched_decision':None,'precommit_decision':None,'last_active_ns':0,'last_context':'STANDBY','last_turn_evidence_ns':0}
        self.hard_override_hist={'side':None,'key':None,'since_ns':0,'last_ns':0,'count':0}
      return base

    if h.get('side') != active or (int(now_ns)-int(h.get('last_active_ns',0) or 0) > int(LANE_CHANGE_SESSION_TIMEOUT_S*1e9)):
      h={'side':active,'blinker_since_ns':int(now_ns),'committed':False,'commit_ns':0,
         'latched_decision':None,'precommit_decision':None,'last_active_ns':int(now_ns),'last_context':'STANDBY','last_turn_evidence_ns':0}
      self.intent_hist=h
      self.hard_override_hist={'side':active,'key':None,'since_ns':0,'last_ns':0,'count':0}
    else:
      h['last_active_ns']=int(now_ns)

    ctx=self._context(active,v_ego,steering_angle_deg,road_curve_direction)
    lane_meta=dict(active_side.get('lane_availability') or {})
    lane_status=str(lane_meta.get('status') or 'UNCERTAIN').upper()
    speed=max(0.0,float(v_ego or 0.0))
    raw_ctx=ctx['kind']
    explicit_turn=raw_ctx=='TURN'
    no_lane_approach=bool(raw_ctx!='TURN' and lane_status=='ABSENT' and speed<=TURN_NO_LANE_APPROACH_MAX_MPS)
    low_speed_wait=bool(raw_ctx!='TURN' and lane_status!='CONFIRMED' and speed<=TURN_WAIT_MAX_SPEED_MPS)
    if explicit_turn or no_lane_approach:
      h['last_turn_evidence_ns']=int(now_ns)
      if no_lane_approach:
        ctx=dict(ctx); ctx['kind']='TURN'; ctx['turn_approach_no_target_lane']=True
    elif low_speed_wait:
      # At a crawl with no confirmed adjacent lane, do not invent a lane-change
      # permission state.  Mark it as a pending turn/road maneuver until geometry
      # becomes clear or the blinker is cancelled.
      ctx=dict(ctx); ctx['kind']='TURN'; ctx['turn_wait_unconfirmed_lane']=True
    else:
      lt=int(h.get('last_turn_evidence_ns',0) or 0)
      if lt:
        age=(int(now_ns)-lt)/1e9
        keep_low_speed=bool(speed<=TURN_LOW_SPEED_LATCH_MPS)
        keep_unconfirmed=bool(lane_status!='CONFIRMED' and speed<=TURN_CURVE_MATCH_MAX_SPEED_MPS)
        keep_timed=bool(age<TURN_EXIT_HOLD_S)
        if keep_low_speed or keep_unconfirmed or keep_timed:
          ctx=dict(ctx); ctx['kind']='TURN'; ctx['turn_release_hold']=True
          ctx['turn_hold_remaining_s']=None if (keep_low_speed or keep_unconfirmed) else round(max(0.0,TURN_EXIT_HOLD_S-age),3)
          ctx['turn_hold_low_speed']=keep_low_speed
          ctx['turn_hold_lane_unconfirmed']=keep_unconfirmed
    h['last_context']=ctx['kind']
    base['maneuver_context']=ctx['kind']
    base['context_detail']=ctx
    base['blinker_age_s']=round(max(0.0,(int(now_ns)-int(h.get('blinker_since_ns',now_ns)))/1e9),3)

    dec=dict(active_side.get('decision') or {})
    base.update({'label':dec.get('label','CHECK ?'),'state':dec.get('state','CAUTION_SHADOW'),
                 'raw_state':dec.get('raw_state'),'reasons':dec.get('reasons',[])[:6],
                 'front_clearance_m':(active_side.get('fg9_evidence') or {}).get('current_front_gap_m'),
                 'rear_clearance_m':(active_side.get('fg9_evidence') or {}).get('current_rear_gap_m'),
                 'predicted_front_min_m':active_side.get('min_front_clearance_during_ego_overlap_m'),
                 'predicted_rear_min_m':active_side.get('min_rear_clearance_during_ego_overlap_m'),
                 'boundary_clearance_m':active_side.get('min_boundary_clearance_during_ego_overlap_m'),
                 'front_ttc_s':active_side.get('current_front_ttc_ca_s'),
                 'rear_ttc_s':active_side.get('current_rear_ttc_ca_s'),
                 'lane_availability':dict(active_side.get('lane_availability') or {})})

    if ctx['kind']=='TURN':
      # Once an intersection/road turn is established, do not flip back to
      # lane-change while creeping, while the target lane is absent/unconfirmed,
      # or during the short steering unwind after the turn.
      h['committed']=False; h['commit_ns']=0; h['latched_decision']=None; h['precommit_decision']=None
      self.hard_override_hist={'side':active,'key':None,'since_ns':0,'last_ns':int(now_ns),'count':0}
      hold=bool(ctx.get('turn_release_hold'))
      approach=bool(ctx.get('turn_approach_no_target_lane'))
      wait=bool(ctx.get('turn_wait_unconfirmed_lane'))
      if approach:
        phase='TURN_APPROACH'; label='TURN ?'; reason='turn_approach_no_target_lane'
      elif wait:
        phase='TURN_WAIT'; label='TURN ?'; reason='turn_wait_unconfirmed_lane'
      elif hold:
        phase='TURN_HOLD'; label='TURN'; reason='intersection_turn_release_hold'
      else:
        phase='TURNING'; label='TURN'; reason='intersection_turn_context'
      base.update({'label':label,'state':'TURN','phase':phase,'committed':False,
                   'reasons':[reason]})
      return base

    # Lane-change context.  V38 log replay showed a false COMMIT while
    # the requested side was explicitly NO LANE.  Commitment now requires a
    # temporally stable confirmed target lane in addition to motion/steering.
    blink_age=max(0.0,(int(now_ns)-int(h.get('blinker_since_ns',now_ns)))/1e9)
    steer=abs(float(steering_angle_deg or 0.0))
    lane_age=float(lane_meta.get('candidate_age_s') or 0.0)
    lane_commit_ready=bool(lane_status=='CONFIRMED' and lane_age>=LANE_CHANGE_COMMIT_LANE_STABLE_S)
    base['lane_commit_ready']=lane_commit_ready
    base['lane_commit_status']=lane_status
    base['lane_commit_age_s']=round(lane_age,3)
    if (not h.get('committed') and lane_commit_ready and float(v_ego)>=LANE_CHANGE_COMMIT_MIN_SPEED_MPS and
        blink_age>=LANE_CHANGE_COMMIT_MIN_BLINKER_S and steer>=LANE_CHANGE_COMMIT_STEER_DEG):
      h['committed']=True; h['commit_ns']=int(now_ns)
      h['latched_decision']=dict(h.get('precommit_decision') or dec)

    if not h.get('committed'):
      h['precommit_decision']=dict(dec)
      base['phase']='PRECHECK'
      if not lane_commit_ready and lane_status!='CONFIRMED':
        base['reasons']=['lane_change_commit_wait_for_lane']+(base.get('reasons') or [])[:5]
      return base

    age=max(0.0,(int(now_ns)-int(h.get('commit_ns',now_ns)))/1e9)
    base['committed']=True; base['commit_age_s']=round(age,3)
    lat=dict(h.get('latched_decision') or dec)

    hard_confirmed,hard_diag=self._stable_hard_override(active,active_side,now_ns)
    base['hard_override']=hard_diag
    if age < LANE_CHANGE_DECISION_HOLD_S:
      base['phase']='COMMIT_HOLD'
      base['hold_remaining_s']=round(max(0.0,LANE_CHANGE_DECISION_HOLD_S-age),3)
      if hard_confirmed:
        base['reasons']=['commit_hold_stable_hard_ttc_override']+(dec.get('reasons') or [])[:5]
      else:
        base['state']=lat.get('state','CAUTION_SHADOW')
        base['label']=lat.get('label','CHECK ?')
        base['reasons']=['lane_change_commit_hold']+(lat.get('reasons') or [])[:5]
      return base

    if age < LANE_CHANGE_DECISION_HOLD_S + LANE_CHANGE_REBASE_S:
      base['phase']='REBASING'
      base['hold_remaining_s']=round(max(0.0,LANE_CHANGE_DECISION_HOLD_S+LANE_CHANGE_REBASE_S-age),3)
      # During target-lane coordinate re-centering, V38 logs showed the minimum
      # hazard key changing almost every frame (e.g. V0594 -> V0603 -> V0597).
      # Treat an unconfirmed one-frame zero-gap/TTC as CHECK, not DANGER.  A real
      # hard conflict still breaks through once the same canonical key persists.
      if base.get('state')=='BLOCKED_SHADOW' and not hard_confirmed:
        base['state']='CAUTION_SHADOW'; base['label']='CHECK ?'
        base['reasons']=['lane_change_rebase_identity_guard']+list(base.get('reasons') or [])[:5]
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

    # FG9 per-key anchor.  These live object states never alias the
    # minimum-gap key of a different horizon or another vehicle.
    fg9_object_states={}
    for k,o in obj_by_key.items():
      st=states_by_key[k][0.0]
      in_core=bool(current_core.get(k))
      overlaps=bool(current_overlap.get(k))
      has_future=any((h.get('front') or {}).get('key')==k or
                     (h.get('rear') or {}).get('key')==k or
                     (h.get('boundary_front') or {}).get('key')==k or
                     (h.get('boundary_rear') or {}).get('key')==k
                     for h in horizons if h.get('ego_target_overlap'))
      fg9_object_states[k]={'s':st.get('s'),'d':st.get('d'),'s_dot':st.get('s_dot'),
           's_sigma':st.get('s_sigma'),'d_sigma':st.get('d_sigma'),
           'current_core':in_core,'current_boundary':overlaps and not in_core,
           'origin_lane':origin_lane.get(k),'forecast_near':has_future,
           'source_age_ms':o.get('source_age_ms',9999.0),
           'source_mask':list(o.get('source_mask') or []),
           'track_duration_s':o.get('canonical_track_duration_s',0.0)}

    return {
      'target_lane':lane_name(target_idx),'target_lane_index':target_idx,
      'fg9_object_states':fg9_object_states, 'horizons':horizons,
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
    lane_availability_raw=adjacent_lane_availability(road_model)
    left['lane_availability']=self._stabilize_lane_availability(lane_availability_raw.get('left'),+1,now_ns)
    right['lane_availability']=self._stabilize_lane_availability(lane_availability_raw.get('right'),-1,now_ns)
    lane_availability={'left':dict(left['lane_availability']),'right':dict(right['lane_availability']),'fresh':bool(lane_availability_raw.get('fresh'))}
    self._fg9_build_evidence(left,+1,now_ns)
    self._fg9_build_evidence(right,-1,now_ns)
    left.pop('fg9_object_states',None)
    right.pop('fg9_object_states',None)
    # Preserve the exact old DEC3 output to diagnose any disagreement in logs.
    left['legacy_decision_raw']=self._decision_legacy(left)
    right['legacy_decision_raw']=self._decision_legacy(right)
    left['decision_raw']=self._apply_road_lane_gate(self._decision(left), left['lane_availability'],left)
    right['decision_raw']=self._apply_road_lane_gate(self._decision(right), right['lane_availability'],right)
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
      'api_version':FUTURE_GAP_API_VERSION,'mode':'SHADOW_GEOMETRY_FG9_ORIGIN_SAMEKEY_PREDICTED_GUARD',
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
      'note':'FG9 origin-aware same-key gaps and TTC; current gap distinct from 3s forecast; FG8 TURN/commit retained; shadow only, NO permission and NO CAN control.'
    }


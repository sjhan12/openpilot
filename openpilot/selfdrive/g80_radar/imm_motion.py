#!/usr/bin/env python3
"""V32 monitor-only IMM motion tracker for Canonical360Tracker vehicles.

Three linear Gaussian motion hypotheses are maintained in a common 6D state:
  [s, s_dot, s_ddot, d, d_dot, d_ddot]

  CV       - acceleration rapidly decays toward zero (steady cruising)
  CA       - longitudinal/lateral acceleration persists (brake/accel)
  MANEUVER - high lateral process noise for lane changes/cut-ins

The tracker is keyed only by canonical Vxxxx IDs. It never publishes radarState,
radarTracks, planner commands, or CAN.  Frenet road coordinates are preferred;
rear/out-of-road-horizon objects fall back to ego-frame x/y as local s/d so the
full 360-degree object set still gets a motion hypothesis.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

import numpy as np

from openpilot.selfdrive.g80_radar.road_geometry import frenet_to_xy, lane_index_from_d, lane_name

IMM_API_VERSION = 2
HORIZONS_S = (0.5, 1.0, 2.0, 3.0)
MODEL_NAMES = ('CV', 'CA', 'MANEUVER')
MODEL_CV, MODEL_CA, MODEL_MAN = 0, 1, 2
TRACK_TTL_NS = int(float(os.getenv('G80_IMM_TRACK_TTL_S', '1.5')) * 1e9)
IMM_HZ = max(1.0, float(os.getenv('G80_IMM_HZ', '5.0')))
IMM_PERIOD_NS = int(1e9 / IMM_HZ)
IMM_FRONT_MAX_M = float(os.getenv('G80_IMM_FRONT_MAX_M', '150.0'))
IMM_REAR_MAX_M = float(os.getenv('G80_IMM_REAR_MAX_M', '80.0'))
IMM_LATERAL_MAX_M = float(os.getenv('G80_IMM_LATERAL_MAX_M', '8.5'))
MAX_DT_S = float(os.getenv('G80_IMM_MAX_DT_S', '0.40'))
MIN_DT_S = 0.01
LANE_HALF_W_M = 1.8

# Row i -> column j transition probability.
TRANSITION = np.array(((0.94, 0.05, 0.01),
                       (0.08, 0.88, 0.04),
                       (0.04, 0.10, 0.86)), dtype=float)
INIT_MU = np.array((0.70, 0.22, 0.08), dtype=float)

# Process spectral densities.  MANEUVER is intentionally agile laterally.
Q_LONG = (0.8, 4.0, 3.0)
Q_LAT = (0.35, 0.55, 6.0)

# Measurement variances: road/ego-frame s, relative longitudinal speed, d.
R_S = float(os.getenv('G80_IMM_R_S', '0.49'))
R_VS = float(os.getenv('G80_IMM_R_VS', '0.81'))
R_D = float(os.getenv('G80_IMM_R_D', '0.25'))
R_DDOT = float(os.getenv('G80_IMM_R_DDOT', '0.36'))


def _finite(v, default=None):
  try:
    x = float(v)
  except Exception:
    return default
  return x if math.isfinite(x) else default


def _q_ca(dt: float, q: float) -> np.ndarray:
  d2, d3, d4, d5 = dt**2, dt**3, dt**4, dt**5
  return q * np.array(((d5/20.0, d4/8.0, d3/6.0),
                       (d4/8.0, d3/3.0, d2/2.0),
                       (d3/6.0, d2/2.0, dt)), dtype=float)


def _block_diag(a: np.ndarray, b: np.ndarray) -> np.ndarray:
  z = np.zeros((6, 6), dtype=float)
  z[:3, :3] = a
  z[3:, 3:] = b
  return z


def _axis_f_long(dt: float, model: int) -> np.ndarray:
  dt2 = dt * dt
  if model == MODEL_CV:
    return np.array(((1.0, dt, 0.0),
                     (0.0, 1.0, 0.0),
                     (0.0, 0.0, 0.05)), dtype=float)
  # CA and MANEUVER can retain longitudinal acceleration.
  return np.array(((1.0, dt, 0.5*dt2),
                   (0.0, 1.0, dt),
                   (0.0, 0.0, 1.0)), dtype=float)


def _axis_f_lat(dt: float, model: int) -> np.ndarray:
  if model == MODEL_MAN:
    dt2=dt*dt
    return np.array(((1.0, dt, 0.5*dt2),
                     (0.0, 1.0, dt),
                     (0.0, 0.0, 1.0)), dtype=float)
  # CV/CA are *lane-keeping* hypotheses laterally.  Lateral velocity decays
  # toward zero instead of being extrapolated forever.  This makes a genuine
  # lane-change onset statistically favor MANEUVER rather than ordinary CV.
  tau=0.40
  decay=math.exp(-max(0.0,dt)/tau)
  integ=tau*(1.0-decay)
  return np.array(((1.0, integ, 0.0),
                   (0.0, decay, 0.0),
                   (0.0, 0.0, 0.05)), dtype=float)


def _f6(dt: float, model: int) -> np.ndarray:
  f = np.zeros((6, 6), dtype=float)
  f[:3, :3] = _axis_f_long(dt, model)
  f[3:, 3:] = _axis_f_lat(dt, model)
  return f


def _q6(dt: float, model: int) -> np.ndarray:
  return _block_diag(_q_ca(dt, Q_LONG[model]), _q_ca(dt, Q_LAT[model]))


def _sym(p: np.ndarray) -> np.ndarray:
  return 0.5 * (p + p.T)


def _gaussian_loglike(y: np.ndarray, s: np.ndarray) -> float:
  try:
    sign, logdet = np.linalg.slogdet(s)
    if sign <= 0:
      return -60.0
    sol = np.linalg.solve(s, y)
    maha = float(y.T @ sol)
    n = len(y)
    return -0.5 * (maha + logdet + n * math.log(2.0 * math.pi))
  except Exception:
    return -60.0


def _measurement(o: dict):
  """Return local road state measurement and the coordinate source.

  V32 also uses the validated KF3 lateral-rate estimate as a soft measurement.
  Position-only IMM likelihoods tended to under-rate a clean lane change because
  the high-Q MANEUVER model naturally has a wider covariance.  d_dot lets the
  model bank distinguish lane keeping (lateral rate decays) from a real maneuver
  without changing Canonical/KF3 authority.
  """
  if o.get('road_projection_valid'):
    s = _finite(o.get('road_s')); d = _finite(o.get('road_d'))
    if s is not None and d is not None:
      return s, d, _finite(o.get('vx')), _finite(o.get('kf_d_dot')), 'c4_path'
  x = _finite(o.get('x')); y = _finite(o.get('y'))
  if x is None or y is None:
    return None
  return x, y, _finite(o.get('vx')), _finite(o.get('kf_vy')), 'ego_xy_fallback'


def _ttlc(d: float, d_dot: float) -> float | None:
  if abs(d_dot) < 0.08:
    return None
  if abs(d) <= LANE_HALF_W_M:
    return 0.0
  if d > LANE_HALF_W_M and d_dot < 0.0:
    t = (LANE_HALF_W_M - d) / d_dot
  elif d < -LANE_HALF_W_M and d_dot > 0.0:
    t = (-LANE_HALF_W_M - d) / d_dot
  else:
    return None
  return t if 0.0 <= t <= 10.0 else None


@dataclass
class ImmTrack:
  key: str
  first_ns: int
  last_filter_ns: int
  last_seen_ns: int
  last_meas_ns: int
  xs: list[np.ndarray] = field(default_factory=list)
  ps: list[np.ndarray] = field(default_factory=list)
  mu: np.ndarray = field(default_factory=lambda: INIT_MU.copy())
  age_frames: int = 0
  coord_source: str = 'ego_xy_fallback'
  reset_count: int = 0
  reinit_count: int = 0
  last_reset_ns: int = 0


class ImmMotionTracker:
  def __init__(self):
    self.tracks: dict[str, ImmTrack] = {}
    self.last_eval_ns = 0
    self.cache: dict[str, dict] = {}

  @staticmethod
  def _key(o: dict) -> str:
    return str(o.get('canonical_key') or o.get('vehicle_key') or o.get('key') or '')

  @staticmethod
  def _init_state(o: dict, meas):
    s, d, vs, vd_meas, source = meas
    # Seed from the already validated KF3 baseline when available.
    s0 = _finite(o.get('kf_s'), s) if source == 'c4_path' else s
    d0 = _finite(o.get('kf_d'), d) if source == 'c4_path' else d
    vs0 = _finite(o.get('kf_s_dot'), vs if vs is not None else 0.0) if source == 'c4_path' else (vs if vs is not None else 0.0)
    as0 = _finite(o.get('kf_s_ddot'), 0.0) if source == 'c4_path' else 0.0
    vd0 = _finite(o.get('kf_d_dot'), 0.0) if source == 'c4_path' else _finite(o.get('kf_vy'), 0.0)
    ad0 = _finite(o.get('kf_d_ddot'), 0.0) if source == 'c4_path' else 0.0
    return np.array((s0, vs0, as0, d0, vd0, ad0), dtype=float), source

  def _new_track(self, key: str, o: dict, meas, now_ns: int) -> ImmTrack:
    x0, source = self._init_state(o, meas)
    p0 = np.diag((1.0, 4.0, 9.0, 0.8, 4.0, 9.0)).astype(float)
    return ImmTrack(key=key, first_ns=now_ns, last_filter_ns=now_ns,
                    last_seen_ns=now_ns, last_meas_ns=0,
                    xs=[x0.copy() for _ in MODEL_NAMES],
                    ps=[p0.copy() for _ in MODEL_NAMES],
                    mu=INIT_MU.copy(), age_frames=0, coord_source=source)

  @staticmethod
  def _mixed_state(t: ImmTrack) -> tuple[np.ndarray, np.ndarray]:
    x = sum(float(t.mu[i]) * t.xs[i] for i in range(3))
    p = np.zeros((6, 6), dtype=float)
    for i in range(3):
      dx = t.xs[i] - x
      p += float(t.mu[i]) * (t.ps[i] + np.outer(dx, dx))
    return x, _sym(p)

  @staticmethod
  def _mix_and_predict(t: ImmTrack, dt: float):
    dt = max(MIN_DT_S, min(MAX_DT_S, float(dt)))
    prior_mu = np.asarray(t.mu, dtype=float)
    c = prior_mu @ TRANSITION
    c = np.maximum(c, 1e-12)
    mix_w = (prior_mu[:, None] * TRANSITION) / c[None, :]
    mixed_x=[]; mixed_p=[]
    for j in range(3):
      xj = sum(float(mix_w[i, j]) * t.xs[i] for i in range(3))
      pj = np.zeros((6, 6), dtype=float)
      for i in range(3):
        dx = t.xs[i] - xj
        pj += float(mix_w[i, j]) * (t.ps[i] + np.outer(dx, dx))
      f = _f6(dt, j)
      mixed_x.append(f @ xj)
      mixed_p.append(_sym(f @ pj @ f.T + _q6(dt, j)))
    t.xs = mixed_x
    t.ps = mixed_p
    # c_j is the Markov-predicted model probability before measurement.
    t.mu = c / float(np.sum(c))

  @staticmethod
  def _update_models(t: ImmTrack, meas):
    s, d, vs, vd_meas, _ = meas
    vals=[s]; rows=[0]; vars_=[R_S]
    if vs is not None:
      vals.append(vs); rows.append(1); vars_.append(R_VS)
    vals.append(d); rows.append(3); vars_.append(R_D)
    if vd_meas is not None:
      vals.append(vd_meas); rows.append(4); vars_.append(R_DDOT)
    z=np.array(vals,dtype=float)
    h=np.zeros((len(rows),6),dtype=float)
    for i,col in enumerate(rows): h[i,col]=1.0
    r=np.diag(vars_).astype(float)

    logs=[]
    eye=np.eye(6)
    for j in range(3):
      x=t.xs[j]; p=t.ps[j]
      y=z - h @ x
      s_mat=_sym(h @ p @ h.T + r)
      logs.append(_gaussian_loglike(y, s_mat))
      try:
        k=p @ h.T @ np.linalg.inv(s_mat)
      except np.linalg.LinAlgError:
        k=p @ h.T @ np.linalg.pinv(s_mat)
      x2=x + k @ y
      ikh=eye-k@h
      p2=_sym(ikh@p@ikh.T + k@r@k.T)
      t.xs[j]=x2; t.ps[j]=p2

    logw=np.log(np.maximum(t.mu,1e-15)) + np.asarray(logs,dtype=float)
    logw -= float(np.max(logw))
    w=np.exp(np.clip(logw,-60.0,0.0))
    sw=float(np.sum(w))
    t.mu = (w/sw) if sw>1e-15 else INIT_MU.copy()

  def _reinitialize(self, t: ImmTrack, o: dict, meas, now_ns: int, reason: str):
    x0,source=self._init_state(o,meas)
    p0=np.diag((1.0,4.0,9.0,0.8,4.0,9.0)).astype(float)
    t.xs=[x0.copy() for _ in MODEL_NAMES]; t.ps=[p0.copy() for _ in MODEL_NAMES]
    t.mu=INIT_MU.copy(); t.coord_source=source
    t.reinit_count+=1; t.last_reset_ns=now_ns
    if reason == 'innovation_reset':
      t.reset_count+=1

  def _maybe_reinitialize(self, t: ImmTrack, o: dict, meas, now_ns: int) -> str:
    mx, _ = self._mixed_state(t)
    s,d,vs,vd_meas,source=meas
    # A Frenet<->ego-frame switch is a coordinate re-anchor, not a tracker
    # failure.  Re-seed from the validated KF3 state but do not count it as a
    # suspicious IMM reset.
    if source != t.coord_source:
      self._reinitialize(t,o,meas,now_ns,'coord_reanchor')
      return 'coord_reanchor'
    # Likewise a Canonical360 reacquisition after a real measurement gap is an
    # expected re-initialisation.  Keeping this separate from innovation resets
    # makes the V32 diagnostics meaningful.
    if bool(o.get('canonical_reacquired')):
      self._reinitialize(t,o,meas,now_ns,'canonical_reacquire')
      return 'canonical_reacquire'
    if abs(s-float(mx[0])) <= 10.0 and abs(d-float(mx[3])) <= 3.5:
      return 'none'
    self._reinitialize(t,o,meas,now_ns,'innovation_reset')
    return 'innovation_reset'

  @staticmethod
  def _predict_model(x: np.ndarray, p: np.ndarray, h: float, model: int):
    f=_f6(max(0.0,float(h)),model)
    q=_q6(max(0.0,float(h)),model)
    return f@x, _sym(f@p@f.T+q)

  def _decorate(self, o: dict, t: ImmTrack, road_model: dict | None, now_ns: int, reinit_reason: str) -> dict:
    dct=dict(o)
    xmix, pmix=self._mixed_state(t)
    mu=np.asarray(t.mu,dtype=float)
    dom=int(np.argmax(mu))
    s,vs,acc,d,vd,ad=map(float,xmix)
    dct.update({
      'imm_valid':True,
      'imm_api_version':IMM_API_VERSION,
      'imm_track_key':f'IMM:{t.key}',
      'imm_age_frames':int(t.age_frames),
      'imm_age_s':round(max(0.0,(now_ns-t.first_ns)/1e9),3),
      'imm_coord_source':t.coord_source,
      'imm_reset_suspect':bool(reinit_reason == 'innovation_reset'),
      'imm_reset_count':int(t.reset_count),
      'imm_reinit_count':int(t.reinit_count),
      'imm_reinit_reason':reinit_reason,
      'imm_interaction_relevant':True,
      'imm_s':round(s,3),'imm_s_dot':round(vs,3),'imm_s_ddot':round(acc,3),
      'imm_d':round(d,3),'imm_d_dot':round(vd,3),'imm_d_ddot':round(ad,3),
      'imm_s_sigma':round(math.sqrt(max(0.0,float(pmix[0,0]))),3),
      'imm_d_sigma':round(math.sqrt(max(0.0,float(pmix[3,3]))),3),
      'imm_d_dot_sigma':round(math.sqrt(max(0.0,float(pmix[4,4]))),3),
      'imm_prob_cv':round(float(mu[MODEL_CV]),4),
      'imm_prob_ca':round(float(mu[MODEL_CA]),4),
      'imm_prob_maneuver':round(float(mu[MODEL_MAN]),4),
      'imm_dominant_model':MODEL_NAMES[dom],
      'imm_lane_index':lane_index_from_d(d),
      'imm_lane':lane_name(lane_index_from_d(d)),
    })
    ttlc=_ttlc(d,vd)
    dct['imm_ttlc_s']=None if ttlc is None else round(float(ttlc),3)

    traj=[]
    for h in HORIZONS_S:
      model_states=[]; model_covs=[]
      for j in range(3):
        xp,pp=self._predict_model(t.xs[j],t.ps[j],h,j)
        model_states.append(xp); model_covs.append(pp)
      xm=sum(float(mu[j])*model_states[j] for j in range(3))
      pm=np.zeros((6,6),dtype=float)
      for j in range(3):
        dx=model_states[j]-xm
        pm += float(mu[j])*(model_covs[j]+np.outer(dx,dx))
      ps,pvs,_,pd,pvd,_=map(float,xm)
      item={'t':h,'s':round(ps,3),'d':round(pd,3),'s_dot':round(pvs,3),'d_dot':round(pvd,3),
            's_sigma':round(math.sqrt(max(0.0,float(pm[0,0]))),3),
            'd_sigma':round(math.sqrt(max(0.0,float(pm[3,3]))),3),
            'lane_index':lane_index_from_d(pd),'lane':lane_name(lane_index_from_d(pd)),
            'model':'IMM'}
      if t.coord_source=='c4_path':
        xy=frenet_to_xy(ps,pd,road_model)
        if xy is not None:
          item['x'],item['y']=round(float(xy[0]),3),round(float(xy[1]),3)
          item['mode']='frenet'
        else:
          item['x'],item['y']=round(ps,3),round(pd,3);item['mode']='ego_fallback'
      else:
        item['x'],item['y']=round(ps,3),round(pd,3);item['mode']='ego_fallback'
      traj.append(item)
    dct['imm_trajectory']=traj

    future_ego=any(p.get('lane')=='ego' for p in traj)
    adjacent=dct['imm_lane'] in ('left1','right1')
    maneuver_prob=float(mu[MODEL_MAN])
    d_sigma=float(dct['imm_d_sigma']); dd_sigma=float(dct['imm_d_dot_sigma'])
    motion_confident=bool(t.age_frames>=4 and d_sigma<=1.0 and dd_sigma<=2.5)
    dct['imm_motion_confident']=motion_confident
    dct['imm_maneuver_candidate']=bool(motion_confident and adjacent and future_ego and maneuver_prob>=0.40 and abs(vd)>=0.20)
    return dct

  @staticmethod
  def _interaction_relevant(o: dict) -> bool:
    # Always keep independently confirmed/control-relevant objects in IMM.
    if o.get('scc_teacher_confirmed') or o.get('camera_confirmed') or o.get('teacher_match') or o.get('rear_teacher_confirmed') or o.get('kf_cutin_candidate'):
      return True
    x=_finite(o.get('x')); y=_finite(o.get('road_d') if o.get('road_projection_valid') else o.get('y'))
    if x is None or y is None:
      return False
    return (-IMM_REAR_MAX_M <= x <= IMM_FRONT_MAX_M) and abs(y) <= IMM_LATERAL_MAX_M

  @staticmethod
  def _copy_cached_imm(o: dict, cached: dict, now_ns: int) -> dict:
    d=dict(o)
    for k,v in cached.items():
      if k.startswith('imm_'):
        d[k]=v
    d['imm_eval_age_ms']=round(max(0.0,(now_ns-int(cached.get('_cache_ns',now_ns)))/1e6),2)
    d['imm_interaction_relevant']=True
    return d

  def _skip(self, o: dict, reason: str) -> dict:
    d=dict(o)
    d['imm_valid']=False
    d['imm_api_version']=IMM_API_VERSION
    d['imm_interaction_relevant']=False
    d['imm_skipped_reason']=reason
    return d

  def update(self, objects: list[dict], road_model: dict | None, now_ns: int, v_ego: float=0.0):
    now_ns=int(now_ns)
    relevant=[]; skipped=[]
    for o in objects:
      (relevant if self._interaction_relevant(o) else skipped).append(o)

    # Full IMM evaluation runs at 5 Hz by default while Canonical360 + KF3 stay
    # at 10 Hz.  New relevant identities force an immediate evaluation.  Cached
    # model probabilities/trajectory are at most ~200 ms old and carry an age.
    new_key=any(self._key(o) and self._key(o) not in self.tracks for o in relevant)
    due=(self.last_eval_ns == 0 or now_ns-self.last_eval_ns >= IMM_PERIOD_NS or new_key)
    if not due:
      out=[]
      relevant_keys=set()
      for o in objects:
        key=self._key(o)
        if self._interaction_relevant(o):
          relevant_keys.add(key)
          c=self.cache.get(key)
          out.append(self._copy_cached_imm(o,c,now_ns) if c is not None else self._skip(o,'awaiting_imm_tick'))
        else:
          out.append(self._skip(o,'out_of_interaction_roi'))
      valid=[o for o in out if o.get('imm_valid')]
      counts={name:sum(1 for o in valid if o.get('imm_dominant_model')==name) for name in MODEL_NAMES}
      means={k:round(sum(float(o.get('imm_prob_'+k,0)) for o in valid)/max(1,len(valid)),4) for k in ('cv','ca','maneuver')}
      return out,{
        'api_version':IMM_API_VERSION,'target_hz':IMM_HZ,'evaluated_this_cycle':False,
        'eval_age_ms':round((now_ns-self.last_eval_ns)/1e6,2),'active_tracks':len(self.tracks),
        'visible_tracks':len(out),'interaction_relevant_tracks':len(relevant),'skipped_tracks':len(skipped),'valid_tracks':len(valid),
        'measurement_updates':0,'reset_suspect_tracks':0,'expected_reinitializations':0,
        'dominant_cv':counts['CV'],'dominant_ca':counts['CA'],'dominant_maneuver':counts['MANEUVER'],
        'maneuver_candidates':sum(1 for o in valid if o.get('imm_maneuver_candidate')),
        'mean_model_probability':means,'models':list(MODEL_NAMES),'horizons_s':list(HORIZONS_S),
        'coordinate_policy':'C4 Frenet when valid; ego x/y fallback for rear/out-of-horizon',
        'interaction_roi_m':{'front':IMM_FRONT_MAX_M,'rear':IMM_REAR_MAX_M,'lateral_abs':IMM_LATERAL_MAX_M},
        'control_connected':False,
      }

    self.last_eval_ns=now_ns
    # Predict only interaction-relevant live tracks to the common IMM tick.
    relevant_keys={self._key(o) for o in relevant if self._key(o)}
    for key,t in list(self.tracks.items()):
      if key not in relevant_keys:
        continue
      dt=(now_ns-t.last_filter_ns)/1e9
      if dt>=MIN_DT_S:
        self._mix_and_predict(t,dt)
        t.last_filter_ns=now_ns

    decorated_by_key={}; updates=0; suspect_resets=0; expected_reinits=0
    for o in relevant:
      key=self._key(o)
      meas=_measurement(o)
      if not key or meas is None:
        decorated_by_key[key]=self._skip(o,'no_measurement'); continue
      t=self.tracks.get(key)
      if t is None:
        t=self._new_track(key,o,meas,now_ns); self.tracks[key]=t
      reason=self._maybe_reinitialize(t,o,meas,now_ns)
      if reason == 'innovation_reset': suspect_resets+=1
      elif reason != 'none': expected_reinits+=1
      meas_ns=int(o.get('recv_ns',now_ns) or now_ns)
      if meas_ns>t.last_meas_ns:
        self._update_models(t,meas)
        t.last_meas_ns=meas_ns; t.age_frames+=1; updates+=1
      t.last_seen_ns=now_ns
      t.coord_source=meas[4]
      d=self._decorate(o,t,road_model,now_ns,reason)
      d['imm_eval_age_ms']=0.0
      decorated_by_key[key]=d
      cached={k:v for k,v in d.items() if k.startswith('imm_')}
      cached['_cache_ns']=now_ns
      self.cache[key]=cached

    stale=[k for k,t in self.tracks.items() if now_ns-t.last_seen_ns>TRACK_TTL_NS]
    for k in stale:
      self.tracks.pop(k,None); self.cache.pop(k,None)

    out=[]
    for o in objects:
      key=self._key(o)
      if self._interaction_relevant(o):
        out.append(decorated_by_key.get(key,self._skip(o,'no_imm_result')))
      else:
        out.append(self._skip(o,'out_of_interaction_roi'))

    valid=[o for o in out if o.get('imm_valid')]
    counts={name:sum(1 for o in valid if o.get('imm_dominant_model')==name) for name in MODEL_NAMES}
    means={
      'cv':round(sum(float(o.get('imm_prob_cv',0)) for o in valid)/max(1,len(valid)),4),
      'ca':round(sum(float(o.get('imm_prob_ca',0)) for o in valid)/max(1,len(valid)),4),
      'maneuver':round(sum(float(o.get('imm_prob_maneuver',0)) for o in valid)/max(1,len(valid)),4),
    }
    return out,{
      'api_version':IMM_API_VERSION,'target_hz':IMM_HZ,'evaluated_this_cycle':True,'eval_age_ms':0.0,
      'active_tracks':len(self.tracks),'visible_tracks':len(out),'interaction_relevant_tracks':len(relevant),'skipped_tracks':len(skipped),'valid_tracks':len(valid),
      'measurement_updates':updates,'reset_suspect_tracks':suspect_resets,'expected_reinitializations':expected_reinits,
      'dominant_cv':counts['CV'],'dominant_ca':counts['CA'],'dominant_maneuver':counts['MANEUVER'],
      'maneuver_candidates':sum(1 for o in valid if o.get('imm_maneuver_candidate')),
      'mean_model_probability':means,
      'models':list(MODEL_NAMES),'horizons_s':list(HORIZONS_S),
      'coordinate_policy':'C4 Frenet when valid; ego x/y fallback for rear/out-of-horizon',
      'interaction_roi_m':{'front':IMM_FRONT_MAX_M,'rear':IMM_REAR_MAX_M,'lateral_abs':IMM_LATERAL_MAX_M},
      'control_connected':False,
    }


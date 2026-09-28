#!/usr/bin/env python3
"""V34 monitor-only motion Kalman tracker driven by Canonical360Tracker IDs.

This module does NOT publish radarTracks/radarState and does not send CAN.
It augments already fused physical-vehicle dictionaries with smoothed motion
state and short-horizon trajectory predictions for validation.

CA state filters are maintained per persistent vehicle, but V30 retains the V29 stabilization and deliberately
uses asymmetric prediction models:
  longitudinal s/x: constant acceleration (CA)
  lateral d/y:      CA for the first 0.5 s, then bounded constant velocity (CV)

This prevents noisy lateral acceleration from exploding 2-3 s predictions.
Frenet is updated only when C4 road projection is valid. Rear objects and
points outside the C4 path horizon keep Cartesian prediction only.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

import numpy as np

from openpilot.selfdrive.g80_radar.road_geometry import frenet_to_xy, lane_index_from_d, lane_name
from openpilot.selfdrive.g80_radar.build_info import KALMAN_API_VERSION


HORIZONS_S = (0.5, 1.0, 2.0, 3.0)
TRACK_TTL_NS = int(float(os.getenv('G80_KF_TRACK_TTL_S', '1.5')) * 1e9)
MAX_DT_S = 0.35
MIN_DT_S = 0.01
LANE_HALF_W_M = 1.8

# White-jerk process spectral densities. Lateral is deliberately more agile so
# an emerging cut-in is not over-smoothed.
Q_LONG = float(os.getenv('G80_KF_Q_LONG', '5.0'))
Q_LAT = float(os.getenv('G80_KF_Q_LAT', '10.0'))
R_X = float(os.getenv('G80_KF_R_X', '0.36'))       # variance, ~0.6 m sigma
R_Y = float(os.getenv('G80_KF_R_Y', '0.25'))       # variance, ~0.5 m sigma
R_VX = float(os.getenv('G80_KF_R_VX', '0.64'))     # variance, ~0.8 m/s sigma
R_S = float(os.getenv('G80_KF_R_S', '0.49'))       # projection adds some noise
R_D = float(os.getenv('G80_KF_R_D', '0.20'))       # ~0.45 m sigma
R_DS = float(os.getenv('G80_KF_R_DS', '1.00'))

# V29 lateral prediction stabilization. Filter state is left untouched for
# diagnostics; only future trajectory extrapolation is bounded/damped.
LATERAL_CA_HORIZON_S = float(os.getenv('G80_KF_LAT_CA_HORIZON_S', '0.5'))
LATERAL_PRED_MAX_SPEED_MPS = float(os.getenv('G80_KF_LAT_PRED_MAX_SPEED_MPS', '2.0'))
LATERAL_PRED_MAX_ACCEL_MPS2 = float(os.getenv('G80_KF_LAT_PRED_MAX_ACCEL_MPS2', '1.5'))
HIGHWAY_CUTIN_MIN_VEGO_MPS = float(os.getenv('G80_KF_HIGHWAY_CUTIN_MIN_VEGO_MPS', '5.0'))
KF_MAX_TRACKS = max(4, int(os.getenv('G80_KF_MAX_TRACKS', '14')))
KF_FRONT_MAX_M = float(os.getenv('G80_KF_FRONT_MAX_M', '150.0'))
KF_REAR_MAX_M = float(os.getenv('G80_KF_REAR_MAX_M', '90.0'))
KF_LATERAL_MAX_M = float(os.getenv('G80_KF_LATERAL_MAX_M', '9.0'))


def _finite(v, default=None):
  try:
    x = float(v)
  except Exception:
    return default
  return x if math.isfinite(x) else default


def _F(dt: float) -> np.ndarray:
  dt2 = dt * dt
  return np.array(((1.0, dt, 0.5 * dt2),
                   (0.0, 1.0, dt),
                   (0.0, 0.0, 1.0)), dtype=float)


def _Q(dt: float, q: float) -> np.ndarray:
  # Continuous white jerk -> discrete CA process covariance.
  d2, d3, d4, d5 = dt**2, dt**3, dt**4, dt**5
  return q * np.array(((d5 / 20.0, d4 / 8.0, d3 / 6.0),
                       (d4 / 8.0, d3 / 3.0, d2 / 2.0),
                       (d3 / 6.0, d2 / 2.0, dt)), dtype=float)


class AxisCAKalman:
  def __init__(self, pos: float, vel: float = 0.0, q: float = 5.0):
    self.x = np.array([float(pos), float(vel), 0.0], dtype=float)
    self.P = np.diag([1.0, 9.0, 16.0]).astype(float)
    self.q = float(q)

  def predict(self, dt: float):
    dt = max(MIN_DT_S, min(MAX_DT_S, float(dt)))
    f = _F(dt)
    self.x = f @ self.x
    self.P = f @ self.P @ f.T + _Q(dt, self.q)
    self.P = 0.5 * (self.P + self.P.T)

  def update_scalar(self, z: float, index: int, r: float):
    if not math.isfinite(float(z)):
      return
    h = np.zeros(3, dtype=float)
    h[int(index)] = 1.0
    innovation = float(z) - float(h @ self.x)
    s = float(h @ self.P @ h.T + max(float(r), 1e-6))
    if not math.isfinite(s) or s <= 1e-9:
      return
    k = (self.P @ h.T) / s
    self.x = self.x + k * innovation
    i_kh = np.eye(3) - np.outer(k, h)
    # Joseph form keeps covariance PSD under repeated updates.
    self.P = i_kh @ self.P @ i_kh.T + np.outer(k, k) * max(float(r), 1e-6)
    self.P = 0.5 * (self.P + self.P.T)

  def predicted(self, t: float) -> tuple[float, float, float]:
    f = _F(max(0.0, float(t)))
    q = f @ self.x
    return float(q[0]), float(q[1]), float(q[2])

  def sigma(self, index: int) -> float:
    return math.sqrt(max(0.0, float(self.P[int(index), int(index)])))


@dataclass
class MotionTrack:
  key: str
  first_ns: int
  last_filter_ns: int
  last_seen_ns: int
  last_meas_ns: int
  age_frames: int = 0
  xk: AxisCAKalman | None = None
  yk: AxisCAKalman | None = None
  sk: AxisCAKalman | None = None
  dk: AxisCAKalman | None = None
  frenet_last_update_ns: int = 0
  source_keys: set[str] = field(default_factory=set)
  cutin_candidate_since_ns: int = 0
  cutin_last_candidate_ns: int = 0


class KalmanMotionTracker:
  """Persistent CA Kalman tracker keyed by canonical physical vehicle ID."""
  def __init__(self, prefix: str = 'KF'):
    self.prefix = str(prefix)
    self.tracks: dict[str, MotionTrack] = {}

  @staticmethod
  def _key(o: dict) -> str:
    return str(o.get('vehicle_key') or o.get('key') or '')

  @staticmethod
  def _source_keys(o: dict) -> set[str]:
    vals = o.get('vehicle_cluster_keys')
    if isinstance(vals, list):
      return {str(v) for v in vals if v is not None}
    k = o.get('key')
    return {str(k)} if k is not None else set()

  def _new_track(self, key: str, o: dict, now_ns: int) -> MotionTrack:
    x = _finite(o.get('x'), 0.0)
    y = _finite(o.get('y'), 0.0)
    vx = _finite(o.get('vx'), 0.0)
    t = MotionTrack(key=key, first_ns=int(now_ns), last_filter_ns=int(now_ns),
                    last_seen_ns=int(now_ns), last_meas_ns=0, age_frames=0,
                    xk=AxisCAKalman(x, vx, Q_LONG), yk=AxisCAKalman(y, 0.0, Q_LAT))
    s = _finite(o.get('road_s')) if o.get('road_projection_valid') else None
    d = _finite(o.get('road_d')) if o.get('road_projection_valid') else None
    if s is not None and d is not None:
      t.sk = AxisCAKalman(s, vx, Q_LONG)
      t.dk = AxisCAKalman(d, 0.0, Q_LAT)
      t.frenet_last_update_ns = int(now_ns)
    t.source_keys = self._source_keys(o)
    return t

  @staticmethod
  def _reinit_cartesian_if_needed(t: MotionTrack, o: dict):
    mx, my = _finite(o.get('x')), _finite(o.get('y'))
    if mx is None or my is None or t.xk is None or t.yk is None:
      return
    if abs(mx - float(t.xk.x[0])) > 12.0 or abs(my - float(t.yk.x[0])) > 4.5:
      vx = _finite(o.get('vx'), 0.0)
      t.xk = AxisCAKalman(mx, vx, Q_LONG)
      t.yk = AxisCAKalman(my, 0.0, Q_LAT)

  @staticmethod
  def _update_frenet(t: MotionTrack, o: dict, now_ns: int):
    if not o.get('road_projection_valid'):
      return
    ms, md = _finite(o.get('road_s')), _finite(o.get('road_d'))
    if ms is None or md is None:
      return
    vx = _finite(o.get('vx'))
    # Reinitialize after a long road-model gap or a large path-coordinate jump.
    stale = t.frenet_last_update_ns == 0 or int(now_ns) - int(t.frenet_last_update_ns) > 700_000_000
    jump = t.sk is not None and t.dk is not None and (abs(ms - float(t.sk.x[0])) > 12.0 or abs(md - float(t.dk.x[0])) > 4.0)
    if t.sk is None or t.dk is None or stale or jump:
      t.sk = AxisCAKalman(ms, 0.0 if vx is None else vx, Q_LONG)
      t.dk = AxisCAKalman(md, 0.0, Q_LAT)
    else:
      t.sk.update_scalar(ms, 0, R_S)
      if vx is not None:
        t.sk.update_scalar(vx, 1, R_DS)
      t.dk.update_scalar(md, 0, R_D)
    t.frenet_last_update_ns = int(now_ns)

  @staticmethod
  def _ttlc(d: float | None, d_dot: float | None) -> float | None:
    if d is None or d_dot is None or abs(d_dot) < 0.08:
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

  @staticmethod
  def _predict_lateral(axis: AxisCAKalman, horizon_s: float) -> tuple[float, float, float, bool]:
    """V29 short-CA then bounded-CV lateral prediction.

    The KF state itself remains unmodified. This only constrains trajectory
    extrapolation used for display/cut-in validation.
    """
    h = max(0.0, float(horizon_s))
    d0, v0, a0 = map(float, axis.x)
    v = max(-LATERAL_PRED_MAX_SPEED_MPS, min(LATERAL_PRED_MAX_SPEED_MPS, v0))
    a = max(-LATERAL_PRED_MAX_ACCEL_MPS2, min(LATERAL_PRED_MAX_ACCEL_MPS2, a0))
    limited = abs(v-v0) > 1e-9 or abs(a-a0) > 1e-9
    ca_h = min(h, max(0.0, LATERAL_CA_HORIZON_S))
    d = d0 + v * ca_h + 0.5 * a * ca_h * ca_h
    v_after = v + a * ca_h
    v_limited = max(-LATERAL_PRED_MAX_SPEED_MPS, min(LATERAL_PRED_MAX_SPEED_MPS, v_after))
    limited = limited or abs(v_limited-v_after) > 1e-9
    if h > ca_h:
      d += v_limited * (h - ca_h)
      return float(d), float(v_limited), 0.0, bool(limited)
    return float(d), float(v_after), float(a), bool(limited)

  def _decorate(self, o: dict, t: MotionTrack, road_model: dict | None, now_ns: int, v_ego: float = 0.0) -> dict:
    dct = dict(o)
    assert t.xk is not None and t.yk is not None
    x, vx, ax = map(float, t.xk.x)
    y, vy, ay = map(float, t.yk.x)
    kalman_age_s = (int(now_ns) - int(t.first_ns)) / 1e9
    canonical_key = str(dct.get('canonical_key') or dct.get('vehicle_key') or '')
    canonical_age_s = _finite(dct.get('canonical_track_duration_s'))
    dct.update({
      'kalman_valid': True,
      'kalman_track_key': f'{self.prefix}:{t.key}',
      'kalman_age_frames': int(t.age_frames),
      'kalman_age_s': round(kalman_age_s, 3),
      'kf_canonical_key_match': bool(not canonical_key or canonical_key == t.key),
      'kf_reset_suspect': bool(canonical_age_s is not None and canonical_age_s > 1.0 and kalman_age_s + 0.5 < canonical_age_s),
      'kf_x': round(x, 3), 'kf_y': round(y, 3),
      'kf_vx': round(vx, 3), 'kf_vy': round(vy, 3),
      'kf_ax': round(ax, 3), 'kf_ay': round(ay, 3),
      'kf_x_sigma': round(t.xk.sigma(0), 3), 'kf_y_sigma': round(t.yk.sigma(0), 3),
      'kf_vx_sigma': round(t.xk.sigma(1), 3), 'kf_vy_sigma': round(t.yk.sigma(1), 3),
    })

    frenet_valid = t.sk is not None and t.dk is not None and int(now_ns) - int(t.frenet_last_update_ns) <= 700_000_000
    dct['kf_frenet_valid'] = bool(frenet_valid)
    cur_s = cur_d = cur_ds = cur_dd = None
    if frenet_valid:
      cur_s, cur_ds, cur_dds = map(float, t.sk.x)
      cur_d, cur_dd, cur_ddd = map(float, t.dk.x)
      dct.update({
        'kf_s': round(cur_s, 3), 'kf_s_dot': round(cur_ds, 3), 'kf_s_ddot': round(cur_dds, 3),
        'kf_d': round(cur_d, 3), 'kf_d_dot': round(cur_dd, 3), 'kf_d_ddot': round(cur_ddd, 3),
        'kf_s_sigma': round(t.sk.sigma(0), 3), 'kf_d_sigma': round(t.dk.sigma(0), 3),
        'kf_s_dot_sigma': round(t.sk.sigma(1), 3), 'kf_d_dot_sigma': round(t.dk.sigma(1), 3),
        'kf_lane_index': lane_index_from_d(cur_d),
        'kf_lane': lane_name(lane_index_from_d(cur_d)),
      })
      ttlc = self._ttlc(cur_d, cur_dd)
      dct['kf_ttlc_s'] = None if ttlc is None else round(ttlc, 3)
      if abs(cur_dd) < 0.25:
        dct['kf_lateral_motion'] = 'STABLE'
      else:
        dct['kf_lateral_motion'] = 'LEFT' if cur_dd > 0.0 else 'RIGHT'
    else:
      dct['kf_lateral_motion'] = 'LEFT' if vy > 0.35 else ('RIGHT' if vy < -0.35 else 'STABLE')
      dct['kf_ttlc_s'] = None

    traj = []
    for h in HORIZONS_S:
      cx, cvx, _ = t.xk.predicted(h)
      cy, cvy, _, y_limited = self._predict_lateral(t.yk, h)
      item = {'t': h, 'x': round(cx, 3), 'y': round(cy, 3), 'mode': 'cartesian',
              'lateral_prediction':'ca0.5_cv', 'lateral_limited': bool(y_limited)}
      if frenet_valid:
        ps, pds, _ = t.sk.predicted(h)
        pd, pdd, _, d_limited = self._predict_lateral(t.dk, h)
        item.update({'s': round(ps, 3), 'd': round(pd, 3), 's_dot': round(pds, 3), 'd_dot': round(pdd, 3),
                     'lateral_limited': bool(d_limited),
                     'lane_index': lane_index_from_d(pd), 'lane': lane_name(lane_index_from_d(pd))})
        xy = frenet_to_xy(ps, pd, road_model)
        if xy is not None:
          item['x'], item['y'] = round(float(xy[0]), 3), round(float(xy[1]), 3)
          item['mode'] = 'frenet'
      traj.append(item)
    dct['kalman_trajectory'] = traj
    dct['kf_lateral_prediction_mode'] = 'ca0.5_cv'
    dct['kf_lateral_prediction_limited'] = any(bool(p.get('lateral_limited')) for p in traj)

    lane_now = dct.get('kf_lane')
    future_ego = any(p.get('lane') == 'ego' for p in traj if p.get('lane') is not None)
    ttlc = _finite(dct.get('kf_ttlc_s'))
    d_dot = _finite(dct.get('kf_d_dot'))
    d_sig = _finite(dct.get('kf_d_sigma'), 99.0)
    dv_sig = _finite(dct.get('kf_d_dot_sigma'), 99.0)

    # V24 tuning from V23 road logs:
    # - keep a broad motion-confidence flag for diagnostics,
    # - only promote an adjacent-lane inward motion to a cut-in candidate,
    # - require lower uncertainty and temporal persistence before CONFIRMED.
    motion_confident = bool(
      frenet_valid and t.age_frames >= 4 and d_sig <= 1.0 and dv_sig <= 2.5
      and d_dot is not None and abs(d_dot) <= 3.0
    )
    adjacent_lane = lane_now in ('left1', 'right1')
    base_lateral_candidate = bool(
      motion_confident and adjacent_lane and future_ego and ttlc is not None
      and 0.20 <= float(ttlc) <= 3.0
      and d_sig <= 0.70 and dv_sig <= 1.80
      and d_dot is not None and 0.30 <= abs(d_dot) <= 2.50
      and (int(now_ns) - int(t.first_ns)) >= 800_000_000
    )
    v_ego_f = max(0.0, _finite(v_ego, 0.0) or 0.0)
    strict_candidate = bool(base_lateral_candidate and v_ego_f >= HIGHWAY_CUTIN_MIN_VEGO_MPS)
    low_speed_candidate = bool(base_lateral_candidate and v_ego_f < HIGHWAY_CUTIN_MIN_VEGO_MPS)

    if strict_candidate:
      if t.cutin_candidate_since_ns == 0 or (t.cutin_last_candidate_ns and int(now_ns) - int(t.cutin_last_candidate_ns) > 350_000_000):
        t.cutin_candidate_since_ns = int(now_ns)
      t.cutin_last_candidate_ns = int(now_ns)
    elif t.cutin_last_candidate_ns == 0 or int(now_ns) - int(t.cutin_last_candidate_ns) > 200_000_000:
      t.cutin_candidate_since_ns = 0

    persistence_s = 0.0
    if strict_candidate and t.cutin_candidate_since_ns:
      persistence_s = max(0.0, (int(now_ns) - int(t.cutin_candidate_since_ns)) / 1e9)
    confirmed = bool(strict_candidate and persistence_s >= 0.25)

    # 0..1 diagnostic score, not a control probability.
    score = 0.0
    if strict_candidate:
      ttlc_term = max(0.0, min(1.0, (3.0 - float(ttlc)) / 2.8))
      sig_term = max(0.0, min(1.0, 1.0 - dv_sig / 1.8))
      persist_term = max(0.0, min(1.0, persistence_s / 0.5))
      score = 0.45 * ttlc_term + 0.25 * sig_term + 0.30 * persist_term

    dct['kf_motion_confident'] = motion_confident
    dct['kf_lateral_candidate'] = base_lateral_candidate
    dct['kf_low_speed_lateral_candidate'] = low_speed_candidate
    dct['kf_cutin_speed_class'] = 'HIGHWAY' if strict_candidate else ('LOW_SPEED' if low_speed_candidate else 'NONE')
    dct['kf_cutin_candidate'] = strict_candidate
    dct['kf_cutin_confirmed'] = confirmed
    dct['kf_cutin_persistence_s'] = round(persistence_s, 3)
    dct['kf_cutin_score'] = round(score, 3)
    return dct

  @staticmethod
  def _priority(o: dict) -> tuple:
    x=_finite(o.get('x'), 999.0)
    d=_finite(o.get('road_d') if o.get('road_projection_valid') else o.get('y'), 99.0)
    evidence=0
    if o.get('scc_teacher_confirmed'): evidence-=80
    if o.get('camera_confirmed'): evidence-=45
    if o.get('rear_teacher_confirmed') or o.get('teacher_match'): evidence-=45
    if o.get('kf_cutin_candidate') or o.get('kf_lateral_candidate'): evidence-=25
    lane_pen=max(0.0,abs(d)-5.5)*8.0
    rear_pen=0.0 if x>=0 else 0.15*abs(x)
    return (evidence + abs(x) + lane_pen + rear_pen, abs(d), abs(x))

  @classmethod
  def _prediction_relevant(cls, o: dict) -> bool:
    if o.get('scc_teacher_confirmed') or o.get('camera_confirmed') or o.get('rear_teacher_confirmed') or o.get('teacher_match'):
      return True
    x=_finite(o.get('x')); d=_finite(o.get('road_d') if o.get('road_projection_valid') else o.get('y'))
    return x is not None and d is not None and -KF_REAR_MAX_M <= x <= KF_FRONT_MAX_M and abs(d) <= KF_LATERAL_MAX_M

  def update(self, objects: list[dict], road_model: dict | None, now_ns: int, v_ego: float = 0.0) -> tuple[list[dict], dict]:
    now_ns = int(now_ns)

    candidates=[o for o in objects if self._key(o) and self._prediction_relevant(o)]
    candidates.sort(key=self._priority)
    selected_keys={self._key(o) for o in candidates[:KF_MAX_TRACKS]}

    # Predict only selected interaction tracks. Canonical360 identity remains full
    # coverage; dense/far objects can fall back to raw/KF-free geometry.
    for key,t in list(self.tracks.items()):
      if key not in selected_keys:
        continue
      dt = (now_ns - int(t.last_filter_ns)) / 1e9
      if dt >= MIN_DT_S:
        if t.xk is not None: t.xk.predict(dt)
        if t.yk is not None: t.yk.predict(dt)
        if t.sk is not None: t.sk.predict(dt)
        if t.dk is not None: t.dk.predict(dt)
        t.last_filter_ns = now_ns

    out = []
    updates = 0
    frenet_updates = 0
    ranked_out=0
    for o in objects:
      key = self._key(o)
      if not key:
        out.append(dict(o))
        continue
      if key not in selected_keys:
        d=dict(o)
        d['kalman_valid']=False
        d['kalman_api_version']=KALMAN_API_VERSION
        d['kf_skipped_reason']='ranked_out_or_outside_roi'
        out.append(d)
        ranked_out+=1
        continue

      t = self.tracks.get(key)
      if t is None:
        t = self._new_track(key, o, now_ns)
        self.tracks[key] = t

      self._reinit_cartesian_if_needed(t, o)
      meas_ns = int(o.get('recv_ns', now_ns) or now_ns)
      is_new_measurement = meas_ns > int(t.last_meas_ns)
      if is_new_measurement:
        mx, my = _finite(o.get('x')), _finite(o.get('y'))
        mvx = _finite(o.get('vx'))
        if mx is not None and t.xk is not None:
          t.xk.update_scalar(mx, 0, R_X)
          if mvx is not None:
            t.xk.update_scalar(mvx, 1, R_VX)
        if my is not None and t.yk is not None:
          t.yk.update_scalar(my, 0, R_Y)
        before = t.frenet_last_update_ns
        self._update_frenet(t, o, now_ns)
        if t.frenet_last_update_ns != before:
          frenet_updates += 1
        t.last_meas_ns = meas_ns
        t.age_frames += 1
        t.source_keys |= self._source_keys(o)
        updates += 1
      t.last_seen_ns = now_ns
      out.append(self._decorate(o, t, road_model, now_ns, v_ego))

    stale = [k for k, t in self.tracks.items() if now_ns - int(t.last_seen_ns) > TRACK_TTL_NS]
    for k in stale:
      self.tracks.pop(k, None)

    cutins = sum(1 for o in out if o.get('kf_cutin_candidate'))
    confirmed_cutins = sum(1 for o in out if o.get('kf_cutin_confirmed'))
    low_speed_lateral = sum(1 for o in out if o.get('kf_low_speed_lateral_candidate'))
    prediction_limited = sum(1 for o in out if o.get('kf_lateral_prediction_limited'))
    frenet_valid = sum(1 for o in out if o.get('kf_frenet_valid'))
    confident = sum(1 for o in out if o.get('kf_motion_confident'))
    lateral_unstable = sum(1 for o in out if o.get('kf_d_dot') is not None and abs(float(o.get('kf_d_dot'))) > 3.0)
    canonical_key_mismatch = sum(1 for o in out if o.get('canonical_valid') and o.get('kalman_valid') and not o.get('kf_canonical_key_match', False))
    reset_suspects = sum(1 for o in out if o.get('kf_reset_suspect'))
    return out, {
      'active_tracks': len(self.tracks),
      'visible_tracks': len(out),
      'selected_tracks': len(selected_keys),
      'candidate_tracks_before_cap': len(candidates),
      'ranked_out_tracks': ranked_out,
      'max_tracks': KF_MAX_TRACKS,
      'measurement_updates': updates,
      'frenet_updates': frenet_updates,
      'frenet_valid_tracks': frenet_valid,
      'motion_confident_tracks': confident,
      'lateral_unstable_tracks': lateral_unstable,
      'canonical_key_mismatch_tracks': canonical_key_mismatch,
      'kf_reset_suspect_tracks': reset_suspects,
      'cutin_candidates': cutins,
      'cutin_confirmed': confirmed_cutins,
      'low_speed_lateral_candidates': low_speed_lateral,
      'lateral_prediction_limited_tracks': prediction_limited,
      'cutin_rule': 'vEgo>=5m/s + adjacent lane + TTLC 0.2..3.0s + low covariance + >=0.25s persistence',
      'lateral_prediction_model': 'CA <=0.5s then bounded CV',
      'model': 'selective CA cartesian+Frenet; Canonical360 remains full coverage',
      'roi_m': {'front':KF_FRONT_MAX_M,'rear':KF_REAR_MAX_M,'lateral_abs':KF_LATERAL_MAX_M},
      'horizons_s': list(HORIZONS_S),
      'control_connected': False,
    }

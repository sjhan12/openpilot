#!/usr/bin/env python3
"""Monitor-only E2E traffic-signal probe for the G80 browser/HUD.

V39 semantics
-------------
openpilot/sunnypilot ModelDataV2 does not publish a direct red/green/yellow/
arrow lamp classifier.  The old V38 probe therefore over-labelled E2E stop
behavior as RED, which can also happen at stop signs, blocked paths, parking
exits, and other non-signal stops.

V39 makes the distinction explicit:
  * STOP_HOLD      : E2E says "stay stopped"; traffic-light color unknown.
  * RED_CANDIDATE  : conservative long/short-path stop candidate, still "RED ?".
  * GREEN_GO       : a stop->go edge was observed from path opening / shouldStop
                     release / sunnypilot green alert.  This is an E2E go-edge,
                     not a direct green-lamp pixel classification.

The fast-green edge deliberately watches the path horizon jump while stopped.
In V38 logs the path frequently opened one or two samples before shouldStop
cleared, so this can display GO earlier without pretending to see lamp pixels.

This process is diagnostic only.  It never publishes controls and never sends CAN.
"""
from __future__ import annotations

import math

PROBE_VERSION = 2
MODEL_MAX_AGE_NS = 700_000_000
AUX_MAX_AGE_NS = 1_200_000_000
STOPPED_MAX_MPS = 0.35

APPROACH_SPEED_MPS = 1.0
APPROACH_MEMORY_S = 10.0
STOP_ARM_WINDOW_S = 2.5
STOP_ARM_PATH_M = 20.0
STOP_REARM_PATH_M = 15.0

# Conservative red candidate: do not claim RED immediately from shouldStop.
RED_CANDIDATE_HOLD_S = 6.0
RED_CANDIDATE_PATH_MAX_M = 10.0

# Stop -> go edge.  These are E2E path/action thresholds, not lamp probabilities.
GREEN_NORMAL_PATH_M = 26.0
GREEN_FAST_PATH_M = 28.0
GREEN_FAST_TOTAL_JUMP_M = 18.0
GREEN_FAST_FRAME_JUMP_M = 5.0
GREEN_CONFIRM_S = 0.08
GREEN_LATCH_S = 1.8


def _clamp(v: float, lo: float = 0.0, hi: float = 1.0) -> float:
  return max(lo, min(hi, float(v)))


def _finite(v, default=0.0) -> float:
  try:
    f = float(v)
  except Exception:
    return float(default)
  return f if math.isfinite(f) else float(default)


def _enum_text(v) -> str:
  try:
    s = str(v)
  except Exception:
    return ''
  if '.' in s:
    s = s.split('.')[-1]
  return s.strip()


class TrafficSignalProbe:
  def __init__(self):
    self.model_recv_ns = 0
    self.path_horizon_m = 0.0
    self.prev_path_horizon_m = 0.0
    self.horizon_delta_m = 0.0
    self.should_stop = False
    self.desired_accel_mps2 = 0.0
    self.model_confidence = 'unknown'

    self.car_recv_ns = 0
    self.v_ego_mps = 0.0
    self.standstill = False
    self.gas_pressed = False
    self.last_moving_ns = 0
    self.last_approach_ns = 0
    self.stop_since_ns = 0

    self.turn_recv_ns = 0
    self.turn_direction = 'none'

    self.plan_recv_ns = 0
    self.green_alert = False

    self.stop_armed = False
    self.stop_armed_since_ns = 0
    self.stop_min_horizon_m = math.inf
    self.green_candidate_since_ns = 0
    self.fast_edge_until_ns = 0
    self.green_latch_until_ns = 0
    self.green_trigger_source = 'none'
    self.last_state = 'UNKNOWN'

  def _clear_stop_context(self) -> None:
    self.stop_armed = False
    self.stop_armed_since_ns = 0
    self.stop_min_horizon_m = math.inf
    self.green_candidate_since_ns = 0
    self.fast_edge_until_ns = 0
    self.green_latch_until_ns = 0
    self.green_trigger_source = 'none'

  def update_model(self, model, recv_ns: int) -> None:
    self.model_recv_ns = int(recv_ns)
    old_horizon = self.path_horizon_m
    try:
      xs = list(model.position.x)
      vals = [_finite(x, math.nan) for x in xs]
      vals = [x for x in vals if math.isfinite(x)]
      new_horizon = max(vals) if vals else 0.0
    except Exception:
      new_horizon = 0.0
    self.prev_path_horizon_m = old_horizon
    self.path_horizon_m = new_horizon
    self.horizon_delta_m = new_horizon - old_horizon
    try:
      self.should_stop = bool(model.action.shouldStop)
    except Exception:
      self.should_stop = False
    try:
      self.desired_accel_mps2 = _finite(model.action.desiredAcceleration)
    except Exception:
      self.desired_accel_mps2 = 0.0
    try:
      self.model_confidence = _enum_text(model.confidence) or 'unknown'
    except Exception:
      self.model_confidence = 'unknown'

  def update_carstate(self, cs, recv_ns: int) -> None:
    self.car_recv_ns = int(recv_ns)
    self.v_ego_mps = _finite(getattr(cs, 'vEgo', 0.0))
    try:
      self.standstill = bool(cs.standstill)
    except Exception:
      self.standstill = abs(self.v_ego_mps) <= STOPPED_MAX_MPS
    try:
      self.gas_pressed = bool(cs.gasPressed)
    except Exception:
      self.gas_pressed = False

    moving = (not self.standstill) and self.v_ego_mps > 0.1
    if moving:
      self.last_moving_ns = int(recv_ns)
      self.stop_since_ns = 0
    elif self.stop_since_ns == 0:
      self.stop_since_ns = int(recv_ns)

    if self.v_ego_mps > APPROACH_SPEED_MPS:
      self.last_approach_ns = int(recv_ns)

    # Once the car clearly leaves the stop, start a fresh candidate next time.
    if self.v_ego_mps > 0.8 or self.gas_pressed:
      self._clear_stop_context()

  def update_turn(self, msg, recv_ns: int) -> None:
    try:
      d = _enum_text(msg.modelDataV2SP.laneTurnDirection)
    except Exception:
      try:
        d = _enum_text(msg.laneTurnDirection)
      except Exception:
        return
    dl = d.lower()
    if 'left' in dl:
      self.turn_direction = 'left'
    elif 'right' in dl:
      self.turn_direction = 'right'
    else:
      self.turn_direction = 'none'
    self.turn_recv_ns = int(recv_ns)

  def update_longitudinal_plan_sp(self, msg, recv_ns: int) -> None:
    try:
      self.green_alert = bool(msg.longitudinalPlanSP.e2eAlerts.greenLightAlert)
    except Exception:
      try:
        self.green_alert = bool(msg.e2eAlerts.greenLightAlert)
      except Exception:
        return
    self.plan_recv_ns = int(recv_ns)

  def snapshot(self, now_ns: int, has_lead: bool = False) -> dict:
    now_ns = int(now_ns)
    model_age_ms = None if self.model_recv_ns <= 0 else (now_ns - self.model_recv_ns) / 1e6
    model_fresh = self.model_recv_ns > 0 and -50_000_000 <= now_ns - self.model_recv_ns <= MODEL_MAX_AGE_NS
    turn_fresh = self.turn_recv_ns > 0 and -50_000_000 <= now_ns - self.turn_recv_ns <= AUX_MAX_AGE_NS
    plan_fresh = self.plan_recv_ns > 0 and -50_000_000 <= now_ns - self.plan_recv_ns <= AUX_MAX_AGE_NS

    stopped = bool(self.standstill or abs(self.v_ego_mps) <= STOPPED_MAX_MPS)
    stopped_for_s = 0.0 if self.stop_since_ns <= 0 else max(0.0, (now_ns - self.stop_since_ns) / 1e9)
    approach_age_s = None if self.last_approach_ns <= 0 else max(0.0, (now_ns - self.last_approach_ns) / 1e9)
    approach_recent = bool(approach_age_s is not None and approach_age_s <= APPROACH_MEMORY_S)

    context_active = bool(model_fresh and stopped and not has_lead and not self.gas_pressed)
    stop_evidence = bool(self.should_stop or self.path_horizon_m < STOP_ARM_PATH_M)

    # Arm only after an actual approach.  This suppresses "red" while sitting in
    # a parking lot / booting stationary / other static non-intersection scenes.
    if context_active and approach_recent and not self.stop_armed:
      if stopped_for_s <= STOP_ARM_WINDOW_S and stop_evidence:
        self.stop_armed = True
        self.stop_armed_since_ns = now_ns
        self.stop_min_horizon_m = self.path_horizon_m
      elif stopped_for_s <= STOP_ARM_WINDOW_S and self.path_horizon_m < STOP_REARM_PATH_M:
        self.stop_armed = True
        self.stop_armed_since_ns = now_ns
        self.stop_min_horizon_m = self.path_horizon_m

    if self.stop_armed and stopped and model_fresh:
      self.stop_min_horizon_m = min(self.stop_min_horizon_m, self.path_horizon_m)

    stop_hold_s = 0.0 if self.stop_armed_since_ns <= 0 else max(0.0, (now_ns - self.stop_armed_since_ns) / 1e9)
    baseline_h = self.stop_min_horizon_m if math.isfinite(self.stop_min_horizon_m) else self.path_horizon_m
    horizon_jump_m = max(0.0, self.path_horizon_m - baseline_h)

    sp_green_alert = bool(plan_fresh and self.green_alert)
    fast_green_edge = bool(
      context_active and self.stop_armed and
      self.path_horizon_m >= GREEN_FAST_PATH_M and
      horizon_jump_m >= GREEN_FAST_TOTAL_JUMP_M and
      (self.horizon_delta_m >= GREEN_FAST_FRAME_JUMP_M or not self.should_stop)
    )
    if fast_green_edge:
      self.fast_edge_until_ns = max(self.fast_edge_until_ns, now_ns + 600_000_000)
    fast_green_edge_active = bool(context_active and self.stop_armed and now_ns < self.fast_edge_until_ns)

    normal_green_gate = bool(
      context_active and self.stop_armed and
      (not self.should_stop) and self.path_horizon_m >= GREEN_NORMAL_PATH_M
    )
    raw_green_gate = bool(sp_green_alert or fast_green_edge_active or normal_green_gate)

    if raw_green_gate:
      if self.green_candidate_since_ns <= 0:
        self.green_candidate_since_ns = now_ns
      if sp_green_alert:
        self.green_trigger_source = 'sunnypilot_green_alert'
      elif fast_green_edge_active:
        self.green_trigger_source = 'fast_path_open_edge'
      else:
        self.green_trigger_source = 'shouldStop_release_path_open'
    else:
      self.green_candidate_since_ns = 0

    green_gate_s = 0.0 if self.green_candidate_since_ns <= 0 else max(0.0, (now_ns - self.green_candidate_since_ns) / 1e9)
    green_confirmed = bool(sp_green_alert or (raw_green_gate and green_gate_s >= GREEN_CONFIRM_S))
    if green_confirmed:
      self.green_latch_until_ns = max(self.green_latch_until_ns, now_ns + int(GREEN_LATCH_S * 1e9))
    green_latched = bool(now_ns < self.green_latch_until_ns and self.stop_armed and stopped)

    # Scores are evidence indices (0..1), not calibrated probabilities.
    path_open = _clamp((self.path_horizon_m - 18.0) / 28.0)
    jump_score = _clamp(horizon_jump_m / 25.0)
    release_score = 0.0 if self.should_stop else 1.0
    accel_go = _clamp((self.desired_accel_mps2 + 0.25) / 1.10)
    go_score = _clamp(0.34 * path_open + 0.34 * jump_score + 0.22 * release_score + 0.10 * accel_go)
    if fast_green_edge_active:
      go_score = max(go_score, 0.90)
    if green_latched:
      go_score = max(go_score, 0.92)
    if sp_green_alert:
      go_score = max(go_score, 0.98)

    short_path = _clamp((20.0 - self.path_horizon_m) / 20.0)
    hold_score = _clamp((stop_hold_s - 2.0) / 6.0)
    approach_score = 1.0 if approach_recent else 0.0
    stop_score = _clamp(0.35 * (1.0 if self.should_stop else 0.0) + 0.30 * short_path + 0.20 * hold_score + 0.15 * approach_score)

    red_candidate = bool(
      context_active and self.stop_armed and not green_latched and
      stop_hold_s >= RED_CANDIDATE_HOLD_S and
      self.should_stop and self.path_horizon_m < RED_CANDIDATE_PATH_MAX_M
    )

    turn_dir = self.turn_direction if turn_fresh else 'none'
    # Arrow is a planned turn path only.  UI should illuminate it only with GO.
    turn_score = 1.0 if turn_dir in ('left', 'right') else 0.0

    if not model_fresh:
      state = 'UNKNOWN'
      label = 'SIGNAL ?'
    elif has_lead and stopped:
      state = 'WAIT_LEAD'
      label = 'LEAD AHEAD'
    elif green_latched:
      state = 'GREEN_GO'
      label = 'GREEN / GO'
    elif red_candidate:
      state = 'RED_CANDIDATE'
      label = 'RED ? · E2E STOP'
    elif self.stop_armed and stopped:
      state = 'STOP_HOLD'
      label = 'STOP/HOLD · LIGHT ?'
    elif stopped:
      state = 'WATCH'
      label = 'STOPPED · SIGNAL ?'
    else:
      state = 'DRIVING'
      label = 'DRIVING'

    self.last_state = state
    return {
      'version': PROBE_VERSION,
      'state': state,
      'label': label,
      'model_fresh': model_fresh,
      'model_age_ms': None if model_age_ms is None else round(model_age_ms, 1),
      'context_active': context_active,
      'standstill': stopped,
      'stopped_for_s': round(stopped_for_s, 2),
      'has_lead': bool(has_lead),
      'gas_pressed': bool(self.gas_pressed),
      'v_ego_mps': round(self.v_ego_mps, 3),
      'approach_recent': approach_recent,
      'approach_age_s': None if approach_age_s is None else round(approach_age_s, 2),
      'stop_armed': bool(self.stop_armed),
      'stop_hold_s': round(stop_hold_s, 2),
      'stop_min_horizon_m': None if not math.isfinite(self.stop_min_horizon_m) else round(self.stop_min_horizon_m, 2),
      'path_horizon_m': round(self.path_horizon_m, 2),
      'prev_path_horizon_m': round(self.prev_path_horizon_m, 2),
      'horizon_delta_m': round(self.horizon_delta_m, 2),
      'horizon_jump_m': round(horizon_jump_m, 2),
      'green_normal_path_m': GREEN_NORMAL_PATH_M,
      'green_fast_path_m': GREEN_FAST_PATH_M,
      'green_fast_total_jump_m': GREEN_FAST_TOTAL_JUMP_M,
      'green_gate_s': round(green_gate_s, 3),
      'green_confirm_s': GREEN_CONFIRM_S,
      'green_latch_s': GREEN_LATCH_S,
      'raw_green_gate': raw_green_gate,
      'fast_green_edge': fast_green_edge,
      'fast_green_edge_active': fast_green_edge_active,
      'green_confirmed': green_confirmed,
      'green_latched': green_latched,
      'green_trigger_source': self.green_trigger_source if green_latched or raw_green_gate else 'none',
      'sunnypilot_green_alert': sp_green_alert,
      'sunnypilot_plan_fresh': plan_fresh,
      'should_stop': bool(self.should_stop),
      'desired_accel_mps2': round(self.desired_accel_mps2, 3),
      'go_score': round(go_score, 3),
      'stop_score': round(stop_score, 3),
      'red_candidate': red_candidate,
      'red_candidate_hold_s': RED_CANDIDATE_HOLD_S,
      'red_candidate_path_max_m': RED_CANDIDATE_PATH_MAX_M,
      'turn_direction': turn_dir,
      'turn_score': round(turn_score, 3),
      'turn_source': 'modelDataV2SP.laneTurnDirection' if turn_fresh else 'none',
      'model_confidence': self.model_confidence,
      'model_confidence_note': 'ModelDataV2 confidence class; NOT traffic-light color',
      'direct_lamp_classifier': False,
      'score_type': 'heuristic_evidence_not_probability',
      'classification_limit': 'No direct traffic-lamp classifier in ModelDataV2; RED is conservative candidate, GREEN is stop->go E2E edge',
      'semantics': 'STOP/HOLD=E2E stop; RED?=conservative candidate only; GREEN/GO=path/action release edge; arrow=planned turn path only',
    }

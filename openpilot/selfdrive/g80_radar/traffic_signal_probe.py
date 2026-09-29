#!/usr/bin/env python3
"""Monitor-only E2E traffic-signal probe for the G80 browser HUD.

Important semantics
-------------------
Current sunnypilot/openpilot ModelDataV2 does NOT expose a direct camera
classification for red/green/yellow/arrow traffic-lamp pixels.  Sunnypilot's
Green Traffic Light Alert (Beta) is an E2E behavior heuristic: while stopped,
with no lead, the model path extending beyond ~30 m for >0.3 s is treated as a
green/go transition.

This probe makes those existing E2E cues visible so they can be compared with
real traffic lights during shadow testing.  It never publishes control messages
and never sends CAN.

Displayed scores are heuristic evidence scores, NOT calibrated probabilities.
Turn direction comes from modelDataV2SP.laneTurnDirection and means the model's
planned path direction; it is NOT direct recognition of a green-arrow lamp.
"""
from __future__ import annotations

import math

GREEN_LIGHT_X_THRESHOLD_M = 30.0
GREEN_CONFIRM_S = 0.30
MODEL_MAX_AGE_NS = 700_000_000
AUX_MAX_AGE_NS = 1_200_000_000
STOPPED_MAX_MPS = 0.35
RECENT_MOVING_CLEAR_S = 2.0


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
    self.should_stop = False
    self.desired_accel_mps2 = 0.0
    self.model_confidence = 'unknown'

    self.car_recv_ns = 0
    self.v_ego_mps = 0.0
    self.standstill = False
    self.gas_pressed = False
    self.last_moving_ns = 0
    self.stop_since_ns = 0

    self.turn_recv_ns = 0
    self.turn_direction = 'none'

    self.plan_recv_ns = 0
    self.green_alert = False

    self.green_gate_since_ns = 0
    self.last_state = 'UNKNOWN'

  def update_model(self, model, recv_ns: int) -> None:
    self.model_recv_ns = int(recv_ns)
    try:
      xs = list(model.position.x)
      vals = [_finite(x, math.nan) for x in xs]
      vals = [x for x in vals if math.isfinite(x)]
      self.path_horizon_m = max(vals) if vals else 0.0
    except Exception:
      self.path_horizon_m = 0.0
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
    recent_moving = ((now_ns - self.last_moving_ns) < int(RECENT_MOVING_CLEAR_S * 1e9)) if self.last_moving_ns > 0 else (stopped_for_s < RECENT_MOVING_CLEAR_S)

    # Match sunnypilot's beta green-light context as closely as possible without
    # participating in controls: stopped, no lead, no gas, and settled after motion.
    context_active = bool(model_fresh and stopped and not has_lead and not self.gas_pressed and not recent_moving)
    raw_green_gate = bool(model_fresh and self.path_horizon_m > GREEN_LIGHT_X_THRESHOLD_M and not self.should_stop)
    if context_active and raw_green_gate:
      if self.green_gate_since_ns <= 0:
        self.green_gate_since_ns = now_ns
    else:
      self.green_gate_since_ns = 0
    green_gate_s = 0.0 if self.green_gate_since_ns <= 0 else max(0.0, (now_ns - self.green_gate_since_ns) / 1e9)
    green_confirmed = bool(context_active and green_gate_s >= GREEN_CONFIRM_S)
    sp_green_alert = bool(plan_fresh and self.green_alert)

    # Evidence scores are deliberately simple and observable. They are NOT
    # calibrated probabilities and are only intended for field comparison.
    horizon_go = _clamp((self.path_horizon_m - 12.0) / 28.0)
    action_go = 0.0 if self.should_stop else 1.0
    accel_go = _clamp((self.desired_accel_mps2 + 0.35) / 1.35)
    go_score = _clamp(0.62 * horizon_go + 0.28 * action_go + 0.10 * accel_go)
    if green_confirmed:
      go_score = max(go_score, 0.88)
    if sp_green_alert:
      go_score = max(go_score, 0.98)
    stop_score = _clamp(0.62 * (1.0 - horizon_go) + 0.38 * (1.0 if self.should_stop else 0.0))

    turn_dir = self.turn_direction if turn_fresh else 'none'
    turn_score = 1.0 if turn_dir in ('left', 'right') else 0.0

    if not model_fresh:
      state = 'UNKNOWN'
      label = 'SIGNAL ?'
    elif has_lead and stopped:
      state = 'WAIT_LEAD'
      label = 'LEAD AHEAD'
    elif context_active and (sp_green_alert or green_confirmed) and go_score >= 0.65:
      state = 'GREEN_GO'
      label = 'GREEN OK'
    elif context_active and (self.should_stop or stop_score >= 0.62):
      state = 'RED_STOP_INFERRED'
      label = 'RED/STOP ?'
    elif stopped:
      state = 'WATCH'
      label = 'SIGNAL WATCH'
    else:
      state = 'DRIVING'
      label = 'DRIVING'

    self.last_state = state
    return {
      'version': 1,
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
      'path_horizon_m': round(self.path_horizon_m, 2),
      'green_threshold_m': GREEN_LIGHT_X_THRESHOLD_M,
      'green_gate_s': round(green_gate_s, 2),
      'green_confirm_s': GREEN_CONFIRM_S,
      'raw_green_gate': raw_green_gate,
      'green_confirmed': green_confirmed,
      'sunnypilot_green_alert': sp_green_alert,
      'sunnypilot_plan_fresh': plan_fresh,
      'should_stop': bool(self.should_stop),
      'desired_accel_mps2': round(self.desired_accel_mps2, 3),
      'go_score': round(go_score, 3),
      'stop_score': round(stop_score, 3),
      'turn_direction': turn_dir,
      'turn_score': round(turn_score, 3),
      'turn_source': 'modelDataV2SP.laneTurnDirection' if turn_fresh else 'none',
      'model_confidence': self.model_confidence,
      'model_confidence_note': 'ModelDataV2 confidence class; NOT traffic-light color',
      'direct_lamp_classifier': False,
      'score_type': 'heuristic_evidence_not_probability',
      'semantics': 'GREEN/STOP inferred from E2E path/action; LEFT/RIGHT is planned path, not lamp-arrow recognition',
    }

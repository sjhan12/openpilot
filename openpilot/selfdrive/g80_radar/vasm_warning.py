#!/usr/bin/env python3
"""V52R4 read-only radar/V-ASM advisory risk overlay.

Not a safety-rated sensor-fusion implementation.  This module never modifies
FG15, the vehicle controller, CAN, radarState, or planning outputs.  Its output
is used to strengthen the separate G80 web/HUD warning and diagnostic UDP only.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math

# Each camera alternates left/right; these bounds are for presentation only.
SIDE_AGE_MAX_MS = 2600.0
FRONT_AGE_MAX_MS = 2000.0
SIDE_SCORE_MIN = 0.78
FRONT_SCORE_MIN = 0.82
RED_RADAR_GAP_M = 9.0
RED_RADAR_TTC_S = 3.0
RED_RADAR_2D_S = 3.0


def _finite(value):
  try:
    n = float(value)
    return n if math.isfinite(n) else None
  except (TypeError, ValueError):
    return None


def _rank(label):
  name = str(label or 'UNKNOWN').upper()
  if name.startswith('DANGER'):
    return 3
  if name.startswith('CHECK') or name.startswith('NO LANE'):
    return 2
  if name.startswith('SAFE'):
    return 1
  return 0


def _observe(camera: dict, key: str, age_limit_ms: float, score_limit: float, source: str):
  c = camera if isinstance(camera, dict) else {}
  obj = c.get(key) if isinstance(c.get(key), dict) else {}
  age_ms = _finite(obj.get('inference_age_ms'))
  # NEVER use the overall daemon age as a fallback; the other ROI may be newer.
  ready = bool(c.get('usable') and c.get('onroad') and c.get('enabled') and
               c.get('model_valid') and c.get('config_loaded') and c.get('camera_connected') and
               age_ms is not None and 0 <= age_ms <= age_limit_ms)
  score = _finite(obj.get('score'))
  active = bool(ready and obj.get('active') and score is not None and score >= score_limit)
  stamp = int(obj.get('last_inference_mono_ns') or 0)
  return {'source': source, 'valid': ready, 'active': active,
          'score': score if ready else None, 'age_ms': age_ms,
          'inference_mono_ns': stamp if ready else 0}


def _radar_watch(side_data: dict):
  """Require a fresh measured radar hazard; never use camera-only objects."""
  e = (side_data.get('fg12_evidence') or side_data.get('fg11_evidence') or {})
  if not isinstance(e, dict):
    return None
  for o in (e.get('observations') or []):
    if not isinstance(o, dict) or not o.get('fresh'):
      continue
    source = [str(x).upper() for x in (o.get('source_mask') or [])]
    if not source or not any(x in source for x in ('FRONT', 'FL', 'FR', 'RL', 'RR', 'CORNER')):
      continue
    if not (o.get('current_core') or o.get('current_boundary') or o.get('stable_incoming')):
      continue
    gap = _finite(o.get('current_gap_m'))
    ttc = _finite(o.get('ttc_linear_s'))
    two_d = _finite(o.get('conflict_entry_s'))
    if (o.get('closing') and gap is not None and gap <= RED_RADAR_GAP_M and
        ((ttc is not None and 0 < ttc <= RED_RADAR_TTC_S) or
         (two_d is not None and 0 <= two_d <= RED_RADAR_2D_S))):
      return {'key': str(o.get('key') or ''), 'gap_m': gap, 'ttc_s': ttc,
              'two_d_s': two_d, 'source_mask': source}
  return None


@dataclass
class _SideState:
  last_seen_stamp: int = 0
  positive_frames: int = 0
  last_positive_ns: int = 0


@dataclass
class VASMWarningEvaluator:
  history: dict = field(default_factory=lambda: {s: _SideState() for s in ('left', 'right')})

  def update(self, future_gap: dict, cabin: dict, wide: dict, now_ns: int, enabled: bool = True) -> dict:
    fg = future_gap or {}
    now_ns = int(now_ns)
    out = {'version': 'V52R4_RADAR_FIRST_VASM_WARNING', 'mono_ns': now_ns,
           'mode': 'ACTIVE_HUD_ADVISORY', 'writes_fg15': False,
           'writes_vehicle_control': False, 'camera_can_clear_risk': False,
           'camera_only_danger_allowed': False, 'enabled': bool(enabled), 'left': {}, 'right': {}}
    intent = fg.get('driver_intent') or {}
    turn = str(intent.get('maneuver_context') or '').upper() == 'TURN' or str(intent.get('state') or '').upper() == 'TURN'
    committed = bool(intent.get('active') and intent.get('committed'))
    for side in ('left', 'right'):
      sd = fg.get(side) or {}
      base = (sd.get('decision') or {}).get('label') or 'UNKNOWN'
      # Keep active FG15 hold states separate from the warning overlay.
      state = self.history[side]
      cabin_o = _observe(cabin, side, SIDE_AGE_MAX_MS, SIDE_SCORE_MIN, 'CABIN_'+side.upper())
      wide_o = _observe(wide, 'fl' if side == 'left' else 'fr', FRONT_AGE_MAX_MS,
                        FRONT_SCORE_MIN, 'WIDE_'+side.upper())
      # Only cabin is eligible to start a visual side warning. The front WIDE
      # classifier is a side-window-trained model reused outside its domain.
      positive = bool(enabled and cabin_o['active'])
      stamp = cabin_o['inference_mono_ns']
      if not positive:
        state.last_seen_stamp = 0
        state.positive_frames = 0
        state.last_positive_ns = 0
      elif stamp > state.last_seen_stamp:
        if state.last_positive_ns and now_ns - state.last_positive_ns <= 4_000_000_000:
          state.positive_frames += 1
        else:
          state.positive_frames = 1
        state.last_seen_stamp = stamp
        state.last_positive_ns = now_ns
      radar = _radar_watch(sd)
      adjusted = base
      reason = 'FG15_UNCHANGED'
      # A turn is never reinterpreted as a lane change, and unknown/NO LANE are
      # preserved. V-ASM alone cannot establish adjacent lane occupancy.
      if enabled and not turn and not committed and _rank(base) and positive:
        if _rank(base) == 1:
          adjusted = 'CHECK CAM'
          reason = 'CABIN_VASM_CAUTION'
        elif (str(base).upper().startswith('CHECK') and 'DATA' not in str(base).upper() and
              state.positive_frames >= 2 and radar is not None):
          adjusted = 'DANGER CAM+RADAR'
          reason = 'CAMERA_AND_FRESH_RADAR_NEAR_CONFLICT'
      changed = adjusted != base
      out[side] = {'radar_label': base, 'warning_label': adjusted,
                   'changed': changed, 'upgrade': reason if changed else 'NONE',
                   'cabin': cabin_o, 'wide': wide_o,
                   'cabin_confirmed_frames': state.positive_frames,
                   'radar_near_conflict': radar,
                   'advisory_only': True}
    return out

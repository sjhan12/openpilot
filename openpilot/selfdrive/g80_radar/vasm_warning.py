#!/usr/bin/env python3
"""V53R1 read-only radar/V-ASM advisory risk overlay.

Not a safety-rated sensor-fusion implementation.  This module never modifies
FG15, the vehicle controller, CAN, radarState, or planning outputs.  Its output
is used only for G80 web/HUD advisory warnings and diagnostics.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math

# Each camera alternates left/right; these bounds are for presentation only.
SIDE_AGE_MAX_MS = 2600.0
FRONT_AGE_MAX_MS = 2000.0
SIDE_SCORE_MIN = 0.78
FRONT_SCORE_MIN = 0.82


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


def _radar_evidence(side_data: dict):
  """Diagnostic only; FG15 is the sole arbiter of red radar warnings."""
  e = (side_data.get('fg12_evidence') or side_data.get('fg11_evidence') or {})
  if not isinstance(e, dict):
    return None
  obs = e.get('observations') or []
  fresh = [o for o in obs if isinstance(o, dict) and o.get('fresh')]
  return {'fresh_observations': len(fresh), 'min_2d_conflict_time_s': e.get('min_2d_conflict_time_s')}


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
    out = {'version': 'V53R1_RADAR_FIRST_VASM_CHECK_ONLY', 'mono_ns': now_ns,
           'mode': 'ACTIVE_HUD_ADVISORY', 'writes_fg15': False,
           'writes_vehicle_control': False, 'camera_can_clear_risk': False,
           'camera_only_danger_allowed': False, 'camera_danger_upgrade_allowed': False, 'enabled': bool(enabled), 'left': {}, 'right': {}}
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
      radar = _radar_evidence(sd)
      adjusted = base
      reason = 'NONE'
      evidence_type = 'FG15'
      base_upper = str(base).upper()
      road_status = str((sd.get('lane_availability') or {}).get('status') or 'UNKNOWN').upper()
      # Distinguish unknown adjacent lane from a vehicle conflict. This does
      # NOT change FG15 road gate, nor does it authorize lane changing.
      if base_upper.startswith('CHECK ROAD'):
        adjusted = 'ROAD ?'
        reason = 'ROAD_GEOMETRY_UNCERTAIN'
        evidence_type = 'ROAD_UNCERTAIN'
      elif base_upper.startswith('NO LANE'):
        evidence_type = 'ROAD_ABSENT'
      elif base_upper.startswith('DANGER'):
        # Existing radar/BSD DANGER remains red irrespective of V-ASM CLEAR.
        evidence_type = 'RADAR_OR_BSD_DANGER'
        reason = 'FG15_BSD_DANGER_PRESERVED' if 'BSD' in base_upper else 'FG15_DANGER_PRESERVED'
      elif base_upper.startswith('CHECK'):
        evidence_type = 'RADAR_CHECK'
        if enabled and positive and not turn and not committed and 'DATA' not in base_upper:
          # Camera supports an EXISTING CHECK; no red upgrade, no 1:1 radar match.
          adjusted = 'CHECK CAM+RADAR'
          reason = 'CAMERA_SUPPORTS_CHECK'
      elif base_upper.startswith('SAFE'):
        evidence_type = 'RADAR_SAFE'
        if enabled and positive and not turn and not committed and road_status == 'CONFIRMED':
          adjusted = 'CHECK CAM'
          reason = 'CAMERA_CAUTION_ONLY'
      # Never use V-ASM negative evidence to clear a radar DANGER/CHECK.
      changed = adjusted != base
      camera_caution = bool(adjusted.startswith('CHECK CAM') and positive)
      out[side] = {'radar_label': base, 'warning_label': adjusted,
                   'changed': changed, 'upgrade': reason if reason == 'CAMERA_CAUTION_ONLY' else 'NONE',
                   'display_reason': reason, 'evidence_type': evidence_type,
                   'road_gate': road_status, 'road_uncertain': evidence_type == 'ROAD_UNCERTAIN',
                   'camera_caution': camera_caution, 'risk_increased_by_camera': reason == 'CAMERA_CAUTION_ONLY',
                   'cabin': cabin_o, 'wide': wide_o,
                   'cabin_confirmed_frames': state.positive_frames,
                   'radar_evidence': radar,
                   'radar_near_conflict': None, # no independent red classification
                   'advisory_only': True, 'control_eligible': False}
    return out

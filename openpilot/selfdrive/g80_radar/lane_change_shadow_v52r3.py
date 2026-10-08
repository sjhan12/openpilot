#!/usr/bin/env python3
"""V52R3 camera/radar evidence interface; not used by FG15 or car controls."""
from __future__ import annotations

CAMERA_AGE_LIMIT_MS = 3000.0

def _camera_side(camera: dict, key: str, side: str) -> dict:
  c = camera or {}
  k = ('fl' if side == 'left' else 'fr') if key == 'front' else side
  obs = c.get(k) or {}
  is_ready = bool(c.get('usable'))
  return {'source':('C4_FRONT_' if key=='front' else 'SIDE_') + k.upper(), 'ready':is_ready, 'occupied_candidate':bool(is_ready and obs.get('effective_active',obs.get('active'))), 'score':obs.get('score') if is_ready else None, 'age_ms':c.get('inference_age_ms'), 'calibration':'NOT_METRIC'}

def build_lane_change_shadow(future_gap:dict, side_vision:dict, front_corner:dict, now_ns:int) -> dict:
  """NO_CONTROL. Camera candidates may flag a review, never clear a target lane."""
  fg = future_gap or {}
  result = {'version':'V52R3_C4_ONLY_CAMERA_CORRELATION_V2','mono_ns':int(now_ns),'mode':'SHADOW_ONLY',
            'controller_export_allowed':False,'metric_camera_tracking':False,
            'limitation':'ROI classifier is not calibrated for physical distance/speed or target-lane occupancy', 'left':{},'right':{}}
  for side in ('left','right'):
    original = (((fg.get(side) or {}).get('decision')) or {}).get('label') or 'UNKNOWN'
    sources=[_camera_side(side_vision,'side',side),_camera_side(front_corner,'front',side)]
    candidate=[s['source'] for s in sources if s['occupied_candidate']]
    ready=[s['source'] for s in sources if s['ready']]
    # Not even a camera-positive ROI is definitive evidence of lane occupancy.
    note = 'CAMERA_REVIEW_CANDIDATE' if candidate else ('NO_CAMERA_CANDIDATE_NOT_CLEARANCE' if ready else 'CAMERA_UNAVAILABLE')
    result[side]={'fg15_unchanged_label':original,'camera_sources':sources,'camera_ready_count':len(ready),
                  'camera_positive_sources':candidate,'shadow_note':note,'safe_by_camera':False,
                  'actuation_eligible':False}
  return result


def build_shadow_lead_interface(shadow:dict, selected:list, now_ns:int) -> dict:
  """L1/L2 validation-only adapter. Never publish to radarState/controlsd."""
  sh=shadow or {}
  sel={x.get('shadow_role'):x for x in (selected or []) if isinstance(x,dict)}
  out={'version':1,'mono_ns':int(now_ns),'mode':'SHADOW_ONLY','controller_export_allowed':False,
       'uses_production_radar_state':False,'leadOne':{},'leadTwo':{}}
  for role,dst,src in (('L1','leadOne','leadOne'),('L2','leadTwo','leadTwo')):
    obj=sel.get(role) or {}
    d=sh.get(src) or {}
    valid=bool(obj and d.get('status'))
    out[dst]={'role':role,'candidate_valid':valid,'canonical_key':str(obj.get('canonical_key') or '') if valid else '',
              'x_m':obj.get('x') if valid else None,'y_m':obj.get('y') if valid else None,
              'vx_mps':obj.get('vx') if valid else None,'validation_state':d.get('validationState'),
              'radar_identity_status':'SHADOW_NOT_CONTROL_APPROVED'}
  return out

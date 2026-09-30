#!/usr/bin/env python3
"""Read-only V41 browser stage contracts and diagnostic raw gate.

Never feeds the fusion, Future Gap, UDP output, car state, radarState or CAN.
Stage ownership is provenance, not object position or fused canonical domain.
"""
from __future__ import annotations
from collections import Counter
from typing import Any

from openpilot.selfdrive.g80_radar.tracker import mark_teacher_matches, _corner_pass, _ref_pass

WEB_VIEW_FIELDS = {
  'raw': 'raw_objects',
  'raw_filtered': 'raw_filtered_objects',
  'corner_fused': 'corner_fused_objects',
  'front_fused': 'front_sensor_objects',
  'all': 'sensor_fused_objects',
  'l1l2': 'selected_shadow_leads',
  'std': 'standard_front_preview',
}
FRONT_VIEW_X_MIN_M = -0.5
FRONT_VIEW_ABS_Y_MAX_M = 5.8  # current CameraRadarFusion.front_sensor_objects ROI


def physical_origin(o: dict) -> str:
  """Return physical stream, explicitly distinguish unverified group1 candidate."""
  src = str(o.get('source') or '')
  sensor = str(o.get('sensor') or '')
  if src == 'corner24':
    return 'CORNER_A' if sensor == 'corner_A' else ('CORNER_B' if sensor == 'corner_B' else 'CORNER_BANK_UNKNOWN')
  if src == 'fr_cmr_reference': return 'FRONT_REFERENCE'
  if src == 'front_group1_candidate': return 'FRONT_GROUP1_CANDIDATE'
  if src in ('front_track', 'front_radar'): return 'FRONT_RADAR'
  if src == 'c4_camera' or bool(o.get('camera_only')): return 'CAMERA_ONLY'
  return 'UNKNOWN'


def diagnostic_raw_filtered(raw: list, rear_teacher: list) -> list:
  """Apply the existing validity/stability gates BEFORE cross-source dedup.

  Unlike production filtered_objects, this read-only browser stage intentionally
  preserves two nearby physical returns to reveal cross-sensor overlap.
  FRONT_GROUP1 remains RAW only: its decode is an unverified candidate.
  """
  marked = mark_teacher_matches(raw, rear_teacher)
  out = []
  for o in marked:
    src = o.get('source')
    if (src == 'corner24' and _corner_pass(o)) or (src == 'fr_cmr_reference' and _ref_pass(o)):
      q = dict(o)
      q['stage_origin'] = physical_origin(q)
      q['stage_gate'] = 'VALIDITY_AND_CONTINUITY_NO_CROSS_SENSOR_DEDUP'
      out.append(q)
  return out


def annotate_raw(raw: list) -> list:
  return [dict(o, stage_origin=physical_origin(o), stage_gate='DECODED_UNFILTERED') for o in raw]


def selected_shadow_leads(shadow: dict, canonical: list) -> list:
  """Exactly selected shadow L1/L2; never display every shadow candidate."""
  by_key = {str(o.get('canonical_key') or o.get('vehicle_key') or o.get('key')):o for o in canonical}
  candidates = (shadow or {}).get('candidates') or []
  out=[]
  for role,field in (('L1','leadOne'),('L2','leadTwo')):
    target = (shadow or {}).get(field) or {}
    if not target.get('status'): continue
    k = str(target.get('key') or target.get('vehicleKey') or '')
    if not k: continue
    c = next((x for x in candidates if str(x.get('key') or x.get('vehicle_key')) == k), None)
    if c is not None:
      x,y,vx = c.get('x'),c.get('y'),c.get('vx')
    else:
      x,y,vx = target.get('dRel'),target.get('yRel'),target.get('vRel')
    if x is None or y is None: continue
    can = by_key.get(k) or {}
    item = {
      'key':k,'canonical_key':k,'source':'selected_shadow_lead','shadow_role':role,
      'x':x,'y':y,'vx':vx,'source_mask':can.get('source_mask') or [],
      'source_age_ms':can.get('source_age_ms'),
      'display_state':'SELECTED_SHADOW_LEAD','camera_confirmed':bool(target.get('cameraConfirmed')),
      'scc_teacher_confirmed':bool(target.get('sccConfirmed')),
      'shadow_validation_state':target.get('validationState'),
      'shadow_reason':target.get('reason'),'stage_origin':'SHADOW_'+role,
    }
    out.append(item)
  return out


def make_stage_audit(raw: list, raw_filtered_diag: list, prod_filtered: list,
                     corner: list, front: list, canonical: list, shadow_selected: list,
                     std: list) -> dict:
  """Make each stage's owner, coverage, omission and provenance measurable."""
  src_count=Counter(physical_origin(o) for o in raw)
  passed_count=Counter(physical_origin(o) for o in raw_filtered_diag)
  front_keys={str(o.get('canonical_key')) for o in front if o.get('canonical_key')}
  canonical_keys=[str(o.get('canonical_key')) for o in canonical if o.get('canonical_key')]
  front_expected = 0;out_roi=0;front_unmatched_in_roi=0
  for o in canonical:
    mask={str(s).upper() for s in (o.get('source_mask') or o.get('canonical_domains') or [])}
    if 'FRONT' not in mask:continue
    front_expected+=1
    x=float(o.get('x') or 0);y=float(o.get('y') or 0)
    if x<FRONT_VIEW_X_MIN_M or abs(y)>FRONT_VIEW_ABS_Y_MAX_M:
      out_roi+=1
    elif str(o.get('canonical_key')) not in front_keys:
      front_unmatched_in_roi+=1
  counts={
    'raw':len(raw),'raw_filtered':len(raw_filtered_diag),
    'production_filtered':len(prod_filtered), 'corner_fused':len(corner),
    'front_fused':len(front), 'all':len(canonical),
    'l1l2':len(shadow_selected), 'std':len(std),
  }
  return {
    'schema':1,'counts':counts,
    'raw_sources':dict(src_count),'raw_filtered_sources':dict(passed_count),
    'raw_filtered_contract':'validity+continuity, no cross-source dedup, group1 RAW-only',
    'front_fused_contract':'front_sensor_objects (front + possible CAMERA-only), ROI x>=-0.5, abs(y)<=5.8',
    'all_contract':'one Canonical360 Vxxxx per physical fused object (FRONT already included)',
    'selected_leads_contract':'only selected L1 and L2, not all shadow candidates',
    'front_domain_canonical_count':front_expected,
    'front_outside_view_roi':out_roi,
    'front_unmatched_inside_view_roi':front_unmatched_in_roi,
    'canonical_duplicate_keys':len(canonical_keys)-len(set(canonical_keys)),
    'front_group1_raw_only':src_count.get('FRONT_GROUP1_CANDIDATE',0),
    'production_dedup_removed_count':max(0,len(raw_filtered_diag)-len(prod_filtered)),
    'monitor_only':True,
  }


def validate_stage_contract(raw: list, raw_filtered_diag: list, corner: list,
                            front: list, canonical: list, selected: list,
                            std: list) -> list[str]:
  errors=[]
  if any(o.get('source') not in ('corner24','fr_cmr_reference') for o in raw_filtered_diag):
    errors.append('RAW_FILTERED_SOURCE_LEAK')
  if any(o.get('source') not in ('corner_fused','corner_vehicle_fused') for o in corner):
    errors.append('CORNER_FUSED_SOURCE_LEAK')
  if any(o.get('source') not in ('front_track','fr_cmr_reference','c4_camera') and not o.get('camera_only') for o in front):
    errors.append('FRONT_FUSED_SOURCE_LEAK')
  if any(o.get('shadow_role') not in ('L1','L2') for o in selected):
    errors.append('L1_L2_ROLE_LEAK')
  if any(o.get('source')!='future_standard_front_preview' for o in std):
    errors.append('STD_SOURCE_LEAK')
  keys=[o.get('canonical_key') for o in canonical if o.get('canonical_key')]
  if len(keys)!=len(set(keys)):
    errors.append('CANONICAL_DUPLICATE_KEY')
  return errors

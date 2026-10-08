"""V52R5 lightweight radar-first cut-in/out candidate display (SHADOW ONLY).

Scores are uncalibrated evidence indices, NOT event likelihoods, not TTC or
safety decisions. This module must never send CAN/actuation/radarState or alter FG15.
"""
from __future__ import annotations
import math

CUTIN_LIMIT = 3
CUTOUT_LIMIT = 2
MIN_FRESH_AGE_MS = 450.0
CUTIN_MIN_INDEX = 45
CUTOUT_MIN_INDEX = 42


def _number(x, default=None):
  try:
    v = float(x)
    return v if math.isfinite(v) else default
  except (ValueError, TypeError, OverflowError):
    return default


def _index(value):
  n = _number(value, 0.0)
  return int(round(max(0.0, min(1.0, n)) * 100))


def _ids(o):
  return set(str(v) for k in ('canonical_key', 'vehicle_key', 'key', 'source_key') for v in [o.get(k)] if v is not None and str(v))


def make_cut_event_shadow(objects: list, shadow: dict, now_ns: int) -> dict:
  """Produce a bounded JSON-ready diagnostic from existing KF and lead history."""
  empty = {'version':1, 'mode':'SHADOW_ONLY', 'calibrated_probability':False,
           'score_unit':'heuristic_index_0_100_not_probability',
           'controls_connected':False, 'candidate_limit':{'cutin':CUTIN_LIMIT,'cutout':CUTOUT_LIMIT},
           'mono_ns':int(now_ns), 'path_valid':False, 'cutin':[], 'cutout':[],
           'candidates':[], 'stats':{'cutin_found':0,'cutout_found':0,'selected':0}}
  if not isinstance(shadow, dict) or not isinstance(shadow.get('stats'), dict):return empty
  if not bool(shadow['stats'].get('path_valid')):return empty
  result = dict(empty, path_valid=True)
  raw = shadow.get('candidates') or []
  lookup = {}
  for o in objects or []:
    if not isinstance(o,dict):continue
    for key in _ids(o):
      if key not in lookup:lookup[key] = o
  leads = shadow.get('leadOne') or {}
  lead_id = _ids(leads) if leads.get('status') else set()
  inbound, outbound = [], []
  seen_cutin, seen_cutout = set(), set()
  for c in raw:   # shadow_leads already gates sample freshness & <=140 m
    if not isinstance(c, dict): continue
    age = _number(c.get('age_ms'))
    x = _number(c.get('x'))
    y = _number(c.get('y'))
    d = _number(c.get('d_path'))
    if age is None or age < 0 or age > MIN_FRESH_AGE_MS or x is None or y is None or d is None: continue
    if x < 2.0 or x > 100.0 or bool(c.get('far_unconfirmed')): continue
    if c.get('source') == 'c4_camera':continue  # physical radar evidence mandatory
    oid = _ids(c)
    if not oid:continue
    o = next((lookup[k] for k in oid if k in lookup), {})
    # No indirect camera-only vehicles or stale/invalid radar-tracks.
    if o.get('source') == 'c4_camera':continue
    source = str(o.get('source',c.get('source','')))
    cid = str(o.get('canonical_key') or c.get('vehicle_key') or c.get('key'))
    if not cid:continue
    lane = str(o.get('kf_lane') or '')
    valid_kf = bool(o.get('kf_frenet_valid'))
    ttlc = _number(o.get('kf_ttlc_s')) if valid_kf else None
    kf_candidate = bool(o.get('kf_cutin_candidate') and valid_kf)
    kf_confirmed = bool(o.get('kf_cutin_confirmed') and valid_kf)
    raw_in = _index(c.get('cutin_score'))
    kf_in = _index(o.get('kf_cutin_score')) if kf_candidate else 0
    in_confirmed = bool(c.get('cutin_confirmed') or kf_confirmed)
    # Only real lateral-entry evidence, not 'there is a car nearby'.
    lateral_evidence = kf_candidate or in_confirmed or (raw_in >= 60 and abs(d) <= 3.7 and not c.get('path_occupied'))
    if lateral_evidence and (abs(d) <= 4.3 or kf_candidate) and cid not in seen_cutin:
      score = max(raw_in,kf_in)
      if (score >= CUTIN_MIN_INDEX or in_confirmed) and (not lane or lane in ('left1','right1','ego') or in_confirmed):
        seen_cutin.add(cid)
        inbound.append({'kind':'CUT-IN','key':cid,'canonical_key':str(o.get('canonical_key') or ''),
                        'score_index':score,'status':'CONFIRMED' if in_confirmed else 'CANDIDATE',
                        'x':round(x,2),'y':round(y,2),'d_path':round(d,2),
                        'canonical_primary_domain':o.get('canonical_primary_domain'), 'sector':o.get('sector'),
                        'vx':_number(c.get('vx')),'source':source,'age_ms':round(age,1),
                        'ttlc_s':None if ttlc is None or ttlc < 0 or ttlc > 10 else round(ttlc,2),
                        'kf_candidate':kf_candidate, 'kf_confirmed':kf_confirmed,
                        'shadow_confirmed':bool(c.get('cutin_confirmed')), 'shadow_index':raw_in,
                        'kf_index':kf_in, 'lane':lane or None,
                        'shadow_role':str(c.get('shadow_role') or ''),
                        'reason':'KF_TTLC_AND_PATH' if kf_candidate else 'PATH_INWARD_HISTORY'})
    # CUT-OUT is only for tracked objects from our route (prefer L1),
    # never for a vehicle merely traveling away in the outer lane.
    is_lead1 = bool(lead_id & oid) or c.get('shadow_role') == 'L1'
    was_in_path = bool(c.get('path_occupied')) or is_lead1
    raw_out = _index(c.get('cutout_score'))
    if (was_in_path and abs(d) <= (2.6 if is_lead1 else 1.55)
        and raw_out >= CUTOUT_MIN_INDEX and cid not in seen_cutout):
      seen_cutout.add(cid)
      outbound.append({'kind':'CUT-OUT','key':cid,'canonical_key':str(o.get('canonical_key') or ''),
                       'score_index':raw_out,'status':'CANDIDATE',
                       'x':round(x,2),'y':round(y,2),'d_path':round(d,2),
                        'canonical_primary_domain':o.get('canonical_primary_domain'), 'sector':o.get('sector'),
                       'vx':_number(c.get('vx')),'source':source,'age_ms':round(age,1),
                       'ttlc_s':None,'kf_candidate':False, 'kf_confirmed':False,
                       'shadow_confirmed':False,'shadow_index':raw_out,'kf_index':0,
                       'lane':lane or None,'shadow_role':'L1' if is_lead1 else str(c.get('shadow_role') or ''),
                       'reason':'LEAD1_MOVING_OUT' if is_lead1 else 'IN_PATH_MOVING_OUT'})
  # A canonical object may have noisy conflicting history. Show it once only.
  in_map={c['key']:c for c in inbound}
  out_map={c['key']:c for c in outbound}
  for key in in_map.keys() & out_map.keys():
    cin=in_map[key];cout=out_map[key]
    if cin['status']=='CONFIRMED' or cin['score_index'] >= cout['score_index']:
      out_map.pop(key,None)
    else:
      in_map.pop(key,None)
  inbound=list(in_map.values())
  outbound=list(out_map.values())
  inbound.sort(key=lambda c:(-c['score_index'],not c['kf_confirmed'],c['x'],c['key']))
  outbound.sort(key=lambda c:(-c['score_index'],c['shadow_role']!='L1',c['x'],c['key']))
  result.update(cutin=inbound[:CUTIN_LIMIT],cutout=outbound[:CUTOUT_LIMIT],
                stats={'cutin_found':len(inbound),'cutout_found':len(outbound),
                       'selected':len(inbound[:CUTIN_LIMIT])+len(outbound[:CUTOUT_LIMIT])})
  result['candidates']=result['cutin']+result['cutout']
  return result

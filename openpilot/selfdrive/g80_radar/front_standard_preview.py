#!/usr/bin/env python3
"""Future standard-radard FRONT adapter preview for G80 V20.

MONITOR-ONLY CONTRACT
---------------------
This module intentionally returns plain Python dictionaries only.
It does NOT import/create RadarData, does NOT publish radarTracks/radarState,
and does NOT send CAN.

Its purpose is to validate, while driving, the exact FRONT physical tracks that
could later be mapped to the standard openpilot RadarData.RadarPoint contract:
  preview track_id -> RadarPoint.trackId
  x                -> RadarPoint.dRel
  y                -> RadarPoint.yRel
  vx               -> RadarPoint.vRel

The raw source is the empirically decoded G80 0x210..0x21F Group1 bank.  The
existing V17/V18 ADAS front reference, C4 model leads, and SCC teacher are used
only as read-only validation teachers.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

SOURCE = 'front_group1_candidate'

# Keep the de-dup/re-identification geometry aligned with the V17 shadow result.
SAME_FRONT_DX_M = 2.6
SAME_FRONT_DY_M = 1.5
SAME_FRONT_DV_MPS = 3.0
REID_DX_M = 4.8
REID_DY_M = 2.1
REID_DV_MPS = 3.0

FRESH_MS = 180.0
MIN_CONSECUTIVE = 2
TRACK_TTL_NS = 450_000_000


def _f(v, default=None):
  try:
    x = float(v)
    return x if math.isfinite(x) else default
  except Exception:
    return default


def _valid_raw(o: dict) -> bool:
  if o.get('source') != SOURCE:
    return False
  x, y, vx = _f(o.get('x')), _f(o.get('y')), _f(o.get('vx'))
  if x is None or y is None or vx is None:
    return False
  if not (0.5 <= x <= 220.0 and abs(y) <= 12.0 and abs(vx) <= 80.0):
    return False
  if float(o.get('display_age_ms', 1e9) or 1e9) > FRESH_MS:
    return False
  return int(o.get('track_consecutive', 0) or 0) >= MIN_CONSECUTIVE


def _same_vehicle(a: dict, b: dict) -> bool:
  return (abs(float(a['x']) - float(b['x'])) <= SAME_FRONT_DX_M and
          abs(float(a['y']) - float(b['y'])) <= SAME_FRONT_DY_M and
          abs(float(a['vx']) - float(b['vx'])) <= SAME_FRONT_DV_MPS)


def _cluster_ok(group: list[dict]) -> bool:
  xs = [float(o['x']) for o in group]
  ys = [float(o['y']) for o in group]
  vs = [float(o['vx']) for o in group]
  return (max(xs)-min(xs) <= SAME_FRONT_DX_M and
          max(ys)-min(ys) <= SAME_FRONT_DY_M and
          max(vs)-min(vs) <= SAME_FRONT_DV_MPS)


def dedup_group1(objects: list[dict]) -> tuple[list[dict], dict]:
  """Complete-link de-dup inside one passenger-car-sized front return group."""
  cand = [dict(o) for o in objects if _valid_raw(o)]
  order = sorted(range(len(cand)), key=lambda i:(
    int(cand[i].get('track_consecutive', 0) or 0),
    -abs(float(cand[i]['y'])),
    -float(cand[i]['x'])
  ), reverse=True)
  remaining = set(order)
  groups: list[list[int]] = []
  for i in order:
    if i not in remaining:
      continue
    remaining.remove(i)
    g = [i]
    for j in list(remaining):
      if all(_same_vehicle(cand[k], cand[j]) for k in g) and _cluster_ok([cand[k] for k in g] + [cand[j]]):
        g.append(j)
        remaining.remove(j)
    groups.append(g)

  out = []
  for g in groups:
    members = [cand[i] for i in g]
    # Prefer the longest-lived member rather than averaging potentially different
    # reflecting surfaces on one physical vehicle.
    anchor = max(members, key=lambda o:(int(o.get('track_consecutive',0) or 0), -abs(float(o['y']))))
    d = dict(anchor)
    d['preview_member_keys'] = sorted(str(o.get('key','')) for o in members)
    d['preview_member_count'] = len(members)
    d['preview_duplicates_merged'] = max(0, len(members)-1)
    out.append(d)
  out.sort(key=lambda o: float(o['x']))
  return out, {
    'group1_fresh_before_dedup': len(cand),
    'group1_after_dedup': len(out),
    'group1_duplicates_merged': max(0, len(cand)-len(out)),
  }


@dataclass(slots=True)
class _Track:
  track_id: int
  x: float
  y: float
  vx: float
  last_ns: int
  source_key: str
  hits: int = 1


class StandardFrontPreview:
  """Persistent physical FRONT tracks shaped like future RadarPoint inputs."""
  def __init__(self):
    self.tracks: dict[int, _Track] = {}
    self.next_id = 1

  @staticmethod
  def _cost(t: _Track, d: dict, now_ns: int):
    dt = max(0.0, min(0.5, (int(now_ns)-int(t.last_ns))/1e9))
    px = t.x + t.vx * dt
    dx = abs(float(d['x'])-px)
    dy = abs(float(d['y'])-t.y)
    dv = abs(float(d['vx'])-t.vx)
    if dx > REID_DX_M or dy > REID_DY_M or dv > REID_DV_MPS:
      return None
    same_key = str(d.get('key','')) == t.source_key
    return (dx/REID_DX_M)**2 + (dy/REID_DY_M)**2 + 0.35*(dv/REID_DV_MPS)**2 - (0.4 if same_key else 0.0)

  @staticmethod
  def _nearest(obj: dict, candidates: list[dict], dx_gate: float, dy_gate: float, dv_gate: float):
    best = None
    for c in candidates or []:
      cx, cy = _f(c.get('x')), _f(c.get('y'))
      if cx is None or cy is None:
        continue
      dx = abs(float(obj['x'])-cx)
      dy = abs(float(obj['y'])-cy)
      ov, cv = _f(obj.get('vx')), _f(c.get('vx'))
      dv = 0.0 if ov is None or cv is None else abs(ov-cv)
      if dx > dx_gate or dy > dy_gate or dv > dv_gate:
        continue
      cost = (dx/dx_gate)**2 + (dy/dy_gate)**2 + 0.25*(dv/dv_gate)**2
      if best is None or cost < best[0]:
        best = (cost, c, dx, dy, dv)
    return best

  def update(self, raw_objects: list[dict], front_reference: list[dict], camera_leads: list[dict],
             scc_teacher: dict | None, now_ns: int) -> dict:
    detections, stats = dedup_group1(raw_objects)

    # Expire first so stale tracks cannot steal a new detection.
    for tid in list(self.tracks):
      if int(now_ns) - int(self.tracks[tid].last_ns) > TRACK_TTL_NS:
        self.tracks.pop(tid, None)

    pairs = []
    for tid, t in self.tracks.items():
      for i, d in enumerate(detections):
        c = self._cost(t, d, now_ns)
        if c is not None:
          pairs.append((c, tid, i))
    pairs.sort()

    used_t, used_d = set(), set()
    for _, tid, i in pairs:
      if tid in used_t or i in used_d:
        continue
      t, d = self.tracks[tid], detections[i]
      t.x, t.y, t.vx = float(d['x']), float(d['y']), float(d['vx'])
      t.last_ns, t.source_key = int(now_ns), str(d.get('key',''))
      t.hits += 1
      used_t.add(tid); used_d.add(i)

    for i, d in enumerate(detections):
      if i in used_d:
        continue
      tid = self.next_id
      self.next_id += 1
      self.tracks[tid] = _Track(tid, float(d['x']), float(d['y']), float(d['vx']), int(now_ns), str(d.get('key','')))
      used_t.add(tid)

    # Only tracks updated in this cycle are exposed as future RadarPoint preview.
    points = []
    for tid in sorted(used_t):
      t = self.tracks.get(tid)
      if t is None:
        continue
      p = {
        'source':'future_standard_front_preview',
        'key':f'RP{tid}',
        'track_id':tid,
        'x':round(t.x,3), 'y':round(t.y,3), 'vx':round(t.vx,3),
        'dRel':round(t.x,3), 'yRel':round(t.y,3), 'vRel':round(t.vx,3),
        'source_key':t.source_key, 'hits':t.hits,
        'future_mapping':'RadarPoint(trackId,dRel,yRel,vRel)',
        'monitor_only':True,
      }
      ref = self._nearest(p, front_reference, 6.0, 3.0, 5.0)
      if ref is not None:
        _, r, dx, dy, dv = ref
        p.update({'reference_match':True,'reference_key':r.get('front_key',r.get('key')),
                  'reference_dx_m':round(dx,3),'reference_dy_m':round(dy,3),'reference_dv_mps':round(dv,3)})
      cam = self._nearest(p, camera_leads, max(5.0,0.25*max(1.0,p['x'])), 3.0, 10.0)
      if cam is not None:
        _, c, dx, dy, dv = cam
        p.update({'camera_match':True,'camera_key':c.get('key'),
                  'camera_dx_m':round(dx,3),'camera_dy_m':round(dy,3),'camera_dv_mps':round(dv,3),
                  'camera_prob':c.get('camera_prob',c.get('prob'))})
      points.append(p)

    teacher_error = None
    teacher_key = None
    if scc_teacher and scc_teacher.get('teacher_usable'):
      td = _f(scc_teacher.get('distance_m'))
      tv = _f(scc_teacher.get('rel_speed_mps'))
      center = [p for p in points if abs(float(p['y'])) <= 2.1]
      best = None
      if td is not None:
        for p in center:
          dx = abs(float(p['x'])-td)
          dv = 0.0 if tv is None else abs(float(p['vx'])-tv)
          cost = dx + 0.4*dv
          if best is None or cost < best[0]:
            best = (cost,p,dx,dv)
      if best is not None:
        _, p, dx, dv = best
        teacher_error = round(dx,3)
        teacher_key = p['key']
        p['scc_teacher_candidate'] = True
        p['scc_teacher_dx_m'] = round(dx,3)
        p['scc_teacher_dv_mps'] = round(dv,3)

    stats.update({
      'preview_track_count':len(points),
      'reference_match_count':sum(1 for p in points if p.get('reference_match')),
      'camera_match_count':sum(1 for p in points if p.get('camera_match')),
      'scc_teacher_candidate':teacher_key,
      'scc_teacher_dx_m':teacher_error,
      'publishes_radarTracks':False,
      'publishes_radarState':False,
      'can_tx':False,
      'control_connected':False,
    })
    return {'points':points,'stats':stats}

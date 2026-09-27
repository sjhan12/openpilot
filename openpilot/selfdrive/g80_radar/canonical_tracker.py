#!/usr/bin/env python3
"""V30 canonical 360-degree physical-vehicle identity tracker.

This module is monitor-only. It does not publish radarTracks/radarState and does
not transmit CAN. Its only job is to turn the already de-duplicated multi-sensor
vehicle clusters into one persistent global identity namespace (V0001, V0002,
...) that survives front/corner/camera hand-offs and short measurement gaps.

Downstream Kalman/shadow logic must use these canonical IDs. Local FRONT/CORNER
trackers may still exist for diagnostics, but they are not identity authorities.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field


def _finite(v, default=None):
  try:
    x=float(v)
    return x if math.isfinite(x) else default
  except Exception:
    return default


def _strset(vals):
  if vals is None:
    return set()
  if isinstance(vals,(list,tuple,set)):
    return {str(v) for v in vals if v is not None and str(v)}
  return {str(vals)} if str(vals) else set()


def _aliases(o: dict) -> set[str]:
  """Stable sensor aliases carried by one physical vehicle cluster.

  C4 leadsV3 CV0/CV1/CV2 are hypothesis slots, not persistent object IDs, so
  they are intentionally excluded from canonical identity matching.
  """
  def stable(v):
    if v is None: return None
    q=str(v)
    if not q or (q.startswith('CV') and q[2:].isdigit()): return None
    return q
  out=set()
  for v in _strset(o.get('vehicle_cluster_keys')):
    q=stable(v)
    if q: out.add(q)
  for k in ('key','vehicle_anchor_key','front_link'):
    q=stable(o.get(k))
    if q: out.add(q)
  # camera_key / camera_hypothesis_keys are evidence only, never identity aliases.
  # Numeric corner link IDs are namespaced so they cannot collide with raw keys.
  if o.get('corner_link_id') is not None:
    out.add(f"corner_link:{o.get('corner_link_id')}")
  return out


def _source_domains(o: dict) -> set[str]:
  """Current sensing domains, for hand-off diagnostics only."""
  out=set()
  src=str(o.get('source',''))
  sources=_strset(o.get('vehicle_cluster_sources'))
  allsrc={src}|sources
  if any('front' in s for s in allsrc) or o.get('front_link') or o.get('scc_teacher_confirmed'):
    out.add('FRONT')
  if any('corner' in s for s in allsrc) or o.get('corner_link_id') is not None:
    sec=str(o.get('sector') or '')
    if sec in ('FL','FR','RL','RR'):
      out.add(sec)
    else:
      x=_finite(o.get('x'),0.0); y=_finite(o.get('y'),0.0)
      if x is not None and y is not None:
        if x >= 0.0 and y > 1.0: out.add('FL')
        elif x >= 0.0 and y < -1.0: out.add('FR')
        elif x < 0.0 and y > 1.0: out.add('RL')
        elif x < 0.0 and y < -1.0: out.add('RR')
        else: out.add('CORNER')
  if any(s=='c4_camera' for s in allsrc) or o.get('camera_confirmed') or o.get('camera_only'):
    out.add('CAMERA')
  if o.get('rear_teacher_confirmed') or o.get('teacher_match'):
    out.add('REAR_TEACHER')
  if not out:
    out.add(src.upper() if src else 'UNKNOWN')
  return out


def _primary_domain(domains:set[str], o:dict) -> str:
  # Preserve physical sensor hand-off information rather than evidence labels.
  for d in ('RL','RR','FL','FR'):
    if d in domains:
      return d
  if 'FRONT' in domains:
    return 'FRONT'
  if 'CAMERA' in domains:
    return 'CAMERA'
  return sorted(domains)[0] if domains else 'UNKNOWN'


@dataclass
class CanonicalTrack:
  tid:int
  first_ns:int
  last_ns:int
  x:float
  y:float
  vx:float|None
  age_frames:int=1
  aliases:set[str]=field(default_factory=set)
  source_history:set[str]=field(default_factory=set)
  last_domains:set[str]=field(default_factory=set)
  last_primary_domain:str='UNKNOWN'
  handoff_count:int=0
  reacquire_count:int=0
  road_d:float|None=None
  road_s:float|None=None
  lane:str|None=None


class Canonical360Tracker:
  """One global physical-vehicle ID namespace for all fused sensors."""
  def __init__(self,prefix='V',ttl_s=1.5):
    self.prefix=str(prefix)
    self.ttl_ns=int(float(ttl_s)*1e9)
    self.tracks:dict[int,CanonicalTrack]={}
    self.next_id=1
    self.created_total=0
    self.handoff_total=0
    self.reacquire_total=0

  @staticmethod
  def _candidate(o:dict,t:CanonicalTrack,now_ns:int):
    x=_finite(o.get('x')); y=_finite(o.get('y'))
    if x is None or y is None: return None
    dt=max(0.0,min(1.5,(int(now_ns)-int(t.last_ns))/1e9))
    px=t.x + (0.0 if t.vx is None else t.vx)*dt
    py=t.y
    dx=abs(x-px); dy=abs(y-py)
    ov=_finite(o.get('vx')); dv=0.0 if ov is None or t.vx is None else abs(ov-t.vx)
    oa=_aliases(o); overlap=len(oa & t.aliases)

    od=_finite(o.get('road_d')) if o.get('road_projection_valid') else None
    dd=None if od is None or t.road_d is None else abs(od-t.road_d)
    lane=str(o.get('road_lane')) if o.get('road_lane') is not None else None

    # Shared raw/front/camera alias is the strongest cue. Allow a wider geometric
    # hand-off gate because different radars can return different body surfaces.
    if overlap:
      if dx>8.0 or dy>3.0 or dv>5.0: return None
      cost=-20.0*overlap + 0.10*dx + 0.20*dy + 0.05*dv
      if dd is not None: cost += 0.06*min(dd,4.0)
      return cost,'alias',overlap,dx,dy,dv,dd

    # No alias overlap: use short-horizon ego-frame kinematics. This is the
    # important rear→side→front sensor hand-off path.
    dx_gate=5.0 + 2.0*dt
    dy_gate=2.35 + 0.25*dt
    dv_gate=4.0
    if dx>dx_gate or dy>dy_gate or dv>dv_gate: return None
    if dd is not None and dd>2.6: return None
    if lane and t.lane and lane!=t.lane and lane not in ('unknown','out') and t.lane not in ('unknown','out'):
      # Lane labels may change during a real lane change; penalize, do not reject.
      lane_pen=0.45
    else:
      lane_pen=0.0
    cost=(dx/dx_gate)**2 + 1.35*(dy/dy_gate)**2 + .35*(dv/dv_gate)**2 + lane_pen
    if dd is not None: cost += .25*(dd/2.6)**2
    return cost,'kinematic',0,dx,dy,dv,dd

  def update(self,objects:list[dict],now_ns:int):
    now_ns=int(now_ns)
    # Drop tracks only after a gap long enough to bridge normal radar hand-offs.
    stale=[tid for tid,t in self.tracks.items() if now_ns-int(t.last_ns)>self.ttl_ns]
    for tid in stale: self.tracks.pop(tid,None)

    valid=[]; passthrough=[]
    for i,o in enumerate(objects):
      if _finite(o.get('x')) is None or _finite(o.get('y')) is None:
        passthrough.append(dict(o))
      else:
        valid.append((i,dict(o)))

    # Global greedy assignment over all valid detection↔track pairs. This avoids
    # letting iteration order decide identity when two cars are close.
    pairs=[]
    candidate_counts={i:0 for i,_ in valid}
    for i,o in valid:
      for tid,t in self.tracks.items():
        c=self._candidate(o,t,now_ns)
        if c is None: continue
        cost,reason,overlap,dx,dy,dv,dd=c
        candidate_counts[i]+=1
        pairs.append((float(cost),-int(overlap),i,tid,reason,dx,dy,dv,dd))
    pairs.sort(key=lambda z:(z[0],z[1],z[2],z[3]))

    assigned_obj={}; used_tracks=set()
    for cost,neg_overlap,i,tid,reason,dx,dy,dv,dd in pairs:
      if i in assigned_obj or tid in used_tracks: continue
      assigned_obj[i]=(tid,reason,cost,-neg_overlap,dx,dy,dv,dd)
      used_tracks.add(tid)

    out=[]
    frame_new=frame_alias=frame_kin=frame_reacq=frame_handoff=frame_ambiguous=0
    matched_existing=0
    for i,o in valid:
      match=assigned_obj.get(i)
      aliases=_aliases(o); domains=_source_domains(o); primary=_primary_domain(domains,o)
      x=float(o['x']); y=float(o['y']); vx=_finite(o.get('vx'))
      road_d=_finite(o.get('road_d')) if o.get('road_projection_valid') else None
      road_s=_finite(o.get('road_s')) if o.get('road_projection_valid') else None
      lane=str(o.get('road_lane')) if o.get('road_lane') is not None else None
      if match is None:
        tid=self.next_id; self.next_id+=1; self.created_total+=1; frame_new+=1
        t=CanonicalTrack(tid=tid,first_ns=now_ns,last_ns=now_ns,x=x,y=y,vx=vx,
                         aliases=set(aliases),source_history=set(domains),last_domains=set(domains),
                         last_primary_domain=primary,road_d=road_d,road_s=road_s,lane=lane)
        self.tracks[tid]=t
        reason='new'; cost=None; overlap=0; gap_ms=0.0; reacquired=False; source_transition=False
      else:
        tid,reason,cost,overlap,dx,dy,dv,dd=match
        t=self.tracks[tid]
        matched_existing+=1
        if reason=='alias': frame_alias+=1
        else: frame_kin+=1
        gap_ms=max(0.0,(now_ns-int(t.last_ns))/1e6)
        reacquired=gap_ms>250.0
        if reacquired:
          t.reacquire_count+=1; self.reacquire_total+=1; frame_reacq+=1
        source_transition=(primary!=t.last_primary_domain and primary!='UNKNOWN' and t.last_primary_domain!='UNKNOWN')
        if source_transition:
          t.handoff_count+=1; self.handoff_total+=1; frame_handoff+=1
        t.age_frames+=1
        t.last_ns=now_ns; t.x=x; t.y=y; t.vx=vx
        t.aliases |= aliases
        # Avoid unbounded growth if a long-lived vehicle accumulates many aliases.
        if len(t.aliases)>64:
          t.aliases=set(sorted(t.aliases)[-64:])
        t.source_history |= domains
        t.last_domains=set(domains)
        t.last_primary_domain=primary
        t.road_d=road_d if road_d is not None else t.road_d
        t.road_s=road_s if road_s is not None else t.road_s
        t.lane=lane if lane is not None else t.lane
      if candidate_counts.get(i,0)>=2: frame_ambiguous+=1

      d=dict(o)
      key=f'{self.prefix}{tid:04d}'
      d.update({
        'vehicle_id':tid,'vehicle_key':key,
        'canonical_id':tid,'canonical_key':key,'canonical_valid':True,
        'canonical_age_frames':int(t.age_frames),
        'canonical_track_duration_s':round((now_ns-int(t.first_ns))/1e9,3),
        'canonical_match_reason':reason,
        'canonical_match_cost':None if cost is None else round(float(cost),4),
        'canonical_alias_overlap':int(overlap),
        'canonical_gap_ms':round(float(gap_ms),1),
        'canonical_reacquired':bool(reacquired),
        'canonical_reacquire_count':int(t.reacquire_count),
        'canonical_domains':sorted(domains),
        'canonical_primary_domain':primary,
        'canonical_domain_history':sorted(t.source_history),
        'canonical_source_transition':bool(source_transition),
        'canonical_handoff_count':int(t.handoff_count),
        'canonical_alias_count':len(t.aliases),
        'canonical_candidate_count':int(candidate_counts.get(i,0)),
      })
      out.append(d)

    out.extend(passthrough)
    out.sort(key=lambda q:float(q.get('x',0.0) or 0.0))
    visible=len(valid)
    stats={
      'visible_tracks':visible,
      'active_tracks':len(self.tracks),
      'matched_existing':matched_existing,
      'new_tracks':frame_new,
      'alias_matches':frame_alias,
      'kinematic_matches':frame_kin,
      'reacquired_tracks':frame_reacq,
      'source_handoffs':frame_handoff,
      'ambiguous_objects':frame_ambiguous,
      'continuity_ratio':round(matched_existing/max(1,visible),4),
      'created_total':self.created_total,
      'handoff_total':self.handoff_total,
      'reacquire_total':self.reacquire_total,
      'ttl_s':round(self.ttl_ns/1e9,3),
      'identity_authority':'canonical360',
      'control_connected':False,
    }
    return out,stats

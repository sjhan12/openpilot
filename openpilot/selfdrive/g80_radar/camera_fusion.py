#!/usr/bin/env python3
from __future__ import annotations
from dataclasses import dataclass
import math

RADAR_TO_CAMERA_M = 1.52
CAM_MATCH_MIN_PROB = 0.35
CAM_ONLY_MIN_PROB = 0.55
CAM_MAX_AGE_NS = 350_000_000

# v17 vehicle-footprint de-duplication.
# A typical passenger car is roughly 4.5-4.9 m long and 1.8-2.0 m wide.
# We use 4.8 x 2.1 m as the maximum cross-sensor envelope, while same-source
# radar tracks use a tighter envelope to avoid joining two real vehicles.
PASSENGER_CAR_LENGTH_M = 4.8
PASSENGER_CAR_WIDTH_M = 2.1
PASSENGER_CAR_VREL_GATE_MPS = 3.0
SAME_CORNER_LENGTH_M = 3.2
SAME_CORNER_WIDTH_M = 1.7
SAME_FRONT_LENGTH_M = 2.6
SAME_FRONT_WIDTH_M = 1.5


def _f0(v, default=None):
  try:
    return float(v[0]) if len(v) else default
  except Exception:
    return default


def _sector(y):
  if y > 1.8: return "FL"
  if y < -1.8: return "FR"
  return "FC"


def _finite(v):
  try:
    x = float(v)
    return x if math.isfinite(x) else None
  except Exception:
    return None


def decode_model_leads(model, log_ns, recv_ns, max_leads=3, min_prob=0.20):
  try:
    model_v_ego = _f0(model.velocity.x, 0.0)
    leads = model.leadsV3
  except Exception:
    return []

  out=[]
  for i,lead in enumerate(leads):
    if i >= max_leads: break
    try: prob=float(lead.prob)
    except Exception: continue
    if prob < min_prob: continue

    lx=_f0(getattr(lead,'x',[]),None)
    ly=_f0(getattr(lead,'y',[]),None)
    lv=_f0(getattr(lead,'v',[]),None)
    la=_f0(getattr(lead,'a',[]),None)
    if lx is None or ly is None or lv is None: continue

    # openpilot radard convention
    x=lx-RADAR_TO_CAMERA_M
    y=-ly
    vx=lv-model_v_ego
    if not (0.0 <= x <= 160.0 and abs(y) <= 8.0 and abs(vx) <= 100.0):
      continue

    xs=_f0(getattr(lead,'xStd',[]),2.0)
    ys=_f0(getattr(lead,'yStd',[]),0.8)
    vs=_f0(getattr(lead,'vStd',[]),2.0)

    out.append({
      'source':'c4_camera','key':f'CV{i}','camera_id':i,
      'x':round(x,3),'y':round(y,3),'vx':round(vx,3),
      'a':None if la is None else round(la,3),
      'prob':round(prob,4),'camera_prob':round(prob,4),
      'x_std':round(float(xs),3),'y_std':round(float(ys),3),'v_std':round(float(vs),3),
      'front_sector':_sector(y),'model_v_ego_mps':round(float(model_v_ego),3),
      'model_log_ns':int(log_ns),'recv_ns':int(recv_ns),
      'camera_only':True,
    })
  return out


def _speed_compatible(a, b, gate=PASSENGER_CAR_VREL_GATE_MPS):
  av=_finite(a.get('vx')); bv=_finite(b.get('vx'))
  return av is None or bv is None or abs(av-bv) <= gate


def _pair_vehicle_compatible(a, b):
  """True when two detections can fit inside one passenger-car footprint.

  Cross-sensor association gets the full 4.8 x 2.1 m envelope.  Same-source
  radar objects use tighter envelopes because two independent tracks from the
  same decoder are more likely to be two real cars when separated in x.
  """
  ax=_finite(a.get('x')); ay=_finite(a.get('y'))
  bx=_finite(b.get('x')); by=_finite(b.get('y'))
  if None in (ax,ay,bx,by) or not _speed_compatible(a,b):
    return False
  dx=abs(ax-bx); dy=abs(ay-by)
  sa=str(a.get('source','')); sb=str(b.get('source',''))
  if sa==sb=='corner_fused':
    return dx <= SAME_CORNER_LENGTH_M and dy <= SAME_CORNER_WIDTH_M
  if sa==sb=='front_track':
    return dx <= SAME_FRONT_LENGTH_M and dy <= SAME_FRONT_WIDTH_M
  if sa==sb=='c4_camera':
    return dx <= PASSENGER_CAR_LENGTH_M and dy <= PASSENGER_CAR_WIDTH_M
  return dx <= PASSENGER_CAR_LENGTH_M and dy <= PASSENGER_CAR_WIDTH_M


def _cluster_envelope_ok(members):
  xs=[_finite(o.get('x')) for o in members]
  ys=[_finite(o.get('y')) for o in members]
  if any(v is None for v in xs+ys):
    return False
  if max(xs)-min(xs) > PASSENGER_CAR_LENGTH_M:
    return False
  if max(ys)-min(ys) > PASSENGER_CAR_WIDTH_M:
    return False
  vs=[_finite(o.get('vx')) for o in members]
  vs=[v for v in vs if v is not None]
  return not vs or max(vs)-min(vs) <= PASSENGER_CAR_VREL_GATE_MPS


def _anchor_score(o):
  # Prefer the best physical track and preserve its x/y/vx rather than averaging
  # a radar center with camera hypotheses or multipath edge reflections.
  s=0.0
  if o.get('source') != 'c4_camera': s += 100.0
  if o.get('scc_teacher_confirmed'): s += 1000.0
  if o.get('camera_confirmed'): s += 300.0
  if o.get('front_link') or o.get('corner_link_id') is not None: s += 250.0
  if o.get('teacher_match'): s += 180.0
  if o.get('source') == 'front_track': s += 120.0
  s += 3.0*min(10,int(o.get('member_count',1) or 1))
  s += min(40,int(o.get('track_consecutive',o.get('age_frames',0)) or 0))
  s += 20.0*float(o.get('camera_prob',o.get('prob',0.0)) or 0.0)
  return s


def _aggregate_vehicle_group(group):
  anchor=max(group,key=lambda o:(_anchor_score(o),str(o.get('key',''))))
  out=dict(anchor)
  keys=sorted({str(o.get('key')) for o in group if o.get('key')})
  sources=sorted({str(o.get('source')) for o in group if o.get('source')})
  xs=[float(o['x']) for o in group]; ys=[float(o['y']) for o in group]

  out['vehicle_anchor_key']=str(anchor.get('key',''))
  out['vehicle_cluster_keys']=keys
  out['vehicle_cluster_sources']=sources
  out['vehicle_member_count']=len(group)
  out['vehicle_duplicates_merged']=max(0,len(group)-1)
  out['vehicle_footprint_merged']=len(group)>1
  out['vehicle_span_x_m']=round(max(xs)-min(xs),3)
  out['vehicle_span_y_m']=round(max(ys)-min(ys),3)
  out['recv_ns']=max(int(o.get('recv_ns',0) or 0) for o in group)

  # Evidence is OR/max aggregated so removing duplicate rectangles never removes
  # a useful camera/SCC/front/rear confirmation carried by another member.
  for k in ('teacher_match','scc_teacher_confirmed','camera_confirmed'):
    if any(bool(o.get(k)) for o in group): out[k]=True
  if any(o.get('front_link') for o in group):
    out['front_link']=next(o.get('front_link') for o in group if o.get('front_link'))
  if any(o.get('corner_link_id') is not None for o in group):
    out['corner_link_id']=next(o.get('corner_link_id') for o in group if o.get('corner_link_id') is not None)

  cam_members=[o for o in group if o.get('camera_confirmed') or o.get('source')=='c4_camera']
  if cam_members:
    best_cam=max(cam_members,key=lambda o:float(o.get('camera_prob',o.get('prob',0.0)) or 0.0))
    out['camera_confirmed']=True
    out['camera_prob']=round(max(float(o.get('camera_prob',o.get('prob',0.0)) or 0.0) for o in cam_members),4)
    for k in ('camera_id','camera_key','camera_match_cost','camera_dx_m','camera_dy_m','camera_dv_mps'):
      if best_cam.get(k) is not None: out[k]=best_cam.get(k)
    hyp=[]
    for o in cam_members:
      vals=o.get('camera_hypothesis_keys')
      if isinstance(vals,list): hyp.extend(str(v) for v in vals)
      elif o.get('source')=='c4_camera' and o.get('key'): hyp.append(str(o.get('key')))
      elif o.get('camera_key'): hyp.append(str(o.get('camera_key')))
    if hyp:
      out['camera_hypothesis_keys']=sorted(set(hyp))
      out['camera_hypothesis_count']=len(out['camera_hypothesis_keys'])

  physical=any(o.get('source')!='c4_camera' for o in group)
  if physical and cam_members:
    out['camera_only']=False
    out['sensor_fusion']='radar+camera'
  elif physical and len(group)>1:
    out['camera_only']=False
    out['sensor_fusion']='radar_vehicle_cluster'
  elif not physical:
    out['camera_only']=True
    out['sensor_fusion']='camera_only'
  return out


def fuse_vehicle_footprints(objects):
  """Collapse overlapping detections into one canonical physical vehicle.

  This is deliberately a *display/shadow sensor-fusion* layer. It does not alter
  raw CAN decoding. Complete-link checks prevent transitive chains from joining
  a line of several vehicles into one large cluster.
  """
  valid=[]; passthrough=[]
  for o in objects:
    d=dict(o)
    if _finite(d.get('x')) is None or _finite(d.get('y')) is None:
      passthrough.append(d)
    else:
      valid.append(d)

  order=sorted(range(len(valid)),key=lambda i:(_anchor_score(valid[i]),str(valid[i].get('key',''))),reverse=True)
  remaining=set(range(len(valid))); groups=[]
  for i in order:
    if i not in remaining: continue
    remaining.remove(i)
    g=[i]
    cand=sorted(list(remaining),key=lambda j:(abs(float(valid[j]['x'])-float(valid[i]['x'])),abs(float(valid[j]['y'])-float(valid[i]['y']))))
    for j in cand:
      if j not in remaining: continue
      if not all(_pair_vehicle_compatible(valid[k],valid[j]) for k in g):
        continue
      if not _cluster_envelope_ok([valid[k] for k in g]+[valid[j]]):
        continue
      g.append(j); remaining.remove(j)
    groups.append(g)

  fused=[_aggregate_vehicle_group([valid[i] for i in g]) for g in groups]
  fused.extend(passthrough)
  fused.sort(key=lambda o:float(o.get('x',0.0)))
  return fused, {
    'vehicle_objects_before':len(objects),
    'vehicle_objects_after':len(fused),
    'vehicle_duplicates_merged':max(0,len(objects)-len(fused)),
    'vehicle_clusters_merged':sum(1 for g in groups if len(g)>1),
    'passenger_car_length_m':PASSENGER_CAR_LENGTH_M,
    'passenger_car_width_m':PASSENGER_CAR_WIDTH_M,
  }



class VehicleFootprintTracker:
  """Assign a persistent physical-vehicle ID after footprint de-duplication."""
  def __init__(self, prefix='V', ttl_s=1.0):
    self.prefix=str(prefix)
    self.ttl_ns=int(float(ttl_s)*1e9)
    self.tracks={}
    self.next_id=1

  @staticmethod
  def _track_cost(o,t,now_ns):
    dt=max(0.0,min(1.0,(int(now_ns)-int(t['last_ns']))/1e9))
    px=float(t['x']) + float(t.get('vx') or 0.0)*dt
    py=float(t['y'])
    dx=abs(float(o['x'])-px); dy=abs(float(o['y'])-py)
    ov=o.get('vx'); tv=t.get('vx')
    dv=0.0 if ov is None or tv is None else abs(float(ov)-float(tv))
    old_keys=set(t.get('member_keys',[])); new_keys=set(o.get('vehicle_cluster_keys',[o.get('key')]))
    overlap=len(old_keys & new_keys)
    # A shared raw/radar member is the strongest identity cue. Otherwise allow
    # a vehicle-sized hand-over between corner/front/camera representations.
    if overlap:
      if dx>7.0 or dy>2.8 or dv>5.0: return None
      return -10.0*overlap + .15*dx + .25*dy + .08*dv
    if dx>5.5 or dy>2.4 or dv>4.0: return None
    return (dx/5.5)**2 + (dy/2.4)**2 + .35*(dv/4.0)**2

  def update(self,objects,now_ns):
    fused,stats=fuse_vehicle_footprints(objects)
    stale=[tid for tid,t in self.tracks.items() if int(now_ns)-int(t['last_ns'])>self.ttl_ns]
    for tid in stale: self.tracks.pop(tid,None)
    available=set(self.tracks)
    out=[]
    # Best-evidence clusters claim prior tracks first.
    for o in sorted(fused,key=lambda q:(_anchor_score(q),-abs(float(q.get('y',0)))),reverse=True):
      best=None
      for tid in available:
        c=self._track_cost(o,self.tracks[tid],now_ns)
        if c is not None and (best is None or c<best[0]): best=(c,tid)
      if best is None:
        tid=self.next_id; self.next_id+=1; first=int(now_ns); age=1
      else:
        tid=best[1]; prev=self.tracks[tid]; first=int(prev.get('first_ns',now_ns)); age=int(prev.get('age_frames',0))+1; available.remove(tid)
      d=dict(o)
      d['vehicle_id']=tid
      d['vehicle_key']=f'{self.prefix}{tid:04d}'
      d['vehicle_age_frames']=age
      d['vehicle_track_duration_ms']=round((int(now_ns)-first)/1e6,1)
      out.append(d)
      self.tracks[tid]={
        'x':d['x'],'y':d['y'],'vx':d.get('vx'),'last_ns':int(now_ns),'first_ns':first,'age_frames':age,
        'member_keys':list(d.get('vehicle_cluster_keys',[d.get('key')]))
      }
    out.sort(key=lambda q:float(q.get('x',0.0)))
    stats=dict(stats,vehicle_tracks_active=len(self.tracks))
    return out,stats

def collapse_camera_hypotheses(camera_leads):
  """leadsV3 CV0/CV1/CV2 are hypotheses, not automatically three vehicles.

  Only hypotheses fitting inside one passenger-car envelope with compatible
  relative speed are collapsed; spatially distinct hypotheses remain separate.
  """
  if not camera_leads:
    return []
  fused,_=fuse_vehicle_footprints([dict(c) for c in camera_leads])
  out=[]
  for c in fused:
    members=c.get('vehicle_cluster_keys',[c.get('key')])
    q=dict(c)
    q['camera_hypothesis_keys']=[str(k) for k in members if k]
    q['camera_hypothesis_count']=len(q['camera_hypothesis_keys'])
    q['camera_hypotheses_merged']=max(0,q['camera_hypothesis_count']-1)
    # The highest-probability member remains the representative key/position.
    q['camera_only']=True
    out.append(q)
  return out


@dataclass
class StickyMatch:
  radar_key: str
  last_ns: int


class CameraRadarFusion:
  def __init__(self):
    self.sticky={}
    self.vehicle_tracker=VehicleFootprintTracker('V')
    self.front_vehicle_tracker=VehicleFootprintTracker('VF')

  @staticmethod
  def _gates(cam):
    x=max(1.0,float(cam['x']))
    xs=max(.8,float(cam.get('x_std',2.0)))
    ys=max(.35,float(cam.get('y_std',.8)))
    vs=max(.8,float(cam.get('v_std',2.0)))
    return min(15.0,max(4.0,2.5*xs,.25*x)), min(3.0,max(1.0,2.5*ys)), min(10.0,max(3.0,2.5*vs))

  def _cost(self,cam,r,now_ns):
    rx=float(r.get('x',1e9)); ry=float(r.get('y',1e9))
    if rx < -.5 or abs(ry) > 6.2: return None
    gx,gy,gv=self._gates(cam)
    dx=rx-float(cam['x']); dy=ry-float(cam['y'])
    if abs(dx)>gx or abs(dy)>gy: return None
    rv=r.get('vx'); dv=None
    if rv is not None:
      dv=float(rv)-float(cam['vx'])
      if abs(dv)>gv: return None
    cost=(dx/gx)**2+(dy/gy)**2
    if dv is not None: cost += .55*(dv/gv)**2
    ck=str(cam.get('key','')); rk=str(r.get('key',''))
    st=self.sticky.get(ck)
    if st and st.radar_key==rk and now_ns-st.last_ns<=600_000_000: cost-=.35
    if r.get('scc_teacher_confirmed') and abs(float(cam['y']))<1.8: cost-=.20
    return cost,dx,dy,dv

  def update(self,radar_objects,camera_leads,now_ns,front_objects=None):
    radar=[dict(o) for o in radar_objects]
    fresh_raw=[dict(c) for c in camera_leads if -50_000_000 <= now_ns-int(c.get('recv_ns',0)) <= CAM_MAX_AGE_NS]
    fresh=collapse_camera_hypotheses(fresh_raw)

    for o in radar:
      for k in ('camera_confirmed','camera_id','camera_prob','camera_key','camera_match_cost',
                'camera_dx_m','camera_dy_m','camera_dv_mps','sensor_fusion',
                'camera_hypothesis_keys','camera_hypothesis_count'):
        o.pop(k,None)

    pairs=[]
    for ci,c in enumerate(fresh):
      if float(c.get('prob',c.get('camera_prob',0))) < CAM_MATCH_MIN_PROB: continue
      for ri,r in enumerate(radar):
        q=self._cost(c,r,now_ns)
        if q is not None:
          cost,dx,dy,dv=q
          pairs.append((cost,ci,ri,dx,dy,dv))
    pairs.sort(key=lambda z:z[0])

    used_c=set();used_r=set();matches=[]
    for cost,ci,ri,dx,dy,dv in pairs:
      if ci in used_c or ri in used_r: continue
      used_c.add(ci);used_r.add(ri)
      c=fresh[ci];r=radar[ri]
      r.update({
        'camera_confirmed':True,'camera_id':c.get('camera_id'),'camera_key':c.get('key'),
        'camera_prob':float(c.get('camera_prob',c.get('prob',0.0)) or 0.0),'camera_match_cost':round(float(cost),4),
        'camera_dx_m':round(float(dx),3),'camera_dy_m':round(float(dy),3),
        'camera_dv_mps':None if dv is None else round(float(dv),3),
        'sensor_fusion':'radar+camera',
        'camera_hypothesis_keys':list(c.get('camera_hypothesis_keys',[c.get('key')])),
        'camera_hypothesis_count':int(c.get('camera_hypothesis_count',1) or 1),
      })
      self.sticky[str(c.get('key',''))]=StickyMatch(str(r.get('key','')),now_ns)
      matches.append({
        'camera_key':c.get('key'),'camera_id':c.get('camera_id'),'camera_prob':float(c.get('camera_prob',c.get('prob',0.0)) or 0.0),
        'camera_hypothesis_keys':list(c.get('camera_hypothesis_keys',[c.get('key')])),
        'camera_hypothesis_count':int(c.get('camera_hypothesis_count',1) or 1),
        'radar_key':r.get('key'),'radar_source':r.get('source'),
        'cost':round(float(cost),4),'dx_m':round(float(dx),3),'dy_m':round(float(dy),3),
        'dv_mps':None if dv is None else round(float(dv),3),
      })

    camera_only=[]
    for ci,c in enumerate(fresh):
      if ci in used_c or float(c.get('camera_prob',c.get('prob',0.0)) or 0.0) < CAM_ONLY_MIN_PROB: continue
      if not (.5 <= float(c['x']) <= 120.0 and abs(float(c['y'])) <= 5.5): continue
      q=dict(c);q.update({'camera_only':True,'camera_confirmed':True,'sensor_fusion':'camera_only'})
      camera_only.append(q)

    for k in [k for k,v in self.sticky.items() if now_ns-v.last_ns>1_500_000_000]:
      self.sticky.pop(k,None)

    pre_vehicle=radar+camera_only
    all_objs,vehicle_stats=self.vehicle_tracker.update(pre_vehicle,now_ns)

    # Keep the FRONT view in the front-radar domain. The fused radar list also
    # contains corner tracks; spatial filtering alone used to copy them here.
    front_source = front_objects if front_objects is not None else [
      o for o in radar if o.get('source') in ('front_track','fr_cmr_reference')]
    front=[]
    for o in front_source:
      if float(o.get('x',-999)) < -.5 or abs(float(o.get('y',999))) > 5.8: continue
      item=dict(o)
      match=next((r for r in radar if r.get('key')==item.get('key') or
                  r.get('front_link')==item.get('front_key',item.get('key'))),None)
      if match and match.get('camera_confirmed'):
        for k in ('camera_confirmed','camera_id','camera_key','camera_prob',
                  'camera_match_cost','camera_dx_m','camera_dy_m','camera_dv_mps','sensor_fusion',
                  'camera_hypothesis_keys','camera_hypothesis_count'):
          if k in match: item[k]=match[k]
      front.append(item)
    front.extend(dict(o) for o in camera_only if abs(float(o.get('y',999)))<=5.8)
    front,front_vehicle_stats=self.front_vehicle_tracker.update(front,now_ns)

    return {
      'sensor_fused_objects':all_objs,
      'front_sensor_objects':front,
      'camera_only_objects':camera_only,
      'camera_matches':matches,
      'stats':{
        'camera_leads_fresh_raw':len(fresh_raw),
        'camera_leads_fresh':len(fresh),
        'camera_hypotheses_merged':max(0,len(fresh_raw)-len(fresh)),
        'camera_radar_matches':len(matches),
        'camera_only_objects':len(camera_only),
        'radar_objects_input':len(radar_objects),
        'sensor_fused_objects':len(all_objs),
        **vehicle_stats,
        'front_vehicle_objects_before':front_vehicle_stats['vehicle_objects_before'],
        'front_vehicle_objects_after':front_vehicle_stats['vehicle_objects_after'],
        'front_vehicle_duplicates_merged':front_vehicle_stats['vehicle_duplicates_merged'],
      },
    }

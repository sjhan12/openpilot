#!/usr/bin/env python3
from __future__ import annotations
from dataclasses import dataclass

RADAR_TO_CAMERA_M = 1.52
CAM_MATCH_MIN_PROB = 0.35
CAM_ONLY_MIN_PROB = 0.55
CAM_MAX_AGE_NS = 350_000_000

def _f0(v, default=None):
  try:
    return float(v[0]) if len(v) else default
  except Exception:
    return default

def _sector(y):
  if y > 1.8: return "FL"
  if y < -1.8: return "FR"
  return "FC"

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

@dataclass
class StickyMatch:
  radar_key: str
  last_ns: int

class CameraRadarFusion:
  def __init__(self):
    self.sticky={}

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

  def update(self,radar_objects,camera_leads,now_ns):
    radar=[dict(o) for o in radar_objects]
    fresh=[dict(c) for c in camera_leads if -50_000_000 <= now_ns-int(c.get('recv_ns',0)) <= CAM_MAX_AGE_NS]

    for o in radar:
      for k in ('camera_confirmed','camera_id','camera_prob','camera_key','camera_match_cost',
                'camera_dx_m','camera_dy_m','camera_dv_mps','sensor_fusion'):
        o.pop(k,None)

    pairs=[]
    for ci,c in enumerate(fresh):
      if float(c.get('prob',0)) < CAM_MATCH_MIN_PROB: continue
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
        'camera_confirmed':True,'camera_id':c['camera_id'],'camera_key':c['key'],
        'camera_prob':c['prob'],'camera_match_cost':round(float(cost),4),
        'camera_dx_m':round(float(dx),3),'camera_dy_m':round(float(dy),3),
        'camera_dv_mps':None if dv is None else round(float(dv),3),
        'sensor_fusion':'radar+camera'
      })
      self.sticky[str(c['key'])]=StickyMatch(str(r.get('key','')),now_ns)
      matches.append({
        'camera_key':c['key'],'camera_id':c['camera_id'],'camera_prob':c['prob'],
        'radar_key':r.get('key'),'radar_source':r.get('source'),
        'cost':round(float(cost),4),'dx_m':round(float(dx),3),'dy_m':round(float(dy),3),
        'dv_mps':None if dv is None else round(float(dv),3),
      })

    camera_only=[]
    for ci,c in enumerate(fresh):
      if ci in used_c or float(c['prob']) < CAM_ONLY_MIN_PROB: continue
      if not (.5 <= float(c['x']) <= 120.0 and abs(float(c['y'])) <= 5.5): continue
      q=dict(c);q.update({'camera_only':True,'camera_confirmed':True,'sensor_fusion':'camera_only'})
      camera_only.append(q)

    for k in [k for k,v in self.sticky.items() if now_ns-v.last_ns>1_500_000_000]:
      self.sticky.pop(k,None)

    all_objs=radar+camera_only
    front=[dict(o) for o in all_objs if float(o.get('x',-999))>=-.5 and abs(float(o.get('y',999)))<=5.8]
    return {
      'sensor_fused_objects':all_objs,
      'front_sensor_objects':front,
      'camera_only_objects':camera_only,
      'camera_matches':matches,
      'stats':{
        'camera_leads_fresh':len(fresh),
        'camera_radar_matches':len(matches),
        'camera_only_objects':len(camera_only),
        'radar_objects_input':len(radar_objects),
        'sensor_fused_objects':len(all_objs),
      },
    }

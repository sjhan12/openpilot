#!/usr/bin/env python3
from __future__ import annotations

def front_sector(y: float)->str:
    if y>1.8:return 'FL'
    if y<-1.8:return 'FR'
    return 'FC'

def build_front_objects(filtered_objects):
    out=[]
    for o in filtered_objects:
        if o.get('source')!='fr_cmr_reference':continue
        d=dict(o)
        d['source']='front_track'
        d['front_sector']=front_sector(float(d['y']))
        d['front_key']=str(d.get('key',''))
        out.append(d)
    return out

def _cost(c,f):
    r=max(abs(float(c['x'])),abs(float(f['x'])))
    gx=min(5.0,1.5+.020*r)
    gy=min(2.8,.9+.012*r)
    dx=abs(float(c['x'])-float(f['x']))
    dy=abs(float(c['y'])-float(f['y']))
    if dx>gx or dy>gy:return None
    cv,fv=c.get('vx'),f.get('vx')
    dv=0.0
    if cv is not None and fv is not None:
        dv=abs(float(cv)-float(fv))
        if dv>3.0:return None
    v=(dx/gx)**2+(dy/gy)**2+.30*(dv/3.0)**2
    if f.get('scc_teacher_confirmed'):
        v*=0.55
    return v

def associate_corner_front(corners,fronts):
    pairs=[]
    for i,c in enumerate(corners):
        if float(c['x'])<-.5:continue
        for j,f in enumerate(fronts):
            v=_cost(c,f)
            if v is not None:pairs.append((v,i,j))
    pairs.sort()
    uc=set();uf=set();ass=[]
    co=[dict(o) for o in corners]
    fo=[dict(o) for o in fronts]
    for cost,i,j in pairs:
        if i in uc or j in uf:continue
        uc.add(i);uf.add(j)
        co[i]['front_link']=fo[j].get('front_key')
        co[i]['front_link_cost']=round(cost,3)
        fo[j]['corner_link_id']=co[i].get('corner_fused_id')
        if fo[j].get('scc_teacher_confirmed'):
            co[i]['scc_teacher_confirmed']=True
            co[i]['scc_teacher_distance_m']=fo[j].get('scc_teacher_distance_m')
            co[i]['scc_teacher_rel_speed_mps']=fo[j].get('scc_teacher_rel_speed_mps')
            co[i]['scc_front_key']=fo[j].get('front_key')
        ass.append({
            'corner_fused_id':co[i].get('corner_fused_id'),
            'corner_key':co[i].get('key'),
            'front_key':fo[j].get('front_key'),
            'cost':round(cost,3),
            'corner_xy':[co[i].get('x'),co[i].get('y')],
            'front_xy':[fo[j].get('x'),fo[j].get('y')],
            'scc_teacher_confirmed':bool(fo[j].get('scc_teacher_confirmed')),
        })
    all_objs=list(co)+[f for j,f in enumerate(fo) if j not in uf]
    return {'corner_objects':co,'front_objects':fo,'associations':ass,'all_fused_objects':all_objs}

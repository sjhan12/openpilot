#!/usr/bin/env python3
from __future__ import annotations
from dataclasses import dataclass
import math


def corner_sector(x: float, y: float) -> str:
    if x > 0.5:
        if y > 1.2: return 'FL'
        if y < -1.2: return 'FR'
        return 'FC'
    if x < -0.5:
        if y > 1.2: return 'RL'
        if y < -1.2: return 'RR'
        return 'RC'
    if y > 1.2: return 'SL'
    if y < -1.2: return 'SR'
    return 'C'


@dataclass
class PairState:
    count: int
    last_ns: int
    mean_dx: float
    mean_dy: float
    residual_ema: float


class CornerFusionTracker:
    """Short-range duplicate fusion for the A/B corner-domain object banks.

    Stage 1: merge nearby fragments from the SAME decoded bank when they move together.
    Stage 2: merge overlapping A/B bank detections.
    Stage 3: keep a persistent corner fused ID over time.

    A/B are decoded object-bank labels, not proven physical FL/FR/RL/RR sensor IDs.
    """
    def __init__(self):
        self.same_pair_hist: dict[tuple[str, str], PairState] = {}
        self.tracks: dict[int, dict] = {}
        self.next_id = 1

    @staticmethod
    def _key(o): return str(o.get('key',''))

    @staticmethod
    def _vx_diff(a,b):
        av,bv=a.get('vx'),b.get('vx')
        if av is None or bv is None: return None
        return abs(float(av)-float(bv))

    @staticmethod
    def _same_gate(a,b):
        r=max(abs(float(a['x'])),abs(float(b['x'])))
        return min(3.0,1.35+0.020*r), min(1.50,0.70+0.010*r)

    @staticmethod
    def _cross_gate(a,b):
        r=max(abs(float(a['x'])),abs(float(b['x'])))
        return min(4.0,1.10+0.018*r), min(2.0,0.75+0.010*r)

    def _stable_same_pair(self,a,b,now_ns):
        if a.get('sensor') != b.get('sensor'): return False
        dx=float(a['x'])-float(b['x']); dy=float(a['y'])-float(b['y'])
        gx,gy=self._same_gate(a,b)
        if abs(dx)>gx or abs(dy)>gy: return False
        dv=self._vx_diff(a,b)
        if dv is not None and dv>2.0: return False

        ka,kb=self._key(a),self._key(b)
        if ka>kb:
            ka,kb=kb,ka; dx,dy=-dx,-dy
        k=(ka,kb)
        st=self.same_pair_hist.get(k)
        if st is None or now_ns-st.last_ns>350_000_000:
            st=PairState(1,now_ns,dx,dy,0.0)
        else:
            residual=math.hypot(dx-st.mean_dx,dy-st.mean_dy)
            alpha=.25
            st=PairState(st.count+1,now_ns,
                         (1-alpha)*st.mean_dx+alpha*dx,
                         (1-alpha)*st.mean_dy+alpha*dy,
                         .70*st.residual_ema+.30*residual)
        self.same_pair_hist[k]=st
        dist=math.hypot(dx,dy)
        if dist<=0.75 and (dv is None or dv<=1.5): return True
        return st.count>=2 and st.residual_ema<=0.65

    @staticmethod
    def _merge(members, local: bool):
        ws=[]
        for o in members:
            w=1.0+min(int(o.get('track_consecutive',0)),12)*.04
            if o.get('teacher_match'): w+=.8
            cc=o.get('coast_count')
            if cc is not None: w/=1.0+.15*min(int(cc),6)
            ws.append(w)
        sw=sum(ws) or 1.0
        x=sum(w*float(o['x']) for w,o in zip(ws,members))/sw
        y=sum(w*float(o['y']) for w,o in zip(ws,members))/sw
        vv=[(w,float(o['vx'])) for w,o in zip(ws,members) if o.get('vx') is not None]
        vx=sum(w*v for w,v in vv)/sum(w for w,_ in vv) if vv else None
        keys=sorted({k for o in members for k in (o.get('member_keys') if isinstance(o.get('member_keys'),list) else [o.get('key')]) if k})
        sensors=sorted({str(o.get('sensor','')) for o in members})
        return {
            'source':'corner_local' if local else 'corner_fused',
            'sensor':'+'.join(sensors),
            'key':('LOCAL:' if local else 'CF:')+'/'.join(keys),
            'x':round(x,3),'y':round(y,3),'vx':None if vx is None else round(vx,3),
            'member_keys':keys,'member_count':len(keys),
            'source_members':[{'key':o.get('key'),'sensor':o.get('sensor'),'x':o.get('x'),'y':o.get('y'),'vx':o.get('vx')} for o in members],
            'teacher_match':any(bool(o.get('teacher_match')) for o in members),
            'track_consecutive':max([int(o.get('track_consecutive',0)) for o in members] or [0]),
            'recv_ns':max([int(o.get('recv_ns',0)) for o in members] or [0]),
            'can_log_ns':max([int(o.get('can_log_ns',0)) for o in members] or [0]),
        }

    def _local_fuse(self, pts, now_ns):
        n=len(pts)
        if n<=1: return [self._merge([o],True) for o in pts]
        parent=list(range(n)); groups=[{i} for i in range(n)]
        def find(i):
            while parent[i]!=i:
                parent[i]=parent[parent[i]]; i=parent[i]
            return i
        def union(i,j):
            ri,rj=find(i),find(j)
            if ri==rj:return
            cand=groups[ri]|groups[rj]
            xs=[float(pts[k]['x']) for k in cand]; ys=[float(pts[k]['y']) for k in cand]
            if max(xs)-min(xs)>3.2 or max(ys)-min(ys)>1.7:return
            parent[rj]=ri; groups[ri]=cand; groups[rj]=set()
        for i in range(n):
            for j in range(i+1,n):
                if self._stable_same_pair(pts[i],pts[j],now_ns): union(i,j)
        comps={}
        for i in range(n): comps.setdefault(find(i),[]).append(i)
        stale=[k for k,v in self.same_pair_hist.items() if now_ns-v.last_ns>1_000_000_000]
        for k in stale:self.same_pair_hist.pop(k,None)
        return [self._merge([pts[i] for i in comp],True) for comp in comps.values()]

    def _cross_cost(self,a,b):
        if a.get('sensor')==b.get('sensor'): return None
        gx,gy=self._cross_gate(a,b)
        dx=abs(float(a['x'])-float(b['x']));dy=abs(float(a['y'])-float(b['y']))
        if dx>gx or dy>gy:return None
        dv=self._vx_diff(a,b)
        if dv is not None and dv>2.5:return None
        return (dx/gx)**2+(dy/gy)**2+(0 if dv is None else .3*(dv/2.5)**2)

    def _cross_fuse(self, locals_):
        pairs=[];used=set();out=[]
        for i in range(len(locals_)):
            for j in range(i+1,len(locals_)):
                c=self._cross_cost(locals_[i],locals_[j])
                if c is not None:pairs.append((c,i,j))
        pairs.sort()
        for _,i,j in pairs:
            if i in used or j in used:continue
            out.append(self._merge([locals_[i],locals_[j]],False));used|={i,j}
        for i,o in enumerate(locals_):
            if i not in used:out.append(self._merge([o],False))
        return out

    @staticmethod
    def _track_score(obj,prev,now_ns):
        overlap=len(set(obj.get('member_keys',[]))&set(prev.get('member_keys',[])))
        dt=max(0.0,min(.5,(now_ns-int(prev['last_ns']))/1e9))
        px=float(prev['x']);py=float(prev['y'])
        if prev.get('vx') is not None:px+=float(prev['vx'])*dt
        dist=math.hypot(float(obj['x'])-px,float(obj['y'])-py)
        ovx,pvx=obj.get('vx'),prev.get('vx')
        dv=0.0 if ovx is None or pvx is None else abs(float(ovx)-float(pvx))
        if overlap:return 100+12*overlap-3*dist-dv
        if dist<=3.0 and dv<=3.0:return 10-2*dist-.5*dv
        return None

    def _assign_ids(self,objects,now_ns):
        stale=[tid for tid,t in self.tracks.items() if now_ns-int(t['last_ns'])>1_000_000_000]
        for tid in stale:self.tracks.pop(tid,None)
        avail=set(self.tracks);res=[]
        for obj in sorted(objects,key=lambda o:o.get('member_count',1),reverse=True):
            best=None
            for tid in avail:
                s=self._track_score(obj,self.tracks[tid],now_ns)
                if s is not None and (best is None or s>best[0]):best=(s,tid)
            if best is None:
                tid=self.next_id;self.next_id+=1;first=now_ns;age=1
            else:
                tid=best[1];prev=self.tracks[tid];first=int(prev.get('first_ns',now_ns));age=int(prev.get('age_frames',0))+1;avail.remove(tid)
            d=dict(obj);d['corner_fused_id']=tid;d['key']=f'C{tid:03d}';d['sector']=corner_sector(float(d['x']),float(d['y']));d['age_frames']=age;d['track_duration_ms']=round((now_ns-first)/1e6,1);res.append(d)
            self.tracks[tid]={'x':d['x'],'y':d['y'],'vx':d.get('vx'),'member_keys':d.get('member_keys',[]),'last_ns':now_ns,'first_ns':first,'age_frames':age}
        return res

    def update(self,filtered_objects,now_ns):
        corner=[dict(o) for o in filtered_objects if o.get('source')=='corner24']
        locals_=self._local_fuse(corner,now_ns)
        fused=self._assign_ids(self._cross_fuse(locals_),now_ns)
        return {
            'corner_fused_objects':fused,
            'corner_local_objects':locals_,
            'stats':{
                'input_corner_points':len(corner),
                'after_same_sensor_local_fusion':len(locals_),
                'after_cross_corner_fusion':len(fused),
                'same_sensor_merged_points':max(0,len(corner)-len(locals_)),
                'cross_source_merged_objects':max(0,len(locals_)-len(fused)),
            }
        }

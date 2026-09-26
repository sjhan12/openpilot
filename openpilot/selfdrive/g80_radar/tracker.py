#!/usr/bin/env python3
from __future__ import annotations
import time, math

LANE_W=3.6

def lane_index(y: float)->int:
    return int(round(y/LANE_W))

def lane_name(i:int)->str:
    return {0:"ego",1:"left1",-1:"right1",2:"left2",-2:"right2"}.get(i,f"lane{i:+d}")

class TrackStore:
    def __init__(self, ttl_s:float=0.75, continuity_s:float=0.30):
        self.ttl_ns=int(ttl_s*1e9)
        self.cont_ns=int(continuity_s*1e9)
        self._tracks={}
    def update(self,obj:dict):
        k=obj["key"];now=int(obj["recv_ns"]);prev=self._tracks.get(k)
        if prev is not None and now-int(prev["recv_ns"])<=self.cont_ns:
            consecutive=int(prev.get("track_consecutive",1))+1
            first_seen=int(prev.get("track_first_seen_ns",now))
        else:
            consecutive=1;first_seen=now
        obj=dict(obj)
        obj["track_consecutive"]=consecutive
        obj["track_first_seen_ns"]=first_seen
        obj["track_duration_ms"]=round((now-first_seen)/1e6,1)
        self._tracks[k]=obj
    def snapshot(self,now_recv_ns=None):
        if now_recv_ns is None: now_recv_ns=time.monotonic_ns()
        stale=[k for k,v in self._tracks.items() if now_recv_ns-int(v["recv_ns"])>self.ttl_ns]
        for k in stale:self._tracks.pop(k,None)
        out=[]
        for v in self._tracks.values():
            d=dict(v);d["display_age_ms"]=round((now_recv_ns-int(v["recv_ns"]))/1e6,1);out.append(d)
        return out

def _teacher_for_side(teacher_rear,left:bool):
    target="LR" if left else "RR"
    for t in teacher_rear:
        if t.get("sector")==target and t.get("teacher_usable"):
            return float(t["distance_candidate_m"])
    return None

def mark_teacher_matches(objects,teacher_rear):
    out=[]
    for o in objects:
        d=dict(o);d["teacher_match"]=False;d["teacher_error_m"]=None
        if d.get("source")=="corner24" and float(d["x"])<-0.5 and abs(float(d["y"]))>0.8:
            td=_teacher_for_side(teacher_rear,float(d["y"])>0)
            if td is not None:
                vx=d.get("vx");pred=-float(d["x"])
                if vx is not None: pred+=0.20*float(vx)
                err=abs(pred-td);d["teacher_error_m"]=round(err,3)
                if err<=1.0:d["teacher_match"]=True
        out.append(d)
    return out

def _corner_pass(o):
    x=float(o["x"]);y=float(o["y"]);vx=o.get("vx")
    if not (-60<=x<=100 and abs(y)<=10.8):return False
    if vx is not None and abs(float(vx))>80:return False
    cc=o.get("coast_count")
    if cc is not None and int(cc)>3:return False
    if bool(o.get("teacher_match")):return True
    return int(o.get("track_consecutive",0))>=3

def _ref_pass(o):
    return int(o.get("track_consecutive",0))>=2 and 0<=float(o["x"])<=160 and abs(float(o["y"]))<=12

def _score(o):
    s=float(o.get("track_consecutive",0))
    if o.get("teacher_match"):s+=100
    cc=o.get("coast_count")
    if cc is not None:s-=min(float(cc),10)*2
    if o.get("source")=="corner24":s+=10
    return s

def _near(a,b,r=1.2):
    dx=float(a["x"])-float(b["x"]);dy=float(a["y"])-float(b["y"])
    if dx*dx+dy*dy>r*r:return False
    av=a.get("vx");bv=b.get("vx")
    if av is not None and bv is not None and abs(float(av)-float(bv))>4:return False
    return True

def dedup(objects):
    kept=[]
    for o in sorted(objects,key=_score,reverse=True):
        if any(_near(o,k) for k in kept):continue
        kept.append(o)
    return kept

def filtered_objects(raw,teacher_rear):
    marked=mark_teacher_matches(raw,teacher_rear)
    cand=[]
    for o in marked:
        src=o.get("source")
        if src=="corner24" and _corner_pass(o):cand.append(o)
        elif src=="fr_cmr_reference" and _ref_pass(o):cand.append(o)
        # Orange front-group candidate remains RAW-only.
    return dedup(cand)

def occupied_zones(objects):
    zones={k:{"occupied":False,"nearest":None} for k in ("left2","left1","ego","right1","right2")}
    for o in objects:
        # V22: forward targets outside the C4 path horizon are intentionally
        # unclassified. Do not re-introduce a raw-y lane label here.
        if o.get("road_lane_source") in ("c4_path_out_of_range", "c4_path_no_projection"):
            continue
        if o.get("road_lane") in zones:
            name=o.get("road_lane")
        else:
            rd=o.get("road_d")
            if rd is None:
                rd=o.get("y")
            try:
                name=lane_name(lane_index(float(rd)))
            except Exception:
                continue
        if name not in zones:continue
        x=float(o["x"])
        if -30<=x<=60:
            zones[name]["occupied"]=True
            cur=zones[name]["nearest"]
            if cur is None or abs(x)<abs(float(cur["x"])):zones[name]["nearest"]=o
    return zones

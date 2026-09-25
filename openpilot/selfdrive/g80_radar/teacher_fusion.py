#!/usr/bin/env python3
from __future__ import annotations
from dataclasses import dataclass

SCC_CONTROL_ADDR = 0x1A0
DEFAULT_SCC_BUS = 2

def bits_le(data: bytes, start: int, width: int) -> int:
    return (int.from_bytes(data, "little") >> start) & ((1 << width) - 1)

def decode_scc_teacher(data: bytes, can_log_ns: int, recv_ns: int, bus: int):
    if len(data) != 32:
        return None
    distance = bits_le(data, 24, 11) * 0.1
    rel_speed = bits_le(data, 35, 9) * 0.1 - 16.4
    obj_valid = bits_le(data, 46, 1)
    main_mode = bits_le(data, 66, 1)
    acc_mode = bits_le(data, 68, 3)
    obj_state = bits_le(data, 108, 3)

    active = main_mode == 1 and acc_mode in (1, 2)
    selected = obj_valid == 0 and obj_state in (1, 2)
    usable = active and selected and 0.5 <= distance <= 200.0

    return {
        "source": "scc_control_teacher",
        "bus": int(bus),
        "address": SCC_CONTROL_ADDR,
        "distance_m": round(distance, 3),
        "rel_speed_mps": round(rel_speed, 3),
        "obj_valid_raw": int(obj_valid),
        "scc_obj_sta": int(obj_state),
        "main_mode_acc": int(main_mode),
        "acc_mode": int(acc_mode),
        "active": bool(active),
        "lead_selected": bool(selected),
        "teacher_usable": bool(usable),
        "can_log_ns": int(can_log_ns),
        "recv_ns": int(recv_ns),
        "transport_lag_ms": round((recv_ns-can_log_ns)/1e6, 3),
    }

@dataclass
class MatchState:
    key: str | None = None
    streak: int = 0
    last_ns: int = 0

class SccFrontTeacherMatcher:
    def __init__(self):
        self.state = MatchState()
        self.distance_bias_m = 0.0
        self.bias_samples = 0

    @staticmethod
    def key(o):
        return str(o.get("front_key") or o.get("key") or "")

    def score(self, o, teacher):
        x=float(o["x"]); y=float(o["y"]); vx=o.get("vx")
        target=float(teacher["distance_m"])+self.distance_bias_m
        dx=x-target
        dv=None if vx is None else float(vx)-float(teacher["rel_speed_mps"])
        if abs(y)>2.4 or abs(dx)>5.0:
            return None
        if dv is not None and abs(dv)>3.5:
            return None
        cost=(dx/3.0)**2+(abs(y)/1.8)**2
        if dv is not None:
            cost += 0.65*(dv/2.5)**2
        if self.key(o)==self.state.key:
            cost -= 0.35
        return cost,dx,dv,target

    def update(self, fronts, teacher, now_ns):
        out=[dict(o) for o in fronts]
        for o in out:
            o["scc_teacher_candidate"]=False
            o["scc_teacher_confirmed"]=False
            o["scc_teacher_distance_error_m"]=None
            o["scc_teacher_speed_error_mps"]=None
            o["scc_teacher_cost"]=None

        status={
            "usable":False,"matched":False,"confirmed":False,
            "front_key":None,"streak":0,
            "distance_bias_m":round(self.distance_bias_m,3),
            "distance_error_m":None,"speed_error_mps":None,"match_cost":None,
        }

        if not teacher or not teacher.get("teacher_usable"):
            self.state=MatchState()
            return out,status

        age=now_ns-int(teacher.get("recv_ns",now_ns))
        if age < -50_000_000 or age > 300_000_000:
            self.state=MatchState()
            status["teacher_stale"]=True
            return out,status

        status["usable"]=True
        cand=[]
        for i,o in enumerate(out):
            s=self.score(o,teacher)
            if s is not None:
                cand.append((s[0],i,s))
        if not cand:
            self.state=MatchState()
            return out,status

        cand.sort(key=lambda x:x[0])
        cost,idx,s=cand[0]
        _,dx,dv,target=s
        key=self.key(out[idx])

        if key==self.state.key and now_ns-self.state.last_ns<=400_000_000:
            streak=self.state.streak+1
        else:
            streak=1
        self.state=MatchState(key,streak,now_ns)

        tight=abs(dx)<=1.0 and abs(float(out[idx]["y"]))<=1.2 and (dv is None or abs(dv)<=1.0)
        confirmed=bool(tight or streak>=2)

        o=out[idx]
        o["scc_teacher_candidate"]=True
        o["scc_teacher_confirmed"]=confirmed
        o["scc_teacher_distance_error_m"]=round(dx,3)
        o["scc_teacher_speed_error_mps"]=None if dv is None else round(dv,3)
        o["scc_teacher_cost"]=round(cost,3)
        o["scc_teacher_distance_m"]=teacher["distance_m"]
        o["scc_teacher_rel_speed_mps"]=teacher["rel_speed_mps"]

        if confirmed and abs(dx)<=3.0:
            alpha=0.08 if self.bias_samples>=5 else 0.20
            nb=(1-alpha)*self.distance_bias_m+alpha*(float(o["x"])-float(teacher["distance_m"]))
            self.distance_bias_m=max(-5.0,min(5.0,nb))
            self.bias_samples+=1

        status.update({
            "matched":True,"confirmed":confirmed,"front_key":key,"streak":streak,
            "distance_bias_m":round(self.distance_bias_m,3),
            "distance_error_m":round(dx,3),
            "speed_error_mps":None if dv is None else round(dv,3),
            "match_cost":round(cost,3),
            "aligned_teacher_distance_m":round(float(teacher["distance_m"])+self.distance_bias_m,3),
        })
        return out,status


def choose_scc_teacher(by_bus: dict, preferred_bus: int | None, now_ns: int, fresh_ns: int = 500_000_000):
    """
    Select SCC teacher:
      1) preferred bus if fresh + usable
      2) freshest usable any bus
      3) preferred bus fresh raw
      4) freshest raw any bus

    Raw/inactive state is returned intentionally so the UI can show what
    SCC_CONTROL is transmitting even when strict validity is false.
    """
    items = []
    for bus, t in by_bus.items():
        if not t:
            continue
        age = now_ns - int(t.get("recv_ns", 0))
        if -50_000_000 <= age <= fresh_ns:
            items.append((int(bus), t, age))
    if not items:
        return None

    if preferred_bus is not None:
        for bus, t, _ in items:
            if bus == preferred_bus and t.get("teacher_usable"):
                return dict(t)

    usable = [(bus, t, age) for bus, t, age in items if t.get("teacher_usable")]
    if usable:
        usable.sort(key=lambda x: x[2])
        return dict(usable[0][1])

    if preferred_bus is not None:
        for bus, t, _ in items:
            if bus == preferred_bus:
                return dict(t)

    items.sort(key=lambda x: x[2])
    return dict(items[0][1])

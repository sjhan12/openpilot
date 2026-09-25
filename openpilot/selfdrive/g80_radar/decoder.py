#!/usr/bin/env python3
"""
G80 RG3 2021 radar decoder - v5 FUSED build.

Receive-only decoder.
A/B corner empirical mapping:
  x_display = x_raw * 0.1 - 2.1 m
  y         = signed11 * 0.1 m
  vx        = signed 10/11 candidate * 0.1 m/s
"""
from __future__ import annotations
from dataclasses import dataclass, asdict
from typing import Optional
import math

CORNER_A = set(range(0x241, 0x250))
CORNER_B = set(range(0x279, 0x288))
FRONT_GROUP1 = set(range(0x210, 0x220))
FR_CMR = {0x180,0x181,0x182,0x183,0x184,0x1B6,0x1B7,0x1B8,0x1B9,0x1FB}
TEACHER_OFFSET_M = 2.1

def bits_le(data: bytes, start: int, width: int) -> int:
    return (int.from_bytes(data, "little") >> start) & ((1 << width) - 1)

def sgn(v: int, width: int) -> int:
    return v - (1 << width) if v & (1 << (width - 1)) else v

@dataclass(slots=True)
class RadarObject:
    source: str
    sensor: str
    key: str
    x: float
    y: float
    vx: Optional[float]
    vy: Optional[float]
    ax: Optional[float]
    quality: Optional[float]
    cls: Optional[int]
    age: Optional[int]
    can_log_ns: int
    recv_ns: int
    confidence: str
    raw_address: int
    raw_slot: int
    coasting: Optional[bool] = None
    updated_count: Optional[int] = None
    coast_count: Optional[int] = None
    stationary_candidate: Optional[bool] = None

    def to_dict(self):
        d = asdict(self)
        d["range"] = round(math.hypot(self.x, self.y), 3)
        d["side"] = "L" if self.y > 0.8 else ("R" if self.y < -0.8 else "C")
        d["long"] = "F" if self.x > 0.5 else ("R" if self.x < -0.5 else "A")
        d["transport_lag_ms"] = round((self.recv_ns - self.can_log_ns) / 1e6, 3)
        return d

def decode_corner24(address: int, data: bytes, can_log_ns: int, recv_ns: int) -> Optional[RadarObject]:
    if address not in CORNER_A and address not in CORNER_B:
        return None
    if len(data) != 24:
        return None
    xr = bits_le(data, 64, 12)
    if xr == 0x7FF or not any(data[3:8]):
        return None

    bank = "A" if address in CORNER_A else "B"
    base = 0x241 if bank == "A" else 0x279
    slot = address - base + 1

    x_raw_m = sgn(xr, 12) * 0.1
    x = x_raw_m - TEACHER_OFFSET_M
    y = sgn(bits_le(data, 76, 11), 11) * 0.1

    v10 = sgn(bits_le(data, 87, 10), 10) * 0.1
    v11 = sgn(bits_le(data, 87, 11), 11) * 0.1
    vx = v11 if abs(v11 - v10) < 1e-9 else None

    oid = data[3]
    return RadarObject(
        source="corner24",
        sensor="corner_A" if bank == "A" else "corner_B",
        key=f"C{bank}:{oid:02X}",
        x=round(x,3), y=round(y,3),
        vx=None if vx is None else round(vx,3),
        vy=None, ax=None, quality=None, cls=None, age=None,
        can_log_ns=can_log_ns, recv_ns=recv_ns,
        confidence="v4_empirical_xy",
        raw_address=address, raw_slot=slot,
        coasting=bool(bits_le(data,61,1)),
        updated_count=bits_le(data,152,4),
        coast_count=bits_le(data,156,4),
        stationary_candidate=bool(bits_le(data,58,1)),
    )

def decode_front_group1_candidate(address: int, data: bytes, can_log_ns: int, recv_ns: int) -> list[RadarObject]:
    if address not in FRONT_GROUP1 or len(data) != 32:
        return []
    out=[]
    for sub in (0,1):
        off=sub*128
        valid=bits_le(data,off+32,8)
        oid=bits_le(data,off+42,8)
        x=bits_le(data,off+64,13)*0.05
        y=sgn(bits_le(data,off+78,11),11)*0.05
        vx=sgn(bits_le(data,off+91,11),11)*0.05
        vy=sgn(bits_le(data,off+104,9),9)*0.05
        ax=sgn(bits_le(data,off+115,9),9)*0.1
        if valid == 0 or not (0 <= x <= 220 and abs(y) <= 30):
            continue
        slot=(address-0x210)*2+sub
        out.append(RadarObject(
            source="front_group1_candidate", sensor="front_candidate",
            key=f"F?:{address:03X}:{sub}:{oid:02X}",
            x=round(x,3), y=round(y,3), vx=round(vx,3),
            vy=round(vy,3), ax=round(ax,3),
            quality=None, cls=None, age=valid,
            can_log_ns=can_log_ns, recv_ns=recv_ns,
            confidence="candidate_display_only",
            raw_address=address, raw_slot=slot))
    return out

def decode_fr_cmr_reference(address: int, data: bytes, can_log_ns: int, recv_ns: int) -> list[RadarObject]:
    if address not in FR_CMR or len(data) != 32:
        return []
    ordered=(0x180,0x181,0x182,0x183,0x184,0x1B6,0x1B7,0x1B8,0x1B9,0x1FB)
    idx=ordered.index(address)
    out=[]
    for sub in (0,1):
        off=sub*128
        quality=bits_le(data,off+24,7)
        x=bits_le(data,off+64,13)*0.05
        y=bits_le(data,off+78,12)*0.05-102.4
        vx=bits_le(data,off+91,12)*0.05-100.0
        if not (quality > 0 and 0 <= x < 180 and abs(y) < 40 and vx > -99):
            continue
        oid=bits_le(data,off+44,7)
        cls=bits_le(data,off+60,3)
        out.append(RadarObject(
            source="fr_cmr_reference", sensor="adas_reference",
            key=f"A:{oid:02X}",
            x=round(x,3), y=round(y,3), vx=round(vx,3),
            vy=None, ax=None, quality=float(quality), cls=cls, age=None,
            can_log_ns=can_log_ns, recv_ns=recv_ns,
            confidence="reference_not_fused",
            raw_address=address, raw_slot=idx*2+sub))
    return out

def decode_rear_teacher_1ea(data: bytes) -> list[dict]:
    if len(data)!=32:
        return []
    out=[]
    for sector,dist_start,state_start in (("LR",139,160),("RR",163,184)):
        status=bits_le(data,state_start,3)
        distance=bits_le(data,dist_start,8)*0.1
        out.append({
            "sector":sector,
            "status_raw":status,
            "distance_candidate_m":round(distance,3),
            "teacher_usable":bool(status==1 and 0.5<=distance<=19.5),
            "teacher_only":True})
    return out

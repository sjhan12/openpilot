#!/usr/bin/env python3
"""Approximate V44 FG12 2D-conflict policy replay for V43 shadow JSONL logs.

Uses logged same-key V43 observations. It does NOT rerun raw CAN decoding/fusion,
so this is a policy regression, not ground-truth safety validation.
"""
from __future__ import annotations
import argparse, gzip, json, math, zlib
from collections import Counter

LONG_REAR=-8.0
LONG_FRONT=3.2
LAT_HALF=1.80
DANGER_S=3.0
CHECK_S=5.0
FRESH_MS=350.0
NEAR_M=10.0
CONFIRM_S=0.15
CONFIRM_COUNT=2
CORE_URGENT_S=0.75
UNCERTAIN_URGENT_S=1.0


def interval_band(x, v, lo, hi, eps=1e-6):
  x=float(x); v=float(v)
  if lo <= x <= hi:
    if abs(v) < eps: return (0.0,None)
    ts=[(lo-x)/v,(hi-x)/v]
    ex=[t for t in ts if t>eps]
    return (0.0,min(ex) if ex else None)
  if abs(v)<eps: return None
  t1=(lo-x)/v; t2=(hi-x)/v
  a,b=min(t1,t2),max(t1,t2)
  if b<0: return None
  e=max(0.0,a)
  return (e,b) if e<=b else None


def intersect(a,b):
  if a is None or b is None: return None
  a0,a1=a; b0,b1=b
  ea=float('inf') if a1 is None else float(a1)
  eb=float('inf') if b1 is None else float(b1)
  st=max(float(a0),float(b0)); en=min(ea,eb)
  return (st,None if math.isinf(en) else en) if st<=en else None


def old_label(side):
  x=str((side.get('decision') or {}).get('label') or '')
  if x.startswith('DANGER'): return 'DANGER'
  if x.startswith('SAFE'): return 'SAFE'
  if x.startswith('NO LANE'): return 'NO LANE'
  if x.startswith('CHECK ROAD'): return 'CHECK ROAD'
  return 'CHECK'


def classify(side, side_name):
  lane=str((side.get('lane_availability') or {}).get('status') or 'UNCERTAIN').upper()
  ev=side.get('fg12_evidence') or side.get('fg11_evidence') or side.get('fg10_evidence') or side.get('fg9_evidence') or {}
  obs=ev.get('observations') or []
  center=3.6 if side_name=='left' else -3.6
  hard=False; urgent_direct=False; watch=False
  for o in obs:
    s=o.get('s_now_m'); vr=o.get('relative_s_dot_mps'); d=o.get('d_now_m'); vd=o.get('d_dot_mps',0.0)
    if None in (s,vr,d): continue
    long_iv=interval_band(float(s),float(vr),LONG_REAR,LONG_FRONT,0.2)
    lat_iv=interval_band(float(d),float(vd or 0.0),center-LAT_HALF,center+LAT_HALF)
    ci=intersect(long_iv,lat_iv)
    ce=ci[0] if ci else None
    fresh=o.get('source_age_ms') is None or float(o.get('source_age_ms'))<=FRESH_MS
    current_core=bool(o.get('current_core'))
    boundary=bool(o.get('current_boundary') and not current_core)
    incoming=bool(o.get('stable_incoming'))
    relevant=current_core or boundary or incoming
    count=int(o.get('tts_persistence_count') or 0)
    age=float(o.get('tts_persistence_s') or 0.0)
    track=float(o.get('track_duration_s') or 0.0)
    conflict_now=bool(ci is not None and ce<=1e-9)
    cand=bool(fresh and relevant and ce is not None and ce<=DANGER_S and (track>=0.20 or conflict_now))
    core_urgent=bool(current_core and ce is not None and ce<=CORE_URGENT_S)
    confirmed=bool(cand and (conflict_now or core_urgent or (count>=CONFIRM_COUNT and age>=CONFIRM_S)))
    if confirmed:
      hard=True
      if current_core and ce is not None and ce<=UNCERTAIN_URGENT_S: urgent_direct=True
    elif relevant and ce is not None and ce<=CHECK_S:
      watch=True
    elif relevant and o.get('current_gap_m') is not None and float(o.get('current_gap_m'))<=NEAR_M:
      watch=True
    elif o.get('confirmed_prediction') or o.get('uncertain_prediction') or o.get('edge_watch') or incoming:
      watch=True
  if lane=='ABSENT': return 'NO LANE'
  if lane!='CONFIRMED': return 'DANGER' if urgent_direct else 'CHECK ROAD'
  if hard: return 'DANGER'
  if watch: return 'CHECK'
  return 'SAFE'


def iter_samples(path):
  if path.endswith('.part'):
    d=zlib.decompressobj(16+zlib.MAX_WBITS); buf=b''
    with open(path,'rb') as f:
      while True:
        c=f.read(1<<20)
        if not c: break
        try: buf+=d.decompress(c)
        except zlib.error: break
        while b'\n' in buf:
          line,buf=buf.split(b'\n',1)
          try: o=json.loads(line)
          except Exception: continue
          if o.get('type')=='sample': yield o
    return
  with gzip.open(path,'rt',errors='replace') as f:
    for line in f:
      try:o=json.loads(line)
      except Exception:continue
      if o.get('type')=='sample':yield o


def main():
  ap=argparse.ArgumentParser(); ap.add_argument('logs',nargs='+'); args=ap.parse_args()
  alltab=Counter(); activetab=Counter(); samples=0; active=0
  for path in args.logs:
    for o in iter_samples(path):
      samples+=1; fg=o.get('future_gap') or {}; ego=fg.get('ego') or {}
      for side_name in ('left','right'):
        side=fg.get(side_name) or {}; old=old_label(side); new=classify(side,side_name); alltab[(old,new)]+=1
        a=((side_name=='left' and ego.get('left_blinker') and not ego.get('right_blinker')) or
           (side_name=='right' and ego.get('right_blinker') and not ego.get('left_blinker')))
        if a: active+=1; activetab[(old,new)]+=1
  labels=['SAFE','CHECK','CHECK ROAD','DANGER','NO LANE']
  print(f'samples={samples} side_evaluations={samples*2} active_side_evaluations={active}')
  for title,tab in [('ALL',alltab),('ACTIVE',activetab)]:
    print(title+' old -> V44-FG12-approx')
    for old in labels:
      vals=' '.join(f'{new}:{tab[(old,new)]}' for new in labels if tab[(old,new)])
      if vals: print(f'  {old}: {vals}')

if __name__=='__main__': main()

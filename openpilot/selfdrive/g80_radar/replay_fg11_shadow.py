#!/usr/bin/env python3
"""Approximate FG11 Time-To-Side policy replay for V39-V42 shadow JSONL logs.

This utility does not re-run radar decoding/fusion. It re-scores already logged
same-key evidence to compare the old displayed decision with the V43 speed-based
3-second Time-To-Side policy. It is therefore a policy regression tool, not
ground-truth accuracy validation.
"""
from __future__ import annotations
import argparse, gzip, json, math, os
from collections import Counter

REAR_EDGE_M=-8.0
FRONT_EDGE_M=3.2
DANGER_S=3.0
CHECK_S=5.0
FRESH_MS=350.0
NEAR_WATCH_M=10.0


def _interval(s: float, v: float):
  s=float(s); v=float(v)
  if REAR_EDGE_M <= s <= FRONT_EDGE_M:
    if abs(v)<0.2: return (0.0,None)
    ts=[(REAR_EDGE_M-s)/v,(FRONT_EDGE_M-s)/v]
    exits=[t for t in ts if t>1e-6]
    return (0.0,min(exits) if exits else None)
  if abs(v)<0.2: return None
  t1=(REAR_EDGE_M-s)/v; t2=(FRONT_EDGE_M-s)/v
  a,b=min(t1,t2),max(t1,t2)
  if b<0: return None
  e=max(0.0,a)
  return (e,b) if e<=b else None


def _old_label(side: dict) -> str:
  x=(side.get('decision') or {}).get('label') or ''
  if str(x).startswith('DANGER'): return 'DANGER'
  if str(x).startswith('SAFE'): return 'SAFE'
  if str(x).startswith('NO LANE'): return 'NO LANE'
  return 'CHECK'


def classify(side: dict) -> str:
  lane=str((side.get('lane_availability') or {}).get('status') or 'UNCERTAIN').upper()
  ev=side.get('fg11_evidence') or side.get('fg10_evidence') or side.get('fg9_evidence') or {}
  obs=ev.get('observations') or []
  hard=False; watch=False
  for x in obs:
    s=x.get('s_now_m'); v=x.get('relative_s_dot_mps')
    if s is None or v is None: continue
    fresh=(x.get('source_age_ms') is None or float(x.get('source_age_ms'))<=FRESH_MS)
    relevant=bool(x.get('current_core') or x.get('stable_incoming'))
    boundary=bool(x.get('current_boundary') and not x.get('current_core'))
    iv=_interval(float(s),float(v)); entry=iv[0] if iv else None
    if lane=='CONFIRMED' and relevant and fresh and entry is not None and entry<=DANGER_S:
      hard=True
    elif relevant or boundary or x.get('forecast_core') or x.get('stable_incoming'):
      gap=x.get('current_gap_m')
      if entry is not None and entry<=CHECK_S: watch=True
      elif gap is not None and float(gap)<=NEAR_WATCH_M: watch=True
      elif x.get('predicted_severe') or x.get('uncertain_prediction'): watch=True
  if lane=='ABSENT': return 'NO LANE'
  if lane!='CONFIRMED': return 'CHECK'
  if hard: return 'DANGER'
  if watch: return 'CHECK'
  return 'SAFE'


def iter_samples(path):
  opener=gzip.open if path.endswith('.gz') else open
  try:
    with opener(path,'rt',errors='replace') as f:
      for line in f:
        try: o=json.loads(line)
        except Exception: continue
        if o.get('type')=='sample': yield o
  except Exception as e:
    print(f'WARN {path}: {e}')


def main():
  ap=argparse.ArgumentParser()
  ap.add_argument('logs',nargs='+')
  args=ap.parse_args()
  table=Counter(); active_table=Counter(); samples=0; active_eval=0
  for p in args.logs:
    for o in iter_samples(p):
      samples+=1
      fg=o.get('future_gap') or {}; ego=fg.get('ego') or {}
      for side_name in ('left','right'):
        side=fg.get(side_name) or {}
        old=_old_label(side); new=classify(side); table[(old,new)]+=1
        active=((side_name=='left' and ego.get('left_blinker') and not ego.get('right_blinker')) or
                (side_name=='right' and ego.get('right_blinker') and not ego.get('left_blinker')))
        if active:
          active_eval+=1; active_table[(old,new)]+=1
  labels=['SAFE','CHECK','DANGER','NO LANE']
  print(f'samples={samples} side_evaluations={samples*2} active_side_evaluations={active_eval}')
  print('ALL old -> V43-approx')
  for old in labels:
    vals=' '.join(f'{new}:{table[(old,new)]}' for new in labels if table[(old,new)])
    if vals: print(f'  {old}: {vals}')
  print('ACTIVE old -> V43-approx')
  for old in labels:
    vals=' '.join(f'{new}:{active_table[(old,new)]}' for new in labels if active_table[(old,new)])
    if vals: print(f'  {old}: {vals}')

if __name__=='__main__': main()

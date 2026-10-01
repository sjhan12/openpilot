#!/usr/bin/env python3
"""Standalone synthetic FG11 policy test. Run from source tree with openpilot available."""
from openpilot.selfdrive.g80_radar.future_gap import FutureGapEvaluator


def run(obj, road_model=None, side='left', blink=True):
  e=FutureGapEvaluator(); r=None
  for i in range(7):
    now=int((10.0+i*0.25)*1e9)
    r=e.update([obj],v_ego=20.0,left_blinker=(blink and side=='left'),right_blinker=(blink and side=='right'),
               now_ns=now,road_model=road_model or {})
  return r[side]

if __name__=='__main__':
  print('This test expects the installed road_geometry/model environment; see V43_TEST_PLAN_KO.txt for expected cases.')

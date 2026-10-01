#!/usr/bin/env python3
"""Re-evaluate logged evidence with shipped FG13; not CAN/fusion or HUD replay."""
import argparse
import collections
import json
import pathlib
import sys
import types

if __package__ in (None, ''):
    root=pathlib.Path(__file__).resolve().parent
    for name in ('openpilot','openpilot.selfdrive','openpilot.selfdrive.g80_radar'):
        if name not in sys.modules:
            m=types.ModuleType(name);m.__path__=[str(root)];sys.modules[name]=m
from openpilot.selfdrive.g80_radar.future_gap import FutureGapEvaluator
from shadow_log_reader import iter_records


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('logs',nargs='+');args=ap.parse_args()
    counts=collections.Counter();transitions={s:collections.Counter() for s in ('left','right')}
    seen=set();reports=[];e=FutureGapEvaluator()
    for path in args.logs:
        report={'file':str(path)};header={}
        for r in iter_records(path,report):
            if r.get('type')=='header':header=r;continue
            if r.get('type')!='sample':continue
            identity=(header.get('instance_id') or str(path),r.get('mono_ns'))
            if identity in seen:counts['duplicates']+=1;continue
            seen.add(identity);counts['unique_samples']+=1
            for name in transitions:
                side=(r.get('future_gap') or {}).get(name)
                if not side:continue
                old=side.get('decision_raw') or {}
                new=e._apply_road_lane_gate(e._decision(side),side.get('lane_availability'),side)
                label=lambda d:d.get('label_override') or d.get('state','MISSING')
                transitions[name][label(old)+' -> '+label(new)]+=1
        reports.append(report)
    print(json.dumps(dict(counts=counts,transitions=transitions,files=reports,
                         limitation='Logged-evidence policy replay only; no raw CAN, perception, display debounce or intent replay.'),ensure_ascii=False,indent=2))

if __name__=='__main__':main()

#!/usr/bin/env python3
"""Read-only live monitor diagnostics. Run parked with service running."""
import json
import urllib.request

with urllib.request.urlopen('http://127.0.0.1:28992/state',timeout=3) as response:
  state=json.load(response)
fg=state.get('future_gap') or {}
print(json.dumps(dict(runtime=state.get('runtime_versions'),
                     runtime_mismatch=state.get('runtime_mismatch'),
                     bsd=fg.get('bsd'),health=fg.get('monitor_health'),
                     left=(fg.get('left') or {}).get('decision'),
                     right=(fg.get('right') or {}).get('decision'),
                     centerline_recognition_available=fg.get('centerline_recognition_available')),
                 ensure_ascii=False,indent=2))

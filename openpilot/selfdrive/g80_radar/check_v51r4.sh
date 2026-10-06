#!/usr/bin/env bash
set -u
REPO="${OPENPILOT_REPO:-/data/openpilot}"
if [[ -d "$REPO/openpilot/selfdrive" ]]; then ROOT="$REPO/openpilot"; else ROOT="$REPO"; fi
PYDEPS="${G80_SIDE_VISION_PYDEPS:-/data/g80_pydeps}"

echo "=== G80 V51r4 CHECK ==="
echo "ROOT=$ROOT"
grep -E "BUILD_VERSION|BUILD_TAG|SIDE_VISION_API_VERSION" "$ROOT/selfdrive/g80_radar/build_info.py" || true

echo "--- filesystem ---"
df -h / /data /tmp 2>/dev/null || true
du -sh "$PYDEPS" /data/g80_pip_tmp 2>/dev/null || true

echo "--- cv2 backend ---"
PYTHONPATH="$PYDEPS${PYTHONPATH:+:$PYTHONPATH}" python3 - <<'PY'
try:
  import numpy as np
  print('numpy:', np.__version__, np.__file__)
  import cv2
  print('cv2: OK', cv2.__version__, cv2.__file__)
  print('DNN ONNX:', hasattr(cv2, 'dnn') and hasattr(cv2.dnn, 'readNetFromONNX'))
except Exception as e:
  print('cv2: MISSING/ERROR', repr(e))
PY

echo "--- process registration ---"
grep -n "g80sidevision" "$ROOT/system/manager/process_config.py" || true

echo "--- processes/ports ---"
ps -eo pid,args | grep -E '[g]80_radar.live_service|[g]80_radar.side_vision' || true
ss -lntup 2>/dev/null | grep -E ':28992|:28993|:28994' || true

echo "--- HTTP state ---"
python3 - <<'PY'
import json, urllib.request
for name,url in [('radar','http://127.0.0.1:28992/state'),('sidevision','http://127.0.0.1:28994/state')]:
  try:
    with urllib.request.urlopen(url,timeout=2) as r: d=json.loads(r.read().decode())
    if name=='sidevision':
      print('sidevision:', 'model',d.get('model_valid'),'backend',d.get('inference_backend'),'cv2',d.get('cv2_available'),'config',d.get('config_loaded'),'snapshot',d.get('snapshot_available'),'pending',d.get('snapshot_pending'),'camera',d.get('camera_connected'),'frames',d.get('frames_received'),'res',str(d.get('camera_width'))+'x'+str(d.get('camera_height')),'model_error',d.get('model_error'),'snapshot_error',d.get('snapshot_error'),'error',d.get('last_error'))
    else:
      print('radar:', 'version',d.get('version'),'sidevision_fresh',d.get('side_vision',{}).get('fresh'))
  except Exception as e:
    print(name, 'ERROR', repr(e))
PY

echo "--- snapshot/config ---"
ls -lh /data/radar/g80_side_vision_snapshot.png /data/radar/g80_side_vision_config.json 2>/dev/null || true


echo "--- V51r4 perf focus ---"
python3 - <<'PY2' || true
import json, urllib.request
try:
  s=json.load(urllib.request.urlopen('http://127.0.0.1:28992/state', timeout=1.5))
  p=s.get('performance_stats',{})
  m=s.get('ml_case_collector',{})
  v=s.get('side_vision',{})
  print('runtime_mismatch=',s.get('runtime_mismatch'))
  print('publish_ms=',p.get('publish_interval_ms'),'processing_ms=',p.get('processing_ms'),'ml_ms=',(p.get('stage_ms') or {}).get('ml_collector'),'ui_json_ms=',p.get('ui_json_ms'))
  print('collector actual_hz=',m.get('actual_capture_hz'),'missed=',m.get('capture_missed_slots'),'max_gap_ms=',m.get('max_frame_gap_ms'))
  print('sidevision usable=',v.get('usable'),'model=',v.get('model_valid'),'config=',v.get('config_loaded'),'camera=',v.get('camera_connected'),'snapshot_pending=',v.get('snapshot_pending'))
except Exception as e:
  print('perf focus error:',repr(e))
PY2

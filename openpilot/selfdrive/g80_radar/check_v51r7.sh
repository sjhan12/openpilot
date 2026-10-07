#!/usr/bin/env bash
set -u
REPO="${OPENPILOT_REPO:-/data/openpilot}"
if [[ -d "$REPO/openpilot/selfdrive" ]]; then ROOT="$REPO/openpilot"; else ROOT="$REPO"; fi
PYDEPS="${G80_SIDE_VISION_PYDEPS:-/data/g80_pydeps}"

echo "=== G80 V51r7 CHECK ==="
echo "ROOT=$ROOT"
grep -E "BUILD_VERSION|BUILD_TAG|SIDE_VISION_API_VERSION" "$ROOT/selfdrive/g80_radar/build_info.py" || true

echo "--- model ---"
if [[ -f "$ROOT/selfdrive/g80_radar/assets/v_asm_model.onnx" ]]; then
  ls -lh "$ROOT/selfdrive/g80_radar/assets/v_asm_model.onnx"
  echo "size=$(stat -c %s "$ROOT/selfdrive/g80_radar/assets/v_asm_model.onnx" 2>/dev/null || echo '?')"
  echo "blob=$(git hash-object "$ROOT/selfdrive/g80_radar/assets/v_asm_model.onnx" 2>/dev/null || echo '?')"
else
  echo "MODEL MISSING"
fi

echo "--- processes/ports ---"
ps -eo pid,args | grep -E '[g]80_radar.live_service|[g]80_radar.side_vision' || true
ss -lntup 2>/dev/null | grep -E ':28992|:28993|:28994' || true

echo "--- states ---"
python3 - <<'PY' || true
import json, urllib.request
for name,url in [('radar','http://127.0.0.1:28992/state'),('sidevision','http://127.0.0.1:28994/state')]:
  try:
    with urllib.request.urlopen(url,timeout=2) as r: d=json.loads(r.read().decode())
    if name=='radar':
      p=d.get('performance_stats',{}); m=d.get('ml_case_collector',{}); v=d.get('side_vision',{})
      print('radar tag=',(d.get('runtime_versions') or {}).get('tag'),'runtime_mismatch=',d.get('runtime_mismatch'))
      print('perf publish=',p.get('publish_interval_ms'),'core=',p.get('processing_ms'),'ml=',(p.get('stage_ms') or {}).get('ml_collector'),'late=',p.get('publish_late_ms'))
      print('collector actual_hz=',m.get('actual_capture_hz'),'missed=',m.get('capture_missed_slots'),'max_gap=',m.get('max_frame_gap_ms'))
      print('radar sidevision usable=',v.get('usable'),'fresh=',v.get('fresh'),'inference_fresh=',v.get('inference_fresh'))
    else:
      print('sidevision model=',d.get('model_valid'),'config=',d.get('config_loaded'),'camera=',d.get('camera_connected'),'onroad=',d.get('onroad'),'offroad_test=',d.get('offroad_test_enabled'),'ready=',d.get('inference_ready'))
      print('frames=',d.get('frames_received'),'inference_ms=',d.get('inference_ms'),'L=',d.get('left'),'R=',d.get('right'))
      print('model_error=',d.get('model_error'),'camera_error=',d.get('camera_error'),'last_error=',d.get('last_error'))
  except Exception as e:
    print(name,'ERROR',repr(e))
PY

echo "--- V51r7 NV12 fix source check ---"
grep -n "_visionipc_nv12_compact\|trailing guard bytes" "$ROOT/selfdrive/g80_radar/side_vision.py" | head -8 || true

echo "--- UI order source check ---"
grep -n 'data-mode="std"\|data-mode="l1l2"' "$ROOT/selfdrive/g80_radar/live_service.py" | head -4 || true

echo "--- config/snapshot ---"
ls -lh /data/radar/g80_side_vision_snapshot.png /data/radar/g80_side_vision_config.json 2>/dev/null || true

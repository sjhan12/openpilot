#!/usr/bin/env bash
set -u
REPO="${OPENPILOT_REPO:-/data/openpilot}"
if [[ -d "$REPO/openpilot/selfdrive" ]]; then ROOT="$REPO/openpilot"; else ROOT="$REPO"; fi

echo "=== G80 V52R2 CHECK ==="
echo "ROOT=$ROOT"
grep -E "BUILD_VERSION|BUILD_TAG|SIDE_VISION_API_VERSION|FRONT_CORNER_VISION_API_VERSION" "$ROOT/selfdrive/g80_radar/build_info.py" || true

echo "--- models ---"
for f in v_asm_model.onnx front_corner_v_asm_model.onnx; do
  P="$ROOT/selfdrive/g80_radar/assets/$f"
  if [[ -f "$P" ]]; then
    ls -lh "$P"
    echo "$f size=$(stat -c %s "$P" 2>/dev/null || echo '?') blob=$(git hash-object "$P" 2>/dev/null || echo '?')"
  else
    echo "$f MISSING"
  fi
done

echo "--- processes/ports ---"
ps -eo pid,args | grep -E '[g]80_radar.live_service|[g]80_radar.side_vision|[g]80_radar.front_corner_vision' || true
ss -lntup 2>/dev/null | grep -E ':28992|:28993|:28994|:28995|:28996' || true

echo "--- states ---"
python3 - <<'PY' || true
import json, urllib.request
for name,url in [
  ('radar','http://127.0.0.1:28992/state'),
  ('side','http://127.0.0.1:28994/state'),
  ('front','http://127.0.0.1:28995/state')]:
  try:
    with urllib.request.urlopen(url,timeout=2) as r:
      d=json.loads(r.read().decode())
    if name=='radar':
      p=d.get('performance_stats',{}); m=d.get('ml_case_collector',{}); sv=d.get('side_vision',{}); fc=d.get('front_corner_vision',{})
      print('radar tag=',(d.get('runtime_versions') or {}).get('tag'),'runtime_mismatch=',d.get('runtime_mismatch'))
      print('perf publish=',p.get('publish_interval_ms'),'core=',p.get('processing_ms'),'ml=',(p.get('stage_ms') or {}).get('ml_collector'),'late=',p.get('publish_late_ms'))
      print('collector mode=',m.get('capture_mode'),'actual_hz=',m.get('actual_capture_hz'),'active_hz=',m.get('actual_active_hz'),'active_missed=',m.get('capture_missed_slots'),'max_gap=',m.get('max_frame_gap_ms'))
      print('side usable=',sv.get('usable'),'L=',sv.get('left'),'R=',sv.get('right'))
      print('front usable=',fc.get('usable'),'FL=',fc.get('fl'),'FR=',fc.get('fr'))
    else:
      print(name,'model=',d.get('model_valid'),'config=',d.get('config_loaded'),'camera=',d.get('camera_connected'),'onroad=',d.get('onroad'),'offroad_test=',d.get('offroad_test_enabled'),'ready=',d.get('inference_ready'))
      print(name,'frames=',d.get('frames_received'),'inference_ms=',d.get('inference_ms'),'left=',d.get('left'),'right=',d.get('right'),'fl=',d.get('fl'),'fr=',d.get('fr'))
      print(name,'model_error=',d.get('model_error'),'camera_error=',d.get('camera_error'),'last_error=',d.get('last_error'))
  except Exception as e:
    print(name,'ERROR',repr(e))
PY

echo "--- process registration ---"
grep -n 'g80radard\|g80sidevision\|g80frontcornervision' "$ROOT/system/manager/process_config.py" || true

echo "--- UI checks ---"
grep -n 'FCAM SETUP\|FRONT CAM FL/FR\|leftFrontCam\|rightFrontCam' "$ROOT/selfdrive/g80_radar/live_service.py" | head -12 || true

echo "--- configs/snapshots ---"
ls -lh \
  /data/radar/g80_side_vision_snapshot.png \
  /data/radar/g80_side_vision_config.json \
  /data/radar/g80_front_corner_vision_snapshot.png \
  /data/radar/g80_front_corner_vision_config.json 2>/dev/null || true

#!/usr/bin/env bash
set -u
ROOT="${OPENPILOT_ROOT:-/data/openpilot}"

echo "=== G80 V51 CHECK ==="
grep -E "BUILD_VERSION|BUILD_TAG|SIDE_VISION_API_VERSION" "$ROOT/selfdrive/g80_radar/build_info.py" || true
grep -E "SCHEMA =|COLLECTOR_VERSION" "$ROOT/selfdrive/g80_radar/my_case_correcter.py" | head -n 3 || true
echo "--- processes ---"
ps -eo pid,args | grep -E '[g]80_radar.live_service|[g]80_radar.side_vision' || true
echo "--- ports ---"
ss -lntup 2>/dev/null | grep -E ':28992|:28993|:28994' || true
echo "--- radar health/state ---"
python3 - <<'PY'
import json, urllib.request
for name,url in [('radar','http://127.0.0.1:28992/state'),('sidevision','http://127.0.0.1:28994/state')]:
  try:
    with urllib.request.urlopen(url,timeout=2) as r:
      d=json.loads(r.read().decode())
    if name=='radar':
      p=d.get('performance_stats',{}); c=d.get('ml_case_collector',{}); v=d.get('side_vision',{})
      print('radar:', 'V',d.get('version'),'publish_ms',p.get('publish_interval_ms'),'core_ms',p.get('processing_ms'),'can_ms',p.get('can_drain_ms'),'late_ms',p.get('publish_late_ms'))
      print('collector:', c.get('collector_version'),'actual_hz',c.get('actual_capture_hz'),'missed',c.get('capture_missed_slots'),'auto',c.get('auto_lane_change',{}).get('last_result'))
      print('vision_in_radar:', 'fresh',v.get('fresh'),'age_ms',v.get('age_ms'),'inf_fresh',v.get('inference_fresh'),'inf_age_ms',v.get('inference_age_ms'),'usable',v.get('usable'),'L',v.get('left',{}).get('score'),'R',v.get('right',{}).get('score'))
    else:
      print('sidevision:', 'model',d.get('model_valid'),'config',d.get('config_loaded'),'camera',d.get('camera_connected'),'onroad',d.get('onroad'),'inf_ms',d.get('inference_ms'),'throttle',d.get('throttle_factor'),'error',d.get('last_error'))
  except Exception as e:
    print(name, 'ERROR', repr(e))
PY

echo "--- model ---"
MODEL="$ROOT/selfdrive/g80_radar/assets/v_asm_model.onnx"
if [[ -f "$MODEL" ]]; then
  echo "model exists: $MODEL"
  if command -v git >/dev/null 2>&1; then
    BLOB="$(git hash-object "$MODEL" 2>/dev/null || true)"
    echo "git blob: $BLOB"
    [[ "$BLOB" == "6a1ea709681ce256927e0cf36e53defad5ce94d8" ]] && echo "model blob: OK" || echo "WARN: unexpected model blob"
  fi
else
  echo "MODEL MISSING: run $ROOT/selfdrive/g80_radar/fetch_vasm_model.sh"
fi

echo "--- logging policy ---"
grep -R "shadow_logger.maybe_write" "$ROOT/selfdrive/g80_radar/live_service.py" >/dev/null 2>&1 && echo "WARN: shadow write call exists" || echo "shadow write call: REMOVED"
[[ -f "$ROOT/selfdrive/g80_radar/raw_golden_logger.py" ]] && echo "WARN: raw_golden_logger.py exists" || echo "raw golden source: REMOVED"

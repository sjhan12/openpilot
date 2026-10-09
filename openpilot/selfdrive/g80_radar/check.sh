#!/usr/bin/env bash
# G80 C4-ONLY diagnostics. Build version is read from build_info.py.
set -u
REPO="${OPENPILOT_REPO:-/data/openpilot}"
if [[ -d "$REPO/openpilot/selfdrive" ]]; then ROOT="$REPO/openpilot"; else ROOT="$REPO"; fi
RADAR="$ROOT/selfdrive/g80_radar"
echo '=== G80 C4-ONLY / NO UVC / NO YOLO ==='
[[ -f "$RADAR/build_info.py" ]] && grep -E '^BUILD_TAG|^BUILD_VERSION' "$RADAR/build_info.py" || true
missing=0
for file in live_service.py lane_change_shadow.py vasm_warning.py cut_event_shadow.py shadow_leads.py side_vision.py front_corner_vision.py install_sidevision_opencv_data.sh assets/v_asm_model.onnx assets/front_corner_v_asm_model.onnx; do
  if [[ -s "$RADAR/$file" ]]; then printf 'OK      %s\n' "$file"; else printf 'MISSING %s\n' "$file"; missing=1; fi
done
for old in lane_change_shadow_v52r3.py vasm_warning_v52r4.py cut_event_shadow_v52r5.py check_v52r7.sh; do
  [[ -e "$RADAR/$old" ]] && echo "WARNING: obsolete versioned file remains: $old"
done
if [[ -f "$ROOT/system/manager/process_config.py" ]]; then
  echo '--- registered services ---'
  grep -nE '^[[:space:]]*PythonProcess\("g80(radard|sidevision|frontcornervision|widevision)"' "$ROOT/system/manager/process_config.py" || true
  if grep -qE '^[[:space:]]*PythonProcess\("g80widevision"' "$ROOT/system/manager/process_config.py"; then echo 'WARNING: external UVC registered'; fi
fi
expected='5d20cdbb457ba18db51a537ee2e305bbe442264b1613956068d473e35d15900d'
for model in v_asm_model.onnx front_corner_v_asm_model.onnx; do
  path="$RADAR/assets/$model"
  if [[ -s "$path" ]]; then
    actual="$(sha256sum "$path" | awk '{print $1}')"
    if [[ "$actual" == "$expected" ]]; then echo "MODEL OK SHA-256 $model"; else echo "MODEL CUSTOM/DIFFERENT SHA-256 $model"; fi
  fi
done
echo '--- web endpoints ---'
for port in 28992 28994 28995; do
  if curl --max-time 2 --fail --silent "http://127.0.0.1:$port/state" -o /dev/null; then
    echo "[$port] /state reachable"
  else echo "[$port] /state unavailable (may be OFFROAD or process stopped)"; fi
done
echo 'NOTE: diagnostics only; this script does not enable any driving control.'
exit "$missing"

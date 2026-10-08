#!/usr/bin/env bash
set -u
REPO="${OPENPILOT_REPO:-/data/openpilot}"
if [[ -d "$REPO/openpilot/selfdrive" ]]; then ROOT="$REPO/openpilot"; else ROOT="$REPO"; fi
printf '%s\n' '=== G80 V52R7 C4-ONLY / NO UVC / NO YOLO ==='
for path in \
  "$ROOT/selfdrive/g80_radar/live_service.py" \
  "$ROOT/selfdrive/g80_radar/lane_change_shadow_v52r3.py" \
  "$ROOT/selfdrive/g80_radar/vasm_warning_v52r4.py" \
  "$ROOT/selfdrive/g80_radar/side_vision.py" \
  "$ROOT/selfdrive/g80_radar/front_corner_vision.py" \
  "$ROOT/selfdrive/g80_radar/assets/v_asm_model.onnx" \
  "$ROOT/selfdrive/g80_radar/assets/front_corner_v_asm_model.onnx"; do
  [[ -f "$path" ]] && printf 'OK      %s\n' "$path" || printf 'MISSING %s\n' "$path"
done
printf '%s\n' '--- running process registrations ---'
grep -n -E '"g80(radard|sidevision|frontcornervision|widevision)"' "$ROOT/system/manager/process_config.py" || true
if grep -q '"g80widevision"' "$ROOT/system/manager/process_config.py"; then
  echo 'WARNING: obsolete external UVC process is still registered.'
fi
printf '%s\n' '--- built-in C4 web status ---'
for port in 28992 28994 28995; do
  echo "[$port]"
  if [[ "$port" == 28992 ]]; then
    curl -m 2 -fsS "http://127.0.0.1:$port/state" 2>/dev/null | python3 -c 'import sys,json;s=json.load(sys.stdin);print("tag:",s.get("runtime_versions",{}).get("tag"));print("C4 cabin:",s.get("side_vision",{}).get("usable")," C4 road wide:",s.get("front_corner_vision",{}).get("usable"));print("VASM warn L/R:",s.get("vasm_warning",{}).get("left",{}).get("warning_label"),s.get("vasm_warning",{}).get("right",{}).get("warning_label"));print("FG15 L/R:",s.get("future_gap",{}).get("left",{}).get("decision",{}).get("label"),s.get("future_gap",{}).get("right",{}).get("decision",{}).get("label"));print("L1/L2:",s.get("shadow_lead_interface",{}).get("leadOne",{}).get("candidate_valid"),s.get("shadow_lead_interface",{}).get("leadTwo",{}).get("candidate_valid"));print("core/publish ms:",s.get("performance_stats",{}).get("processing_ms"),s.get("performance_stats",{}).get("publish_interval_ms"))' 2>/dev/null || echo 'WAIT (service not available)'
  else
    curl -m 2 -fsS "http://127.0.0.1:$port/state" 2>/dev/null | python3 -c 'import sys,json;s=json.load(sys.stdin);print("model",s.get("model_valid"),"config",s.get("config_loaded"),"camera",s.get("camera_connected"),"onroad",s.get("onroad"),"usable",s.get("usable"),"fresh",s.get("fresh"))' 2>/dev/null || echo 'WAIT (service not available)'
  fi
done
printf 'backup: '; cat /data/radar/v52r7_last_backup 2>/dev/null || printf '%s\n' 'UNKNOWN'
echo 'NOTE: NO external UVC; no modeld, CAN TX, controlsd or FG15 patch.'
echo 'To disable V-ASM HUD warning: touch /data/radar/DISABLE_G80_VASM_WARNING'

# V52R7: model second lead may exist without a second radar observation.
curl -m 2 -fsS http://127.0.0.1:28992/state 2>/dev/null | python3 -c 'import json,sys;s=json.load(sys.stdin).get("shadow_leads",{});print("L2 reason:",s.get("stats",{}).get("lead2_selection_reason"));print("CUT-IN:",s.get("cutInLead",{}).get("status"),"STOP HAZARD:",s.get("stopHazard",{}).get("status"));print("C4 model2:",s.get("visionLeadTwo"));print("measured next:",s.get("nextForward",{}).get("status"))' 2>/dev/null || true

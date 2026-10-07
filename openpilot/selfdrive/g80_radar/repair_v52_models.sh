#!/usr/bin/env bash
set -euo pipefail
if [[ -d /data/openpilot/openpilot/selfdrive ]]; then ROOT=/data/openpilot/openpilot; else ROOT=/data/openpilot; fi
D="$ROOT/selfdrive/g80_radar/assets"
mkdir -p "$D"
echo "[V52R2] repairing side/front camera models..."
OPENPILOT_ROOT="$ROOT" bash "$ROOT/selfdrive/g80_radar/fetch_vasm_model.sh"
if [[ ! -f "$D/front_corner_v_asm_model.onnx" ]]; then
  cp -a "$D/v_asm_model.onnx" "$D/front_corner_v_asm_model.onnx"
fi
for f in v_asm_model.onnx front_corner_v_asm_model.onnx; do
  echo "--- $f ---"
  ls -lh "$D/$f"
  echo "size=$(stat -c %s "$D/$f") blob=$(git hash-object "$D/$f" 2>/dev/null || echo '?')"
done
echo "[V52R2] done. Daemons reload models automatically within about 5 seconds."

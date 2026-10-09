#!/usr/bin/env bash
# G80 V52R7 - C4 cabin / front WIDE V-ASM model assets for OpenCV DNN.
# Stable script filename for direct GitHub updates. No pip/system modification.
# Usage: bash install_sidevision_opencv_data.sh [--check|--repair|--force]
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ASSETS="$HERE/assets"
SHA256='5d20cdbb457ba18db51a537ee2e305bbe442264b1613956068d473e35d15900d'
SOURCE='https://raw.githubusercontent.com/firestar5683/StarPilot/0122e4069b627948b219e419d2e84b5f22773c43/starpilot/assets/vision_models/v_asm_model.onnx'
MODE="${1:---repair}"
case "$MODE" in --check|--repair|--force) ;; *) echo 'Usage: bash install_sidevision_opencv_data.sh [--check|--repair|--force]' >&2; exit 2;; esac

MODEL_NAMES=(v_asm_model.onnx front_corner_v_asm_model.onnx)
valid() {
  [[ -s "$1" ]] && [[ "$(sha256sum "$1" | awk '{print $1}')" == "$SHA256" ]]
}

# Do not modify the installed model directory while driving.
if [[ "$MODE" != '--check' ]]; then
  PARAMS="${G80_PARAMS_DIR:-/data/params/d}"
  if [[ "$(cat "$PARAMS/IsOnroad" 2>/dev/null || true)" == '1' ]] || \
     [[ "$(cat "$PARAMS/IsOffroad" 2>/dev/null || true)" == '0' ]]; then
    echo '[G80 V-ASM] OFFROAD required for model installation.' >&2; exit 1
  fi
  mkdir -p "$ASSETS"
fi

# A valid bundled/previous asset is preferable to network download.
get_valid_source() {
  for name in "${MODEL_NAMES[@]}"; do
    if valid "$ASSETS/$name"; then printf '%s\n' "$ASSETS/$name"; return 0; fi
  done
  return 1
}

for name in "${MODEL_NAMES[@]}"; do
  dest="$ASSETS/$name"
  if valid "$dest" && [[ "$MODE" != '--force' ]]; then
    echo "[G80 V-ASM] OK $name (SHA-256)"
    continue
  fi
  if [[ "$MODE" == '--check' ]]; then
    echo "[G80 V-ASM] FAIL missing/nonstandard model: $dest" >&2
    exit 1
  fi
  if [[ -s "$dest" && "$MODE" != '--force' ]]; then
    echo "[G80 V-ASM] Nonstandard model preserved: $dest; use --force only if appropriate" >&2
    exit 1
  fi
  tmp="$(mktemp "$ASSETS/.vasm_model_XXXXXX")"
  if source_file="$(get_valid_source)"; then
    if ! cp "$source_file" "$tmp"; then rm -f "$tmp"; exit 1; fi
  else
    if ! curl --fail --location --retry 2 --connect-timeout 8 --max-time 120 --silent --show-error "$SOURCE" -o "$tmp"; then
      rm -f "$tmp"; echo '[G80 V-ASM] Download failed; previous model preserved.' >&2; exit 1
    fi
  fi
  if ! valid "$tmp"; then
    rm -f "$tmp"; echo '[G80 V-ASM] SHA-256 mismatch; previous model preserved.' >&2; exit 1
  fi
  if [[ -s "$dest" ]]; then cp -p "$dest" "$dest.bak.$(date +%Y%m%d_%H%M%S)"; fi
  chmod 644 "$tmp"
  mv -f "$tmp" "$dest"
  echo "[G80 V-ASM] Installed verified model: $name"
done

# Confirm the currently selected Python environment has OpenCV DNN and the
# model parser can load this exact ONNX. This does not run inference.
python3 - "$ASSETS" <<'PY'
import sys
from pathlib import Path
try:
    import cv2
except Exception as exc:
    raise SystemExit(f'[G80 V-ASM] OpenCV import failed: {exc}; Python environment must be repaired separately')
if not hasattr(cv2, 'dnn'):
    raise SystemExit('[G80 V-ASM] OpenCV DNN module missing; no automatic system modifications performed')
folder = Path(sys.argv[1])
for file in ('v_asm_model.onnx', 'front_corner_v_asm_model.onnx'):
    try:
        net = cv2.dnn.readNetFromONNX(str(folder / file))
        if net.empty():
            raise RuntimeError('DNN net empty')
    except Exception as exc:
        raise SystemExit(f'[G80 V-ASM] OpenCV ONNX load failed for {file}: {exc}')
    print(f'[G80 V-ASM] OpenCV DNN model load OK: {file}')
print(f'[G80 V-ASM] READY OpenCV {cv2.__version__}; inference/ROI alignment must still be tested on Comma 4')
PY

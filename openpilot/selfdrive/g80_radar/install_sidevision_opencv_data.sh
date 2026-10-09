#!/usr/bin/env bash
# G80 SideVision / FrontCorner OpenCV installer (V52R7 distribution).
# Restores the V52R2 /data-only installation scheme; no system pip/NumPy changes.
# Usage: bash install_sidevision_opencv_data.sh [--install|--check|--repair|--force]
set -euo pipefail

TARGET="${G80_SIDE_VISION_PYDEPS:-/data/g80_pydeps}"
TMPROOT="${G80_SIDE_VISION_PIP_TMP:-/data/g80_pip_tmp}"
OPENCV_VER="${G80_SIDE_VISION_OPENCV_VERSION:-4.10.0.84}"
PYTHON_BIN="${G80_SIDE_VISION_PYTHON:-python3}"
MODE="${1:---install}"
ASSETS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/assets"
case "$MODE" in
  --check|--install|--repair|--force) ;;
  *) echo 'Usage: bash install_sidevision_opencv_data.sh [--install|--check|--repair|--force]' >&2; exit 2 ;;
esac

# Limit writes to persistent user-data paths; keep the small AGNOS system partition untouched.
for dir in "$TARGET" "$TMPROOT"; do
  case "$dir" in
    /data/*) ;;
    *) echo "[G80 OpenCV] Refusing a non-/data install/temp directory: $dir" >&2; exit 2 ;;
  esac
  [[ "$dir" != *'/../'* && "$dir" != *'/./'* && "$dir" != */.. && "$dir" != */. ]] || {
    echo "[G80 OpenCV] Unsafe path: $dir" >&2; exit 2;
  }
done
[[ "$TARGET" != "$TMPROOT" ]] || { echo '[G80 OpenCV] TARGET and TMPROOT must differ' >&2; exit 2; }

# Loads only the copied cv2. Uses the existing Openpilot NumPy and checks its ABI.
check_opencv() {
  PYTHONPATH="$TARGET${PYTHONPATH:+:$PYTHONPATH}" "$PYTHON_BIN" - "$TARGET" "$ASSETS" <<'PY'
import os, sys
from pathlib import Path
root = Path(sys.argv[1]).resolve()
assets = Path(sys.argv[2])
import numpy as np
import cv2
module = Path(cv2.__file__).resolve()
if not module.is_relative_to(root):
    raise SystemExit(f'cv2 loaded outside private target: {module}')
if not hasattr(cv2, 'dnn') or not hasattr(cv2.dnn, 'readNetFromONNX'):
    raise SystemExit('OpenCV DNN/ONNX API missing')
print(f'[G80 OpenCV] OK version={cv2.__version__} numpy={np.__version__}')
print(f'[G80 OpenCV] cv2={module}')
for name in ('v_asm_model.onnx', 'front_corner_v_asm_model.onnx'):
    model = assets / name
    if model.is_file():
        net = cv2.dnn.readNetFromONNX(str(model))
        if net.empty():
            raise SystemExit(f'Empty ONNX network: {name}')
        print(f'[G80 OpenCV] DNN model OK: {name}')
    else:
        print(f'[G80 OpenCV] Model absent (install separately): {name}')
PY
}

printf '%s\n' '=== G80 V52R7 OpenCV private /data install ==='
printf 'TARGET=%s\nTMPDIR=%s\nVERSION=%s\n' "$TARGET" "$TMPROOT" "$OPENCV_VER"
df -h / /data 2>/dev/null || true

if [[ "$MODE" == '--check' ]]; then
  check_opencv
  exit 0
fi

PARAMS="${G80_PARAMS_DIR:-/data/params/d}"
if [[ "$(cat "$PARAMS/IsOnroad" 2>/dev/null || true)" == '1' ]] || \
   [[ "$(cat "$PARAMS/IsOffroad" 2>/dev/null || true)" == '0' ]]; then
  echo '[G80 OpenCV] Stop: install only while parked and OFFROAD.' >&2; exit 1
fi

# Do not download/reinstall OpenCV when an existing private copy works.
if [[ "$MODE" != '--force' ]] && check_opencv >/dev/null 2>&1; then
  echo '[G80 OpenCV] Existing /data OpenCV-DNN works; no installation needed.'
  check_opencv
  exit 0
fi

"$PYTHON_BIN" - <<'PY'
import numpy as np
print(f'[G80 OpenCV] Using existing system NumPy {np.__version__}: {np.__file__}')
PY
"$PYTHON_BIN" -m pip --version >/dev/null || {
  echo '[G80 OpenCV] pip is missing from this Python; no system modification performed.' >&2; exit 1;
}

FREE_KB="$(df -Pk /data | awk 'NR==2 {print $4}')"
if [[ -z "$FREE_KB" ]] || (( FREE_KB < 307200 )); then
  echo '[G80 OpenCV] /data needs at least 300 MB free before installing.' >&2; exit 2
fi

# All unpack/cache activity stays on /data, including pip's temporary files.
mkdir -p "$TARGET" "$TMPROOT"
WORK="$(mktemp -d "$TMPROOT/opencv.XXXXXXXX")"
STAGE="$WORK/stage"
BACKUP="$WORK/backup"
mkdir -p "$STAGE" "$BACKUP"
export TMPDIR="$WORK" TMP="$WORK" TEMP="$WORK"
export PIP_NO_CACHE_DIR=1 PIP_DISABLE_PIP_VERSION_CHECK=1
COMMITTED=0
MOVED_OLD=0
cleanup() {
  status=$?
  if [[ "$MOVED_OLD" == 1 && "$COMMITTED" == 0 ]]; then
    echo '[G80 OpenCV] Installation failed; restoring previous private cv2.' >&2
    shopt -s nullglob
    for f in "$STAGE"/cv2 "$STAGE"/opencv_python_headless* "$TARGET"/cv2 "$TARGET"/opencv_python_headless*; do
      [[ -e "$f" ]] && rm -rf -- "$f"
    done
    for f in "$BACKUP"/*; do
      [[ -e "$f" ]] && mv -- "$f" "$TARGET/"
    done
    shopt -u nullglob
  fi
  rm -rf -- "$WORK"
  return "$status"
}
trap cleanup EXIT

"$PYTHON_BIN" -m pip install \
  --no-cache-dir --no-deps --no-compile --only-binary=:all: \
  --target "$STAGE" "opencv-python-headless==${OPENCV_VER}"

# Validate the downloaded wheel in isolation BEFORE replacing any working copy.
PYTHONPATH="$STAGE${PYTHONPATH:+:$PYTHONPATH}" "$PYTHON_BIN" - "$STAGE" "$ASSETS" <<'PY'
import sys
from pathlib import Path
import numpy as np
import cv2
stage = Path(sys.argv[1]).resolve()
if not Path(cv2.__file__).resolve().is_relative_to(stage):
    raise SystemExit('Wheel validation used a different cv2')
if not hasattr(cv2.dnn, 'readNetFromONNX'):
    raise SystemExit('OpenCV DNN/ONNX API missing')
for name in ('v_asm_model.onnx', 'front_corner_v_asm_model.onnx'):
    model = Path(sys.argv[2]) / name
    if model.is_file():
        net = cv2.dnn.readNetFromONNX(str(model))
        if net.empty():
            raise SystemExit(f'ONNX load failed: {name}')
print(f'[G80 OpenCV] Wheel validated: OpenCV {cv2.__version__}, NumPy {np.__version__}')
PY

# Files are moved within /data (atomic per entry); restore previous copy if a step fails.
shopt -s nullglob
for f in "$TARGET"/cv2 "$TARGET"/opencv_python_headless*; do
  [[ -e "$f" ]] && mv -- "$f" "$BACKUP/"
done
MOVED_OLD=1
for f in "$STAGE"/*; do
  mv -- "$f" "$TARGET/"
done
shopt -u nullglob
check_opencv
COMMITTED=1
printf '%s\n' '[G80 OpenCV] Installed privately under /data. Reboot while OFFROAD to restart both C4 vision daemons.'

#!/usr/bin/env bash
set -euo pipefail
TARGET="${G80_SIDE_VISION_PYDEPS:-/data/g80_pydeps}"
mkdir -p "$TARGET"
echo "Installing opencv-python-headless under $TARGET (not the AGNOS system partition)..."
python3 -m pip install --no-cache-dir --upgrade --target "$TARGET" opencv-python-headless
PYTHONPATH="$TARGET${PYTHONPATH:+:$PYTHONPATH}" python3 - <<'PY'
import cv2
print('cv2 OK:', cv2.__version__, cv2.__file__)
PY
echo "Done. Reboot or restart g80sidevision. side_vision.py auto-adds $TARGET to sys.path."

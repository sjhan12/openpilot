#!/usr/bin/env bash
set -euo pipefail

TARGET="${G80_SIDE_VISION_PYDEPS:-/data/g80_pydeps}"
TMPROOT="${G80_SIDE_VISION_PIP_TMP:-/data/g80_pip_tmp}"
OPENCV_VER="${G80_SIDE_VISION_OPENCV_VERSION:-4.10.0.84}"

# pip --target still unpacks wheels in TMPDIR.  On AGNOS /tmp lives on the
# small system partition, so force all temporary work onto /data as well.
rm -rf "$TMPROOT"
mkdir -p "$TARGET" "$TMPROOT"

# Remove only leftovers from a previous failed G80 side-vision install.
rm -rf "$TARGET/cv2" "$TARGET/opencv_python_headless-"*.dist-info 2>/dev/null || true

printf '%s\n' "=== G80 SideVision OpenCV install ==="
printf 'TARGET=%s\nTMPDIR=%s\n' "$TARGET" "$TMPROOT"
df -h / /data "$TMPROOT" 2>/dev/null || true

# openpilot already requires NumPy.  Do not pull a second NumPy into /data;
# besides wasting space, a second ABI can break the running openpilot process.
python3 - <<'PY'
import numpy as np
print('system numpy OK:', np.__version__, np.__file__)
PY

# Require a modest safety margin for wheel extraction.  The wheel itself is
# ~tens of MB but temporary extraction needs considerably more room.
FREE_KB="$(df -Pk /data | awk 'NR==2 {print $4}')"
if [[ -n "$FREE_KB" && "$FREE_KB" -lt 307200 ]]; then
  echo "ERROR: /data has less than 300 MB free. Free space first, then rerun."
  exit 2
fi

export TMPDIR="$TMPROOT"
export TMP="$TMPROOT"
export TEMP="$TMPROOT"
export PIP_NO_CACHE_DIR=1
export PIP_DISABLE_PIP_VERSION_CHECK=1

# Pin to a mature OpenCV-DNN build and reuse openpilot's existing NumPy.
python3 -m pip install \
  --no-cache-dir --no-deps --no-compile \
  --target "$TARGET" \
  "opencv-python-headless==${OPENCV_VER}"

PYTHONPATH="$TARGET${PYTHONPATH:+:$PYTHONPATH}" python3 - <<'PY'
import numpy as np
import cv2
print('numpy:', np.__version__, np.__file__)
print('cv2 OK:', cv2.__version__, cv2.__file__)
print('DNN ONNX:', hasattr(cv2, 'dnn') and hasattr(cv2.dnn, 'readNetFromONNX'))
PY

rm -rf "$TMPROOT"
echo "Done. Restart g80sidevision or reboot."

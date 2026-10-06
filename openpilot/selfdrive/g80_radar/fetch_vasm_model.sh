#!/usr/bin/env bash
set -euo pipefail

ROOT="${OPENPILOT_ROOT:-/data/openpilot}"
DEST="${G80_SIDE_VISION_MODEL:-$ROOT/selfdrive/g80_radar/assets/v_asm_model.onnx}"
TMP="${DEST}.tmp.$$"
COMMIT="0122e4069b627948b219e419d2e84b5f22773c43"
EXPECTED_BLOB="6a1ea709681ce256927e0cf36e53defad5ce94d8"
URL="https://raw.githubusercontent.com/firestar5683/StarPilot/${COMMIT}/starpilot/assets/vision_models/v_asm_model.onnx"

mkdir -p "$(dirname "$DEST")"

if [[ -f "$DEST" ]]; then
  if command -v git >/dev/null 2>&1 && [[ "$(git hash-object "$DEST" 2>/dev/null || true)" == "$EXPECTED_BLOB" ]]; then
    echo "[V51] V-ASM model already verified: $DEST"
    exit 0
  fi
  echo "[V51] Existing model is not the expected StarPilot blob; replacing it."
fi

rm -f "$TMP"
if command -v curl >/dev/null 2>&1; then
  curl -L --fail --retry 3 --connect-timeout 10 --max-time 180 -o "$TMP" "$URL"
elif command -v wget >/dev/null 2>&1; then
  wget -O "$TMP" "$URL"
else
  echo "[V51] ERROR: curl/wget not available."
  echo "Download the StarPilot model manually to: $DEST"
  exit 2
fi

if command -v git >/dev/null 2>&1; then
  GOT="$(git hash-object "$TMP")"
  if [[ "$GOT" != "$EXPECTED_BLOB" ]]; then
    echo "[V51] ERROR: model Git blob mismatch: got=$GOT expected=$EXPECTED_BLOB"
    rm -f "$TMP"
    exit 3
  fi
fi

mv "$TMP" "$DEST"
sync
echo "[V51] V-ASM model installed: $DEST"
echo "[V51] source commit: $COMMIT"

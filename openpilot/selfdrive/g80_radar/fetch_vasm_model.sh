#!/usr/bin/env bash
set -euo pipefail

if [[ -n "${OPENPILOT_ROOT:-}" ]]; then
  ROOT="$OPENPILOT_ROOT"
elif [[ -d /data/openpilot/openpilot/selfdrive ]]; then
  ROOT=/data/openpilot/openpilot
else
  ROOT=/data/openpilot
fi
DEST="${G80_SIDE_VISION_MODEL:-$ROOT/selfdrive/g80_radar/assets/v_asm_model.onnx}"
TMP="${DEST}.tmp.$$"
COMMIT="0122e4069b627948b219e419d2e84b5f22773c43"
EXPECTED_BLOB="6a1ea709681ce256927e0cf36e53defad5ce94d8"
EXPECTED_SIZE="6166229"
URL1="https://github.com/firestar5683/StarPilot/raw/${COMMIT}/starpilot%2Fassets%2Fvision_models%2Fv_asm_model.onnx"
URL2="https://raw.githubusercontent.com/firestar5683/StarPilot/${COMMIT}/starpilot/assets/vision_models/v_asm_model.onnx"

mkdir -p "$(dirname "$DEST")"

verify_model() {
  local f="$1"
  [[ -f "$f" ]] || return 1
  local sz
  sz="$(stat -c %s "$f" 2>/dev/null || wc -c < "$f")"
  if [[ "$sz" != "$EXPECTED_SIZE" ]]; then
    echo "[V51r5] model size mismatch: got=$sz expected=$EXPECTED_SIZE" >&2
    return 1
  fi
  if command -v git >/dev/null 2>&1; then
    local got
    got="$(git hash-object "$f" 2>/dev/null || true)"
    if [[ "$got" != "$EXPECTED_BLOB" ]]; then
      echo "[V51r5] model Git blob mismatch: got=$got expected=$EXPECTED_BLOB" >&2
      return 1
    fi
  fi
  return 0
}

if verify_model "$DEST"; then
  echo "[V51r5] V-ASM model already verified: $DEST"
  exit 0
fi

rm -f "$TMP"
fetch_one() {
  local url="$1"
  echo "[V51r5] downloading model from: $url"
  if command -v curl >/dev/null 2>&1; then
    curl -L --fail --retry 3 --retry-delay 2 --connect-timeout 15 --max-time 240 -o "$TMP" "$url"
  elif command -v wget >/dev/null 2>&1; then
    wget --timeout=30 --tries=3 -O "$TMP" "$url"
  else
    return 127
  fi
}

ok=0
for u in "$URL1" "$URL2"; do
  rm -f "$TMP"
  if fetch_one "$u" && verify_model "$TMP"; then
    ok=1
    break
  fi
  echo "[V51r5] source failed or verification failed; trying fallback..." >&2
done

if [[ "$ok" != "1" ]]; then
  rm -f "$TMP"
  echo "[V51r5] ERROR: could not install StarPilot V-ASM model." >&2
  echo "[V51r5] Side Vision will remain MODEL WAIT; radar/FG15 are unaffected." >&2
  exit 3
fi

mv "$TMP" "$DEST"
sync
echo "[V51r5] V-ASM model installed and verified: $DEST"
echo "[V51r5] size: $EXPECTED_SIZE bytes"
echo "[V51r5] source commit: $COMMIT"
echo "[V51r5] side_vision daemon retries model load every ~5 s; reboot is not required."

#!/usr/bin/env bash
# G80 C4 V-ASM ONNX assets — pinned SHA-256 verified, atomic replacement.
# This package contains verified assets; network is needed only when assets are unavailable
# or when explicitly requested by --download.
set -euo pipefail
PKG="$(cd "$(dirname "$0")" && pwd)"
REPO="${OPENPILOT_REPO:-/data/openpilot}"
if [[ -d "$REPO/openpilot/selfdrive" ]]; then ROOT="$REPO/openpilot"
elif [[ -d "$REPO/selfdrive" ]]; then ROOT="$REPO"
else echo '[G80] Cannot find OpenPilot repository' >&2; exit 1; fi
DEST="$ROOT/selfdrive/g80_radar/assets"
SRC="$PKG/selfdrive/g80_radar/assets/v_asm_model.onnx"
SHA='5d20cdbb457ba18db51a537ee2e305bbe442264b1613956068d473e35d15900d'
URL='https://raw.githubusercontent.com/firestar5683/StarPilot/0122e4069b627948b219e419d2e84b5f22773c43/starpilot/assets/vision_models/v_asm_model.onnx'
MODE="${1:---repair}"
case "$MODE" in --repair|--check|--download|--force) ;; *) echo "Usage: bash download_vasm_model.sh [--check|--repair|--download|--force]" >&2; exit 2;; esac
if [[ "$MODE" != '--check' ]]; then mkdir -p "$DEST"; fi
ok_hash() { [[ -s "$1" ]] && [[ "$(sha256sum "$1" | awk '{print $1}')" == "$SHA" ]]; }
install_asset() {
  local path="$1" temp
  if [[ -s "$path" && "$MODE" != '--force' && "$MODE" != '--download' ]]; then
    if ok_hash "$path"; then echo "[G80] OK existing: $path"; return 0; fi
    echo "[G80] Existing custom or invalid model preserved: $path (use --force to replace)" >&2
    return 1
  fi
  if [[ "$MODE" == '--check' ]]; then
    if ok_hash "$path"; then echo "[G80] PASS SHA-256: $path"; return 0; fi
    echo "[G80] FAIL missing or different SHA-256: $path" >&2; return 1
  fi
  temp="$(mktemp "$DEST/.g80_vasm_XXXXXX")"
  if [[ "$MODE" == '--download' ]]; then
    if ! curl --fail --location --retry 2 --connect-timeout 8 --max-time 120 --silent --show-error "$URL" -o "$temp"; then rm -f "$temp"; return 1; fi
  elif ok_hash "$SRC"; then
    cp "$SRC" "$temp"
  else
    if ! curl --fail --location --retry 2 --connect-timeout 8 --max-time 120 --silent --show-error "$URL" -o "$temp"; then rm -f "$temp"; return 1; fi
  fi
  if ! ok_hash "$temp"; then
    echo '[G80] Downloaded/bundled model SHA-256 mismatch — previous file unchanged.' >&2
    rm -f "$temp";return 1
  fi
  chmod 644 "$temp"
  mv -f "$temp" "$path"
  echo "[G80] Installed verified model: $path"
}
install_asset "$DEST/v_asm_model.onnx"
install_asset "$DEST/front_corner_v_asm_model.onnx"
echo '[G80] V-ASM weights ready. Runtime availability depends on OpenCV and camera ROI configuration.'

#!/usr/bin/env bash
set -u
ROOT="${OPENPILOT_ROOT:-/data/openpilot/openpilot}"
[[ -d "$ROOT/selfdrive" ]] || ROOT=/data/openpilot
MODEL="$ROOT/selfdrive/g80_radar/assets/v_asm_model.onnx"
echo "=== G80 V-ASM MODEL CHECK ==="
echo "root=$ROOT"
if [[ -f "$MODEL" ]]; then
  ls -lh "$MODEL"
  echo "bytes=$(stat -c %s "$MODEL" 2>/dev/null || wc -c < "$MODEL")"
  command -v git >/dev/null 2>&1 && echo "git_blob=$(git hash-object "$MODEL" 2>/dev/null || true)"
else
  echo "MODEL MISSING: $MODEL"
fi
echo "--- sidevision state ---"
curl -s http://127.0.0.1:28994/state || true
echo

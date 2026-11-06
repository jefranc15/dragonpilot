#!/bin/bash
# Offline dual-route R2 validation. Never launches carcontroller or Panda.
# Run this ONLY from a detached/isolated R2 worktree while the car is parked.
set -Eeuo pipefail
trap 'echo "R2 OFFLINE REPLAY ERROR: line $LINENO (exit $?)" >&2' ERR

WORK=/data/v50_r2_zip_replay
NOV05="${1:-/data/realdata1.zip}"
NOV06="${2:-/data/nov06_curve_rev.zip}"
OUT="${3:-/data/v50_r2_dual_results}"
PY=/data/data/com.termux/files/usr/bin/python3

if [ ! -f "$NOV05" ]; then echo "Missing November 5 ZIP: $NOV05"; exit 1; fi
if [ ! -f "$NOV06" ]; then
  echo "Missing November 6 ZIP: $NOV06"
  echo "Copy the 3-segment 'v5.0 curve brakle rev logs.zip' to that path first."
  exit 1
fi
if [ ! -d "$WORK/.git" ] && [ ! -f "$WORK/.git" ]; then
  echo "Missing isolated worktree: $WORK"; exit 1
fi
test -f "$WORK/selfdrive/car/dnga/tests/r2_exact_controller_replay.py"
if [ -x /data/openpilot/venv/bin/python3 ]; then
  PY=/data/openpilot/venv/bin/python3
fi

mkdir -p "$OUT/nov05" "$OUT/nov06"
cd "$WORK"
export PYTHONPATH="$WORK:/data/openpilot"
echo "== OFFLINE R2 GIT =="
git rev-parse --short HEAD
echo "== R2 TESTS =="
"$PY" -m unittest discover -s selfdrive/car/dnga/tests -p 'test_longitudinal_r2*.py' -v
echo "== NOVEMBER 5: CHRONOLOGICALLY ORDERED BASELINE =="
"$PY" -u selfdrive/car/dnga/tests/r2_exact_controller_replay.py \
  "$NOV05" --output "$OUT/nov05"
echo "== NOVEMBER 6: UPHILL CURVE RPM CASE =="
"$PY" -u selfdrive/car/dnga/tests/r2_exact_controller_replay.py \
  "$NOV06" --output "$OUT/nov06"
echo "== BOTH RESULT SUMMARIES =="
cat "$OUT/nov05/r2_replay_summary.json"
cat "$OUT/nov06/r2_replay_summary.json"

# Bundle for chat analysis; no raw rlogs included.
OUT="$OUT" "$PY" - <<'PY'
from pathlib import Path
import os,zipfile
out=Path(os.environ['OUT'])
with zipfile.ZipFile(out/'R2_dual_replay_results.zip','w',zipfile.ZIP_DEFLATED,compresslevel=7) as z:
  for d in ('nov05','nov06'):
    for f in (out/d).glob('r2_*'):
      if f.is_file():
        z.write(f,f'{d}/{f.name}')
print('BUNDLE:',out/'R2_dual_replay_results.zip')
PY
echo "OFFLINE R2 DUAL REPLAY COMPLETE"

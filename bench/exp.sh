#!/usr/bin/env bash
# Run a labelled experiment over a seed range and print the aggregate.
# Usage: ./exp.sh <label> <start_seed> <count> [cap=16] [mode]
# Results land in $BENCH_OUT (default bench/out), which is git-ignored.
set -euo pipefail
cd "$(dirname "$0")"
LABEL=$1; START=$2; N=$3; CAP=${4:-16}; MODE=${5:-}
OUTDIR=${BENCH_OUT:-$(pwd)/out}
mkdir -p "$OUTDIR"
OUT=$OUTDIR/${LABEL}.csv
if [ -n "$MODE" ]; then
  seq "$START" $((START + N - 1)) | xargs -P "$(nproc)" -I{} ./bench {} "$CAP" "$MODE" 2>/dev/null \
    | grep -E "^[0-9]+,${CAP}," > "$OUT"
else
  seq "$START" $((START + N - 1)) | xargs -P "$(nproc)" -I{} ./bench {} "$CAP" > "$OUT"
fi
echo "--- $LABEL (seeds $START..$((START+N-1)) cap=$CAP ${MODE}) ---"
python3 - "$OUT" <<'PY'
import sys, statistics as st
rows=[l.split(',') for l in open(sys.argv[1]) if l.strip()]
c=[int(r[4]) for r in rows]; d=[int(r[3]) for r in rows]
we=[int(r[7]) for r in rows]; de=[int(r[8]) for r in rows]; et=[int(r[6]) for r in rows]
n=len(c)
print(f"n={n}  completed={st.mean(c):.2f}  discovered={st.mean(d):.2f}  conv={st.mean(c)/st.mean(d)*100:.0f}%")
print(f"  workerE_left={st.mean(we):.0f}  droneE_left={st.mean(de):.0f}  end_t={st.mean(et):.0f}"
      f"  P(c>=11)={sum(1 for x in c if x>=11)/n:.3f}  P(c>=12)={sum(1 for x in c if x>=12)/n:.3f}"
      f"  P(d>=14)={sum(1 for x in d if x>=14)/n:.3f}")
PY

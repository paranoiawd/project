#!/usr/bin/env bash
# Run N seeds in parallel and aggregate.
# Usage: ./run.sh [num_seeds=300] [cap=20] [start_seed=1]
set -euo pipefail
cd "$(dirname "$0")"
N=${1:-300}
CAP=${2:-20}
START=${3:-1}
OUT=results_cap${CAP}_n${N}_s${START}.csv
seq "$START" $((START + N - 1)) | xargs -P "$(nproc)" -I{} ./bench {} "$CAP" > "$OUT"
python3 aggregate.py "$OUT"

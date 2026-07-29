#!/usr/bin/env bash
# Build every bench tool on Linux (g++). The conio.h shim in bench/shim
# satisfies simulator.h's Windows include.
#   bench         -- the simulation harness (needs schedular.cpp)
#   exact         -- exact optimum by subset DP: the real upper bound
#   verify_exact  -- independent verification of `exact` (see HANDOFF 3a)
#   plan          -- heuristic offline planner: a LOWER bound only
set -euo pipefail
cd "$(dirname "$0")"
g++ -O2 -std=c++17 -w -I shim -o bench bench.cpp ../simulator.cpp ../schedular.cpp
echo "built: bench/bench"
if [ "${1:-}" = "all" ]; then
  for t in exact verify_exact plan; do
    g++ -O2 -std=c++17 -w -I shim -o "$t" "$t.cpp" ../simulator.cpp
    echo "built: bench/$t"
  done
fi

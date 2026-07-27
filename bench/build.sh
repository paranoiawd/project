#!/usr/bin/env bash
# Build the benchmark binary on Linux (g++). The conio.h shim in bench/shim
# satisfies simulator.h's Windows include.
set -euo pipefail
cd "$(dirname "$0")"
g++ -O2 -std=c++17 -w -I shim -o bench bench.cpp ../simulator.cpp ../schedular.cpp
echo "built: bench/bench"

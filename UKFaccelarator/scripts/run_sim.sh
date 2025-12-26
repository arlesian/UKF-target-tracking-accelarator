#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/../verilog/build"
cmake --build . --target run_sim
gtkwave ukf.vcd
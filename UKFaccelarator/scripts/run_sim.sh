#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VERILOG_DIR="$(cd "${SCRIPT_DIR}/../verilog" && pwd)"
BUILD_DIR="${VERILOG_DIR}/build"
VCD_FILE="${BUILD_DIR}/ukf.vcd"

cmake -S "${VERILOG_DIR}" -B "${BUILD_DIR}"
cmake --build "${BUILD_DIR}" --target run_sim

echo "Simulation completed."
echo "Waveform written to: ${VCD_FILE}"

if command -v gtkwave >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
    gtkwave "${VCD_FILE}"
else
    echo "GTKWave not launched automatically."
    echo "In GitHub Codespaces, download ${VCD_FILE} or open it in a compatible waveform viewer."
fi

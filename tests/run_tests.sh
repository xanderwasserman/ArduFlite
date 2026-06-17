#!/usr/bin/env bash
# run_tests.sh — build and run the ArduFlite host-side unit test suite.
#
# Usage: ./tests/run_tests.sh [--clean]
#
# Requirements: CMake ≥ 3.14, a C++17 compiler (clang++ or g++), internet
# access on first run (GoogleTest is fetched automatically by CMake).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/unit/build"

if [[ "${1:-}" == "--clean" ]]; then
    echo "Removing build directory: ${BUILD_DIR}"
    rm -rf "${BUILD_DIR}"
fi

mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

echo "── Configuring ──────────────────────────────────────────────────────────"
cmake .. -DCMAKE_BUILD_TYPE=Debug

echo "── Building ─────────────────────────────────────────────────────────────"
JOBS="$(sysctl -n hw.logicalcpu 2>/dev/null || true)"
if [[ -z "${JOBS}" ]] && command -v nproc >/dev/null 2>&1; then
    JOBS="$(nproc)"
fi
if [[ -z "${JOBS}" ]]; then
    JOBS=2
fi
cmake --build . --parallel "${JOBS}"

echo "── Running tests ────────────────────────────────────────────────────────"
./unit_tests

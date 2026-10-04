#!/usr/bin/env bash
# run_tests.sh — build and run the ArduFlite host-side unit test suite.
#
# Usage: ./tests/run_tests.sh [--clean] [--m32]
#
#   --no-checks  Skip the static guards; run the unit tests only.
#   --san   Build and run with AddressSanitizer + UndefinedBehaviorSanitizer.
#           Catches out-of-bounds access and UB that a plain build executes
#           happily. Slower, and a separate build directory, so it is opt-in
#           rather than part of every run.
#   --m32   Build and run 32-bit, matching the firmware's ILP32 ABI.
#           Needs gcc-multilib (Linux); not available on Apple Silicon.
#           See ADR-056 for why running BOTH widths is the point: the
#           firmware is ILP32 and the host is LP64, and a host-only test
#           result is only as trustworthy as the code's independence from
#           that difference. Any behaviour that differs between the two
#           runs is an ABI dependency, which on a flight controller means
#           the tests describe a program that is not the one flying.
#
# Requirements: CMake ≥ 3.14, a C++20 compiler (clang++ or g++), internet
# access on first run (GoogleTest is fetched automatically by CMake).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/unit/build"
CMAKE_EXTRA=()
SKIP_CHECKS=0

for arg in "$@"; do
    case "$arg" in
        --clean)
            echo "Removing build directory: ${BUILD_DIR}"
            rm -rf "${BUILD_DIR}"
            ;;
        --no-checks)
            SKIP_CHECKS=1
            ;;
        --san)
            BUILD_DIR="${SCRIPT_DIR}/unit/build-san"
            CMAKE_EXTRA+=(-DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer -g"
                          -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address,undefined")
            # UBSan otherwise reports and continues, so a violation would scroll
            # past in a passing run.
            export UBSAN_OPTIONS="print_stacktrace=1:halt_on_error=1"
            ;;
        --m32)
            # A separate build directory: mixing widths in one is a link error
            # that reads as something else entirely.
            BUILD_DIR="${SCRIPT_DIR}/unit/build-m32"
            CMAKE_EXTRA+=(-DCMAKE_CXX_FLAGS=-m32 -DCMAKE_C_FLAGS=-m32
                          -DCMAKE_EXE_LINKER_FLAGS=-m32)
            ;;
        *)
            echo "unknown option: $arg" >&2
            exit 2
            ;;
    esac
done

mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

echo "── Configuring ──────────────────────────────────────────────────────────"
# The +"..." form keeps an EMPTY array from tripping `set -u` on bash 3.2,
# which is what macOS still ships.
cmake .. -DCMAKE_BUILD_TYPE=Debug ${CMAKE_EXTRA[@]+"${CMAKE_EXTRA[@]}"}

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

# ── Static guards, run here rather than only in CI ───────────────────────────
#
# These used to live only in the GitHub workflow. A guard that runs somewhere
# nobody looks is not a guard: the whole point of check_layering.sh is to fail
# at the moment the mistake is made, and the moment the mistake is made is when
# someone runs the tests locally. Both are seconds, and `set -e` at the top of
# this script means either one failing fails the run.
#
# --no-checks skips them for a tight edit/test loop; CI has no such option.
if [[ "${SKIP_CHECKS}" != "1" ]]; then
    echo ""
    "${SCRIPT_DIR}/../tools/ci/check_layering.sh"
    echo ""
    "${SCRIPT_DIR}/../tools/ci/check_target_warnings.sh"
fi

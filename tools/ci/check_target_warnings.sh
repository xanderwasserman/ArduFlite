#!/usr/bin/env bash
# check_target_warnings.sh — compile the portable code FOR THE TARGET, with
# warnings actually turned on.
#
# Why this exists (ADR-057):
#
#   * The firmware IS built for ILP32 locally, by ./build.sh — but the ESP32
#     core passes `-w`. All 186 translation units compile with every warning
#     suppressed, so the one place the code meets a 32-bit compiler is the one
#     place it is told to say nothing.
#   * The host test suite runs LP64. `--m32` closes that gap on Linux, but
#     Apple Silicon has no 32-bit support, so it cannot run on this machine.
#
# This is what is left that DOES run locally on any machine with the ESP32
# toolchain: the real riscv32 compiler, the real ILP32 ABI, strict warnings.
# It is a compile-time check, not a behavioural one — it will not catch a
# runtime ABI dependency the way running the tests 32-bit would. It does catch
# the conversions, sign mismatches and size assumptions that are the usual way
# in.
#
# Files are DISCOVERED, not listed: anything under src/ that compiles without
# Arduino or ESP-IDF headers is portable by definition and gets checked. That
# means the set grows by itself as the platform-call burn-down proceeds, and
# the floor below stops it silently shrinking instead.

set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."

# The count must never fall. A file dropping out of the portable set is either
# a regression or a deliberate change, and both deserve to be noticed — without
# this, a file that stopped compiling standalone would simply stop being
# checked, which is the quietest possible way to lose coverage.
FLOOR=27

CC=""
for candidate in \
    "$HOME"/Library/Arduino15/packages/esp32/tools/esp-rv32/*/bin/riscv32-esp-elf-g++ \
    "$HOME"/.arduino15/packages/esp32/tools/esp-rv32/*/bin/riscv32-esp-elf-g++ \
    "$HOME"/Library/Arduino15/packages/esp32/tools/esp-x32/*/bin/xtensa-esp32-elf-g++ \
    "$HOME"/.arduino15/packages/esp32/tools/esp-x32/*/bin/xtensa-esp32-elf-g++; do
    if [ -x "$candidate" ]; then CC="$candidate"; break; fi
done

if [ -z "$CC" ]; then
    echo "SKIP: no ESP32 cross-compiler found (install the core with arduino-cli)."
    echo "      This check cannot run, so ILP32 warnings are NOT covered here."
    exit 0
fi

# Confirm the compiler really is ILP32 before trusting anything it says.
long_size=$("$CC" -dM -E -x c++ /dev/null 2>/dev/null | awk '/__SIZEOF_LONG__/{print $3}')
if [ "$long_size" != "4" ]; then
    echo "FAIL: $(basename "$CC") reports sizeof(long)=$long_size, expected 4."
    echo "      This check is pointless unless the compiler is the target ABI."
    exit 1
fi

# -Wdouble-promotion is deliberately NOT here, though it is exactly the warning
# a chip with no FPU wants. Its only hits in this codebase are printf-style LOG
# calls, where promoting float to double is mandated by the language and cannot
# be avoided. Enforcing it would mean either casting at every log site or
# accepting permanent noise, and a check that is always slightly failing gets
# ignored. Worth an occasional MANUAL pass to catch real double arithmetic:
#   riscv32-esp-elf-g++ -std=c++20 -I. -Wdouble-promotion -fsyntax-only <file>
WARNINGS="-Wall -Wextra -Wconversion -Wsign-conversion -Wshadow"

echo "── Target warnings (ILP32, $(basename "$CC")) ──"
checked=0
failed=0
while IFS= read -r file; do
    # Portable == compiles with no Arduino or ESP-IDF include path at all.
    "$CC" -std=c++20 -I. -fsyntax-only "$file" >/dev/null 2>&1 || continue
    checked=$((checked + 1))

    output=$("$CC" -std=c++20 -I. $WARNINGS -fsyntax-only "$file" 2>&1)
    if echo "$output" | grep -q "warning:"; then
        echo "$output" | grep "warning:" | sed 's/^/  /'
        failed=1
    fi
done < <(find src -name '*.cpp' | sort)

echo "  $checked portable files compiled clean for the target"

if [ "$checked" -lt "$FLOOR" ]; then
    echo "FAIL: only $checked files checked, floor is $FLOOR."
    echo "      A file stopped compiling standalone, so it is no longer covered."
    echo "      Fix it, or lower FLOOR deliberately and say why."
    exit 1
fi

if [ "$failed" -ne 0 ]; then
    echo "FAIL: warnings above are invisible to ./build.sh, which compiles with -w."
    exit 1
fi

echo "Target warning checks passed."
exit 0

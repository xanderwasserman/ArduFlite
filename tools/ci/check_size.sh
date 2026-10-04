#!/usr/bin/env bash
# check_size.sh — firmware must fit the partition, and must not grow by surprise.
#
# Two limits, for two different failures:
#   HARD      the ESP32-C3 app partition (0x200000). Exceeding it cannot be
#             flashed at all, so this is a hardware fact, not a policy.
#   BUDGET    a generous regression ceiling. Catches unintended growth long
#             before the hard limit. See specs/hal/baseline.md.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

# 0x200000. Both boards use the same app partition (build/*/partitions.csv),
# so one hard limit covers all four builds.
HARD_LIMIT=2097152
BUDGET_LITE=800000
BUDGET_FULL=1750000

fail=0

check() {
    local variant="$1" budget="$2" bin="$3"
    if [ ! -f "$bin" ]; then
        echo "FAIL: $variant did not build"
        fail=1
        return
    fi
    local size
    size=$(stat -f%z "$bin" 2>/dev/null || stat -c%s "$bin")
    local pct=$(( size * 100 / HARD_LIMIT ))

    printf "  %-12s %8d bytes  %3d%% of partition" "$variant" "$size" "$pct"

    if [ "$size" -gt "$HARD_LIMIT" ]; then
        printf "   FAIL: exceeds the app partition — cannot be flashed\n"
        fail=1
    elif [ "$size" -gt "$budget" ]; then
        printf "   FAIL: over the %d byte regression budget\n" "$budget"
        fail=1
    else
        printf "   ok (budget %d)\n" "$budget"
    fi
}

# All four: two boards, two variants. Building only one board means a change
# that breaks the other is invisible until someone builds it by hand.
./build.sh lolin lite > /dev/null 2>&1 || true
./build.sh lolin      > /dev/null 2>&1 || true
./build.sh fire  lite > /dev/null 2>&1 || true
./build.sh fire       > /dev/null 2>&1 || true

echo "Firmware size:"
check "lolin-lite" "$BUDGET_LITE" build/lolin-lite/ArduFlite.ino.bin
check "lolin-full" "$BUDGET_FULL" build/lolin-full/ArduFlite.ino.bin
check "fire-lite"  "$BUDGET_LITE" build/fire-lite/ArduFlite.ino.bin
check "fire-full"  "$BUDGET_FULL" build/fire-full/ArduFlite.ino.bin

exit "$fail"

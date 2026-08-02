#!/usr/bin/env bash
# check_layering.sh — enforce the HAL tier rules mechanically.
#
# specs/hal/05-testing-strategy.md §4. These rules are what keep the abstraction
# honest over time; a convention nobody checks decays.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
fail=0

report() { echo "FAIL: $1"; fail=1; }

# ── 1. No vendor types in portable headers ──────────────────────────────────
# hal/core, hal/platform and hal/device must be portable to a non-Arduino MCU.
PORTABLE="src/hal/core src/hal/platform src/hal/device src/hal/board"
BANNED='HardwareSerial|TwoWire|\bWire\b|Serial\.|\bledc[A-Z]|#include <Arduino\.h>|\bServo\b|\bString\b|Adafruit_|FastIMU'

# Comment lines are stripped first: these headers legitimately DISCUSS the types
# they must not use ("no Print, no Stream", "the legacy Servo::write path").
strip_comments() { sed -E 's,//.*,,; s,/\*.*,,; /^[[:space:]]*\*/d'; }

for dir in $PORTABLE; do
    [ -d "$dir" ] || continue
    while IFS= read -r file; do
        if hits=$(strip_comments < "$file" | grep -nE "$BANNED"); then
            report "vendor type in portable header $file:"
            echo "$hits" | sed 's/^/    /'
        fi
    done < <(find "$dir" -name '*.h' -o -name '*.cpp')
done

# ── 2. Only Board.cpp and the app may name a concrete driver ────────────────
if [ -d src/hal/drivers ]; then
    if hits=$(grep -rn "hal/drivers/" --include=*.cpp --include=*.h src \
                | grep -v "^src/hal/drivers/" \
                | grep -v "^src/hal/board/Board"); then
        report "flight code includes a driver header directly:"
        echo "$hits" | sed 's/^/    /'
    fi
fi

# ── 3. Nothing outside hal/ may include a platform implementation ───────────
if [ -d src/hal/esp32 ]; then
    if hits=$(grep -rn "hal/esp32/" --include=*.cpp --include=*.h src \
                | grep -v "^src/hal/"); then
        report "non-HAL code includes an ESP32 platform header:"
        echo "$hits" | sed 's/^/    /'
    fi
fi

# ── 4. The board #if lives in exactly one place ─────────────────────────────
if hits=$(grep -rn "BOARD_TYPE" --include=*.cpp --include=*.h src include 2>/dev/null \
            | grep -v "^src/hal/"); then
    # Legacy PinConfiguration.h still exists until Phase 8; warn, do not fail.
    echo "NOTE: legacy BOARD_TYPE still present (removed in Phase 8):"
    echo "$hits" | sed 's/^/    /' | head -5
fi

# ── 5. Host platform must not be under src/ ────────────────────────────────
# arduino-cli compiles src/ recursively, so std::thread would land in firmware.
if [ -d src/hal/host ]; then
    report "src/hal/host exists — it would be compiled into the firmware. Use tests/unit/hal_host/."
fi

if [ "$fail" -eq 0 ]; then
    echo "Layering checks passed."
fi
exit "$fail"

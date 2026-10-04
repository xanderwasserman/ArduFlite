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
# src/hal/protocol/ is deliberately NOT a driver: it holds wire-format constants
# and PODs with no bus, pin or register access, so any layer may include it.
# CRSF has two implementations that must agree byte for byte (the receive parser,
# which is a driver, and the telemetry encoder, which is not); the shared wire
# layout has to live somewhere both can reach.
if [ -d src/hal/drivers ]; then
    # Match #include lines only. An earlier version grepped for the bare path
    # and fired on prose that NAMED src/hal/drivers to explain why a file is
    # deliberately not in it — the same false positive the estimation and
    # pause-protocol checks below strip comments to avoid.
    drv_hits=""
    for f in $(find src -name '*.cpp' -o -name '*.h'); do
        case "$f" in
            src/hal/drivers/*|src/hal/board/Board*) continue ;;
        esac
        n=$(grep -nE '^[[:space:]]*#[[:space:]]*include.*hal/drivers/' "$f" | sed "s|^|    $f:|")
        [ -n "$n" ] && drv_hits="$drv_hits$n\n"
    done
    if [ -n "$drv_hits" ]; then
        report "flight code includes a driver header directly:"
        printf "%b" "$drv_hits"
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

# ── 6. The estimation layer stays free of the RTOS ──────────────────────────
# This is the property that makes the twelve-step tick contract testable at all.
# While that logic lived inside ArduFliteIMU::update() — holding a mutex, feeding
# the watchdog, driving the task's own timing — none of it could run anywhere but
# on hardware inside a live task, so the step ORDER was unverifiable. Every
# plausible order compiles and runs; only one is correct (ADR-033).
#
# The subsystem may use hal::Scheduler and hal::Watchdog. It may NOT use the
# RTOS directly — that is the whole distinction.
if [ -d src/estimation ]; then
    est_hits=""
    for f in src/estimation/*.cpp src/estimation/*.h; do
        [ -e "$f" ] || continue
        # Comments legitimately NAME these APIs to explain why they are absent.
        n=$(strip_comments < "$f" \
              | grep -nE "xTaskCreate|vTaskDelay|xSemaphore|TaskHandle_t|esp_task_wdt|portMAX_DELAY|\bmillis\(\)|\bmicros\(\)" \
              | sed "s|^|    $f:|")
        [ -n "$n" ] && est_hits="$est_hits$n\n"
    done
    if [ -n "$est_hits" ]; then
        report "src/estimation must not call the RTOS directly (use hal::Scheduler/Watchdog):"
        printf "%b" "$est_hits"
    fi
fi

# ── 7. The barometer has no task of its own ─────────────────────────────────
# A second task on the sensor mutex caused priority inversion and dropped baro
# samples. The symptom looks like a sensor fault rather than an architectural
# one, which is why this is worth pinning. No behavioural test can assert the
# absence of a task nobody created.
if hits=$(grep -rn "baroTask" src 2>/dev/null); then
    report "the barometer must be read from the IMU task, not its own:"
    echo "$hits" | sed 's/^/    /'
fi

# ── 8. Calibration must not regrow a pause protocol ─────────────────────────
# The old selfCalibrate() suspended the sampling task to take the bus, which
# needed a spin-wait handshake, manual watchdog feeding and careful mutex
# ordering to avoid deadlocking against the task it had just suspended
# (ADR-034). All of it existed because calibration ran outside the task that
# owns the bus.
# NOTE: ArduFliteController::pauseTasks() legitimately uses vTaskSuspend — it
# stops the control surfaces twitching while someone holds the airframe still,
# which is a usability and finger-safety choice, not a bus-ownership workaround.
# This check targets the IMU handshake specifically.
pause_hits=""
for f in $(find src -name '*.cpp' -o -name '*.h'); do
    n=$(strip_comments < "$f" | grep -n "_pauseRequested\|_taskPaused" | sed "s|^|    $f:|")
    [ -n "$n" ] && pause_hits="$pause_hits$n\n"
done
if [ -n "$pause_hits" ]; then
    report "calibration must run inside the sampling task, not by suspending it:"
    printf "%b" "$pause_hits"
fi
if hits=$(grep -rn "vTaskSuspend" src/orientation src/estimation 2>/dev/null); then
    report "the sampling task must never be suspended:"
    echo "$hits" | sed 's/^/    /'
fi

# ── 9. No ABI-dependent scalar type punning ─────────────────────────────────
#
# ADR-056. The Adafruit filter reinterpreted a float through a `long`:
#
#     union { float f; long i; } conv = {x};
#
# `long` is 4 bytes on both ESP32 targets and 8 on a 64-bit host, so on a host
# that union read four bytes of uninitialised memory and returned a NEGATED
# result. It was silent: the value stayed finite and plausible, the filter still
# roughly worked, and every host test measured a filter that does not exist on
# the aircraft.
#
# `std::bit_cast` is the fix and it is free: it will not compile unless the two
# types are the same size, so the bug becomes a build error on the platform that
# is wrong. C++20, which this project already requires (ADR-021).
echo "── Scalar type punning ──"
PUN_UNION='union[[:space:]]*\{[^}]*\b(float|double)\b'
PUN_CAST='reinterpret_cast<[[:space:]]*(unsigned[[:space:]]+)?(long|int|short|float|double)'
for file in $(find src include -name '*.cpp' -o -name '*.h' | sort); do
    if strip_comments < "$file" | tr '\n' ' ' | grep -qE "$PUN_UNION"; then
        echo "  FAIL: $file puns a float through a union - use std::bit_cast"
        fail=1
    fi
    if strip_comments < "$file" | grep -qE "$PUN_CAST"; then
        echo "  FAIL: $file reinterpret_casts a scalar - use std::bit_cast"
        fail=1
    fi
done
[ "$fail" -eq 0 ] && echo "  none (std::bit_cast is the house rule)"

# ── 10. Platform-call burn-down in flight code ──────────────────────────────
# REPORTS, does not fail. Phases 4-7 legitimately still have these; the number
# is the migration's progress metric and must only ever go DOWN.
#
# This check exists because it was missing: the Phase 2 controller migration was
# reported complete on the strength of greps for xTaskCreate/xSemaphoreCreate,
# which all returned zero, while esp_task_wdt_*, ESP.restart() and micros() were
# still there. Choosing the greps that match what you changed finds nothing.
PLATFORM_CALLS='esp_[a-z_]+\(|ESP\.[a-zA-Z]+\(|xTask[A-Za-z]*\(|vTask[A-Za-z]*\(|xSemaphore[A-Za-z]*\(|\bmicros\(\)|\bmillis\(\)|\bledc[A-Z]'

echo ""
echo "── Platform-call burn-down (flight code outside src/hal) ──"
total=0
while IFS= read -r file; do
    n=$(strip_comments < "$file" | grep -cE "$PLATFORM_CALLS")
    if [ "$n" -gt 0 ]; then
        printf "  %4d  %s\n" "$n" "$file"
        total=$((total + n))
    fi
done < <(find src include -name '*.cpp' -o -name '*.h' | grep -v '^src/hal/' | sort)
echo "  ----"
printf "  %4d  TOTAL (must only decrease)\n" "$total"

if [ "$fail" -eq 0 ]; then
    echo ""
    echo "Layering checks passed."
fi
exit "$fail"

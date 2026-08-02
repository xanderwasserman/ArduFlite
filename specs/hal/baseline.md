# Baseline — pre-HAL measurements

**Commit:** `9ca8484` (last commit before HAL work) · **Captured:** 2026-08-02
**Toolchain:** esp32 core 3.3.10, riscv32-esp-elf g++, `-Os`, `-std=gnu++2a`

Reference point for the size budgets in §05 §9 and the Phase 2 regression gate
in §06.

---

## Build size — captured

Both variants, `esp32:esp32:lolin_c3_mini:PartitionScheme=no_ota`.

| Variant | Binary (bytes) | text | data | bss | Notes |
|---|---:|---:|---:|---:|---|
| `lolin-full` | **1,418,832** | 1,216,829 | 226,097 | 1,215,915 | WiFi/HTTP stack + embedded Web UI |
| `lolin-lite` | **630,464** | 500,266 | 130,076 | 534,320 | `-DENABLE_WEB_SERVER=0` |

Web UI assets, full build only: HTML 4,417 + CSS 12,512 + JS 23,321 = 40,213 B,
compressed to 8,976 B (22.3 %).

> `bss` here is the linker's static figure, not runtime heap. It includes IDF
> buffers sized at link time and is useful only as a **relative** number between
> builds — do not read it as "free RAM".

### CI budget

Fail the build if either binary exceeds baseline by more than **3 %**:

| Variant | Baseline | CI ceiling |
|---|---:|---:|
| `lolin-full` | 1,418,832 | 1,461,397 |
| `lolin-lite` | 630,464 | 649,378 |

ADR-002 predicts ~2 KB of vtable growth (≈0.3 % of the lite build), so 3 % is
generous headroom that still catches a real regression. Tighten it after Phase 2
once the actual cost is known.

---

## Loop timing and stack usage — NOT captured

`stats` (inner/outer `avgDt`, `maxDt`, `overrunCount`) and `tasks` (stack
high-water marks) were **not recorded**. The maintainer has opted not to run this
capture, so the plan uses an A/B protocol instead — see below.

### Why this is recoverable

The original plan compared Phase 2 against numbers recorded weeks earlier. That was
never the only option, and arguably not the best one: measurements taken at different
times, temperatures and NVS states are not perfectly comparable anyway.

**Phase 2 instead runs an A/B comparison in a single bench session:**

1. Flash the tagged pre-HAL firmware (`9ca8484`). Board on USB — **no flying, no
   airframe required**; the IMU task and both control loops run on the bench.
2. Capture `stats` over 60 s and `tasks`. This is the baseline.
3. Flash the Phase 2 firmware from the same bench, same session.
4. Capture `stats` and `tasks` again.
5. Compare.

Same board, same session, same conditions — a **better** controlled comparison than
one against a stale recording. Total cost is roughly ten minutes and two flashes.

### What is genuinely lost

Only one thing: **early warning.** With a pre-recorded baseline, a timing regression
introduced in Phase 2 is detectable the moment Phase 2 is testable. With A/B, the
comparison happens at the same point anyway — so in practice nothing is lost, because
Phase 2's gate is the first time the numbers matter.

The one residual risk: if the pre-HAL firmware will no longer build or flash by the
time Phase 2 lands, the A/B baseline is unrecoverable. **Mitigation:** the pre-HAL
tree is tagged and its binaries are reproducible from `9ca8484`; the two `.bin` files
above are also archived by this commit's measurements. Do not delete
`build/lolin-*/ArduFlite.ino.bin` without re-recording.

### Stack sizes

§06 Phase 2 lists "stack sizes audited once against the measured high-water marks".
Without a baseline capture, that audit moves into the same A/B session — read `tasks`
on the pre-HAL firmware, size the `TaskConfig` entries from those numbers plus
headroom, then confirm on the new firmware.

**Until then, Phase 2 must carry over the existing stack sizes verbatim** rather than
guessing at new ones. Changing task stack sizes and the task-creation mechanism in the
same phase would make an overflow impossible to attribute.

---

## Host test suite

| Metric | Value |
|---|---|
| Tests | 115 passed, 1 disabled |
| Disabled | `ProductionContracts.DISABLED_ConfigRegistryRejectsWeakApPasswordAtSetTime` — never passed; see the comment in `test_production_contracts.cpp` |
| Standard | C++17 (raised to C++20 in Phase 0 — ADR-021) |
| Production files compiled | 2 (`pid.cpp`, `FliteQuaternion.cpp`) |

That last row is the number Phase 0 onwards should move.

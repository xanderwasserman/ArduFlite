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

### CI budget — revised after Phase 4

The original 3 % ceiling was set against the pre-HAL baseline on the assumption
that the HAL would add only ~2 KB of vtables. That was the wrong shape: the HAL
legitimately adds *code*, not just dispatch, and by Phase 4 the lite build sat
3.3 KB from tripping a limit that has nothing to do with the hardware.

**The real limit is the app partition**, from `build/*/partitions.csv`:

```
app0, app, ota_0, 0x10000, 0x200000     ->  2,097,152 bytes
```

Two checks instead of one:

| Check | lolin-lite | lolin-full | Rationale |
|---|---:|---:|---|
| **Hard limit** (fail) | 2,097,152 | 2,097,152 | Exceeding this cannot be flashed |
| **Regression budget** (fail) | 800,000 | 1,750,000 | Catches unintended growth well before the hard limit |
| Measured at end of Phase 4 | 646,064 | 1,441,264 | 31 % / 69 % of partition |
| Measured at end of Phase 5 | 635,648 | 1,431,456 | 30 % / 68 % of partition |
| Measured at end of Phase 6 | 633,216 | 1,429,072 | 30 % / 68 % of partition |
| Measured at end of Phase 6B | 633,936 | 1,429,856 | 30 % / 68 % of partition |

Phase 6 came out **smaller** than Phase 5 on both variants, by ~2.4 KB. The
estimation layer's structure, the CRC and the NVS store cost about 3 KB;
dropping the EEPROM backend, its migration path and the ArduFliteIMU façade gave
back more than that.

Phase 5 *reduced* both builds by ~10 KB. Replacing FastIMU and Adafruit_BMP280
with two own drivers also dropped Adafruit_BusIO and Adafruit_Unified_Sensor,
which came in as transitive dependencies. Writing the drivers cost less flash
than depending on libraries that abstract over hardware this project does not
have.

The full build is the one to watch: at 69 % of the partition it has ~656 KB of
real headroom, but it carries the WiFi/HTTP stack. The lite build — the one that
flies — uses under a third of the partition and is not a concern.

**Do not tighten these to track the current size.** A budget that ratchets down
after every phase turns into a tripwire that fires on legitimate work; the point
is to catch a *surprise*, not to enforce a diet.

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
| Tests | 115 passed, 1 disabled (218 after Phase 5; 286 after Phase 6; 294 after Phase 6B; 317 mid-Phase-7) |
| Disabled | `ProductionContracts.DISABLED_ConfigRegistryRejectsWeakApPasswordAtSetTime` — never passed; see the comment in `test_production_contracts.cpp` |
| Standard | C++17 (raised to C++20 in Phase 0 — ADR-021) |
| Production files compiled | 2 (`pid.cpp`, `FliteQuaternion.cpp`) — 7 after Phase 5 |

That last row is the number Phase 0 onwards should move.

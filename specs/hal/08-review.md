# 08 — Critical Review of This Specification

**Status:** Review pass 1, statuses updated after migration-plan revision 2 ·
**Date:** 2026-08-02 · **Reviewer:** self, adversarial

A deliberate attempt to break the design in §00–§07 before any code is written.
Findings are ordered by severity. Each says what is wrong, why it matters, and what
was done about it. **Eleven findings were fixed; two are accepted with their risk
stated; two are deferred with a trigger.** (R10 and R13 moved from *accepted* to
*fixed* when the migration plan was restructured — see revision 2 of §06.)

Phase numbers below refer to the **revision 2** plan in §06.

The uncomfortable summary: the interfaces held up reasonably, but the **contracts
around them did not**. Threading, ownership and lifetime were consistently
under-specified — which is the exact failure mode that produced the current
codebase's problems in the first place.

---

## Blocking — would have caused real bugs

### R1 — `read()` is `const` but returns state that `sample()` mutates. Unsynchronised. — **FIXED**

`Sensor::sample()` refreshes a cache; `Accelerometer::read()` is `const` and
returns it. Nothing in §03 said who may call which, from which task.

Meanwhile `Board` hands out `Span<device::Accelerometer*>` to anybody who asks. So
the telemetry task could hold an `Accelerometer*` and call `read()` while the
inertial task is inside `sample()` writing the same struct — a torn read of exactly
the kind the current `ArduFliteIMU` seqlock was built to prevent. I reintroduced the
bug the existing code already solved, one layer down.

**Fix:** an explicit ownership contract in §03 3.1 — drivers are owned by the
sampling task, `sample()` and `read()` may be called *only* from that task, and
every other consumer reads the published `SeqLock<ImuState>`. `Board`'s sensor spans
are documented as composition-time wiring for `InertialSubsystem`, not a general
data source. Enforced by a host test that asserts a single calling thread id.

### R2 — The sampling loop ignores decimation, contradicting the rest of the design — **FIXED**

§03 3.1 showed:

```cpp
for (auto* dev : board.sensors()) dev->sample();   // one bus pass
```

At the 500 Hz inertial rate that hammers the BMP280 at 500 Hz. The BMP280 conversion
takes ~12 ms; the current firmware decimates to ~50 Hz for exactly this reason, and
§02 4.x claims the factor is derived from `nativeRate_hz()`. The example contradicted
the claim, and if implemented as written would have blown the loop budget and
saturated the I2C bus.

**Fix:** the loop is now rate-aware, with the per-device tick accumulator derived
from `nativeRate_hz()`. Written out in §03 3.1 so the sketch and the claim agree.

### R3 — `Board.h` transitively exposes every driver, defeating the layering check — **FIXED**

ADR-011 makes all drivers *members* of `Board` (static storage, no heap). But then
`Board.h` must `#include` every driver header to declare those members. §05 §4's
include-direction check says flight code may include `Board.h` but not
`hal/drivers/*` — which `Board.h` would smuggle in transitively. The check would have
been theatre.

**Fix:** `Board` gets a pimpl split — `Board.h` declares only interface-typed
accessors and an opaque `BoardStorage& _storage`; `BoardStorage` (with the concrete
driver members) lives in `Board_Internal.h`, included only by `Board.cpp`. Static
storage and the no-heap property are preserved; the include check becomes real.

### R4 — Board validation contradicts the migration plan on day one — **FIXED**

§04 §4 asserts `requiredPeripheralsPresent()` — "RC uart if rcLink != None".
§06 Phase 0 says to record the FireBeetle's unknown CRSF pins as `kNoPin`. The
FireBeetle descriptor sets `rcLink = Crsf`. Those three statements cannot all hold:
Phase 0 would not compile.

**Fix:** `RcPart::Unknown` added, plus a `BoardMaturity { Supported, Untested }`
field. `Untested` boards are excluded from the "fitted parts must be wired" assertion
and log a loud warning at boot. FireBeetle is marked `Untested` until someone puts a
meter on it. Better than either weakening the check or inventing pin numbers.

---

## Significant — design gaps, not yet bugs

### R5 — `InertialSubsystem` is the largest new component and has no interface spec — **FIXED**

It absorbs most of `ArduFliteIMU`: the sampling task, per-sensor axis transforms,
calibration offsets, the low-pass bank, baro decimation, climb rate, health
validation, motion detection, and the seqlock publish. §02 and §06 reference it
repeatedly; §03 never specified it. The single biggest piece of what is now Phase 6
was a name.

**Fix:** new §03 3.8 specifying `InertialSubsystem`, `ImuState`, and the ordering
contract inside a tick.

### R6 — Calibration is hand-waved, and it is the riskiest existing flow — **FIXED**

The current code has `selfCalibrate()`, `baroCalibrate()`, and a cooperative
`pauseTask()`/`_taskPaused` spin-wait protocol with watchdog feeding — built the hard
way, around a real deadlock. §02 4.3 dismisses this in one line ("calibration is just
acquire the device and run a routine"), which is glib: the pause exists because the
IMU task owns the bus and calibration needs it too.

**Fix:** §03 3.8 specifies `CalibrationService` with an explicit state machine
(`Idle → Requested → Running → Complete`) driven *inside* the sampling task, so no
pause protocol is needed at all — the task that owns the bus is the task that
calibrates. That is a genuine simplification the new structure earns, but it had to
be designed rather than asserted.

### R7 — `Gnss` invented a second API shape for no reason — **FIXED**

Every other sensor is `sample()` + `read()`. `Gnss` had `poll()` + `readFix()`.
Two idioms for the same thing.

**Fix:** `Gnss` implements `Sensor`; `sample()` drains the UART and parses,
`readFix()` keeps its "true if new" semantics (genuinely different — a fix is an
event, not a continuous quantity). One idiom, one justified exception, stated.

### R8 — `ARDUFLITE_TRY(dev->begin())` contradicts "each sensor fails independently" — **FIXED**

The §04 §6 sketch used `ARDUFLITE_TRY` inside the sensor loop, which returns from
`Board::begin()` on the first sensor that fails to initialise. Two paragraphs later
the text promises independent degradation. The sketch was wrong.

**Fix:** the loop logs and continues; only *bus* failures are fatal. Also flagged the
general hazard: `ARDUFLITE_TRY` inside a loop is almost always a mistake.

### R9 — Actuator concurrency unspecified — **FIXED**

The control loop calls `write()`/`commit()` at 500 Hz. The failsafe path and
`CommandSystem` call `disable()` from a different task. `PwmActuatorBank` holds slew
state (`lastCommand`). Nothing said whether that is safe.

**Fix:** §03 3.3 states the contract — `write()`/`commit()` are single-writer
(control loop only); `disable()` is the one method safe from any task and is
implemented so it cannot interleave destructively (atomic flag checked in `commit()`,
plus a direct hardware-idle path). A host test drives concurrent `disable()`.

---

## Fixed by restructuring the plan (originally accepted)

### R10 — Actuator work was done twice — **FIXED in §06 revision 2**

Originally: Phase 1 dropped ESP32Servo and converted `ServoManager` to microseconds;
Phase 3 then replaced `ServoManager` entirely. The µs conversion was written twice and
the aircraft bench-verified twice — and servo output is the thing most likely to
damage an airframe, so a duplicated bench gate is a real cost, not a paperwork one.

I originally accepted this on the grounds that the alternative left Tier 0 notional.
That was the wrong call: **`Esp32PwmOut` can be written and unit-tested in Phase 2
without being wired to `ServoManager`.** Tier 0 is complete and proven by host tests;
the hardware cutover happens once, in Phase 3, with all the other actuator work.
Revision 2 does exactly that.

### R13 — One phase carried both high-risk changes — **FIXED in §06 revision 2**


The original Phase 2 covered three register drivers written from scratch, a
cross-check harness, the estimation layer, `CalibrationService`, the seqlock
migration and a façade cutover — estimated at "6–10 sessions". Beyond being
optimistic, the deeper problem was that **the two scariest changes shared one
branch**: if the aircraft flew badly afterwards, "new driver scaling" and "new
estimator" would be indistinguishable.

Revision 2 splits them into Phase 5 (drivers, verified by a cross-check harness
against FastIMU on the same physical sensor) and Phase 6 (estimation, verified by log
replay). Each is independently revertible and has its own oracle. The estimates are
also reframed as a planning aid rather than a commitment.

---

## Accepted with stated risk

### R11 — `Priority` enum has duplicate values — **ACCEPTED**

`Web = 1, Telemetry = 1` and `RcLink = 3, InnerLoop = 3` means `Priority::RcLink ==
Priority::InnerLoop` compares true and the enum cannot be switched exhaustively.
That is genuinely a bit sloppy for a type whose purpose is to make the ladder
explicit.

**Why accepted:** the duplicates are *correct* — those tasks really do share a
FreeRTOS priority, and inventing distinct values to keep the enum injective would
misrepresent the system. Noted so nobody "fixes" it by spreading the values out.

### R12 — `Task::requestStop()` is speculative generality — **ACCEPTED**

No task in ArduFlite ever stops. The method exists because it looked tidy.

**Why accepted:** it is one virtual method and the host `StepScheduler` needs a
teardown hook for tests anyway. Flagged so it does not grow a lifecycle state machine
around it. If it is still unused at Phase 8, delete it.


---

## Deferred, with a trigger

### R14 — Sensor failover can step-change the estimator input

`SensorSelector` switching from `imu0` to `imu1` mid-flight hands the estimator a
different sensor with different bias and (potentially) a different mount transform.
Madgwick will see a discontinuity and take time to reconverge — during which the
attitude estimate is wrong, in flight, immediately after a sensor fault. That is a
bad moment for a transient.

**Deferred because** Phase 6 ships a single sensor and the trivial selector, so the
path is unreachable. **Trigger:** the first time a board descriptor lists two
instances of the same measurement, this needs a designed answer.

**Update — the *path* is now guaranteed, at the maintainer's request.** Failover need
not be implemented now, but adding it later must not be a redesign. Auditing that
claim found three things that would have made it one, all since closed:

| Gap | Consequence had it stayed | Fixed by |
|---|---|---|
| `AttitudeEstimator` had `reset()` but no way to inject a state | re-seeding after a switch was impossible through the interface, though `Adafruit_Madgwick::setQuaternion()` provides it underneath | `setOrientation(Quaternion)` added |
| `ImuState` did not record which instance was live | a failover would be invisible in the flash log — post-flight you could not answer "did it switch?" | `SelectionState` published in `ImuState` |
| `SensorSelector` was named in four documents but never specified | the trivial and sophisticated versions would have been different shapes | interface specified in §03 3.8 |

With those, all three known transition strategies (crossfade, re-seed, median-of-three)
are **additive**: a new `SensorSelector` implementation plus a branch inside
`InertialSubsystem::tick()`. No interface change, nothing above the estimation layer,
no board-descriptor change. See ADR-026.

**Still true:** the transient itself is unsolved. Do not add redundant hardware and
assume failover is free.

### R15 — `openDevice()` lifetime and exhaustion unspecified

`I2cBus::openDevice()` returns `Result<RegisterDevice*>`, but ADR-011 forbids heap
after boot. So the bus holds a fixed array of handles — of what size, and what
happens on exhaustion?

**Deferred because** it is a one-line answer at implementation time (`kMaxDevices =
8`, return `Status::NoSpace`). **Trigger:** must be decided in Phase 2, not left to
whoever types it. Now written into Phase 2's task list so it cannot be forgotten.

---

## Found after the review pass — toolchain assumptions

Two claims in the spec were assumptions rather than checks. Both were wrong, and both
were only caught by reading the installed toolchain.

### R16 — The spec assumed C++17; the firmware is C++20 — **FIXED**

`esp32c3-libs/3.3.10/flags/cpp_flags` ends with `-std=gnu++2a`. Earlier drafts
specified a bespoke `Span<T>` "because C++17 has no `std::span`" — solving a problem
that does not exist. Worse, `tests/unit/CMakeLists.txt` really *is* C++17, so host
tests and firmware were compiling under different language rules.

**Fix:** ADR-021, §09, and a Phase 0 task raising the host suite to C++20 with a CI
check that they stay aligned. `std::span` replaces the bespoke type; `SeqLock` gains
a `requires std::is_trivially_copyable_v<T>` constraint; `Status` becomes
`enum class [[nodiscard]]`.

### R17 — ADR-009 justified `Result<T>` with a flag that is not set — **FIXED**

It claimed the design "matches the ESP32 Arduino default build (`-fno-exceptions`)".
The core compiles with `-fexceptions` **on**. The decision is unchanged and still
correct — unbounded unwind latency has no place in a 500 Hz loop — but a
justification that is factually wrong is worse than no justification, because it
survives review by looking authoritative.

**Fix:** ADR-009's rationale rewritten, with `-fno-exceptions` reframed as something
to *measure* in Phase 0 rather than assume.

**Lesson worth carrying:** both of these were one shell command away from being
correct in the first draft. Anything stated about the toolchain, the hardware or a
dependency should be checked, not recalled — §00 2.9b (the C3 has no FPU) is a third
instance of the same class.

## What held up

Not everything was wrong, and it is worth recording what survived the pass so the
next review does not re-litigate it:

* **The two-tier split** (platform / device) — no finding challenged it.
* **`RegisterDevice`** as the driver-facing bus abstraction — clean, and R1's fix
  strengthened rather than weakened it.
* **`AxisTransform`** — the determinant rule survived scrutiny; the mirrored-map case
  is the one that would have broken a rotation-only design, and it is handled.
* **Per-measurement sensor interfaces** (ADR-019) — the `sample()`/`read()` split is
  what makes it affordable, and R1/R2 sharpened its contract rather than undermining
  it.
* **Transport-neutral `ActuatorBank`** (ADR-020) — R9 added a missing contract; the
  shape itself is sound.
* **Log replay as a regression oracle** — still the highest-value item in the plan.

## Honest overall assessment

The abstractions are in reasonable shape. **The contracts were not**, and that is the
more dangerous kind of gap: an interface with an unstated threading rule looks
finished and fails in the field. Six of the nine fixes above are contract
specifications, not interface changes — which suggests the review was worth doing and
that a second pass should focus on the same class of thing (lifetime, reentrancy,
failure ordering) rather than on interface shape.

Two items to watch that are not findings:

1. **The spec is 3500+ lines and growing.** Its job is to be read before coding. If
   it passes ~4000 lines it starts to fail at that job — prefer sharpening over
   adding from here.
2. **Nothing has been compiled.** Every interface in §03 is plausible C++, not
   verified C++. Phase 0's real purpose is to find out which parts do not survive
   contact with a compiler; expect §03 to need corrections, and update it rather than
   letting the code and the spec diverge on day one.

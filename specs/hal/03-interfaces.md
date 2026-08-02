# 03 — Interface Specification

**Status:** Draft · **Date:** 2026-08-02

These are the proposed headers. They are illustrative sketches, not final code —
Doxygen comments are omitted here for density and will be written in full when the
headers land. Everything lives under namespace `arduflite`.

**Language baseline: C++20** (`-std=gnu++2a`, `-fno-rtti`, `-fexceptions`), verified
against the installed toolchain — see [09-cpp-conventions.md](09-cpp-conventions.md)
for what that enables and the conventions used below (`std::span`, concepts,
`constinit`, standard `Lockable` mutexes, `std::chrono` time).

**Naming rule:** full words, not abbreviations — `arduflite`, `device`, `drivers`,
`descriptor`. Established acronyms are fine (`hal`, `imu`, `pwm`, `uart`, `i2c`,
`spi`, `rc`, `crsf`, `nvs`), as are standard unit symbols in field suffixes
(`_us`, `_hz`, `_pa`, `_dps`, `_deg`, `_rad`, `_g`, `_m`, `_mps`).

---

## 1. `hal/core` — vocabulary types

### 1.1 `Status.h`

```cpp
namespace arduflite {

// [[nodiscard]] on the enum makes EVERY function returning Status warn if its
// result is dropped — the single highest-value line in this header, given that the
// current codebase's failure mode is "log it and carry on". See §09 2.3.
enum class [[nodiscard]] Status : std::uint8_t {
    Ok = 0,
    NotPresent,      // device did not acknowledge — not fitted
    IoError,         // bus transaction failed
    Timeout,
    InvalidArg,
    NotSupported,    // driver does not implement this capability
    NotInitialised,  // begin() not called or failed
    Busy,            // resource held by another owner
    OutOfRange,
    Corrupt,         // CRC / magic / schema mismatch
    NoSpace,
};

const char* toString(Status s);          // for logs and CLI

}   // namespace arduflite
```

### 1.2 `Result.h`

```cpp
namespace arduflite {

template <typename T>
class [[nodiscard]] Result {
public:
    constexpr Result(T value) noexcept(std::is_nothrow_move_constructible_v<T>)
        : _value(std::move(value)), _status(Status::Ok) {}
    constexpr Result(Status error) noexcept : _status(error) {}

    [[nodiscard]] constexpr bool ok() const noexcept { return _status == Status::Ok; }
    [[nodiscard]] constexpr explicit operator bool() const noexcept { return ok(); }
    [[nodiscard]] constexpr Status status() const noexcept { return _status; }

    // Precondition: ok(). Ref-qualified so a temporary moves out instead of copying.
    [[nodiscard]] constexpr const T& value() const &  noexcept { return _value; }
    [[nodiscard]] constexpr T&&      value() &&       noexcept { return std::move(_value); }

    template <class U>
    [[nodiscard]] constexpr T value_or(U&& fallback) const& {
        return ok() ? _value : static_cast<T>(std::forward<U>(fallback));
    }

private:
    T      _value{};
    Status _status;
};

}   // namespace arduflite

// Early-return on failure, preserving the status.
#define ARDUFLITE_TRY(expr)                                                    \
    do {                                                                 \
        const ::arduflite::Status _arduflite_s = (expr);                             \
        if (_arduflite_s != ::arduflite::Status::Ok) return _arduflite_s;                  \
    } while (0)
```

No RTTI (`-fno-rtti`), no heap after boot, and nothing throws. Note that the ESP32
core actually compiles with `-fexceptions` **on** — an earlier draft claimed
otherwise. The reasoning for value-returned errors is unbounded unwind latency in a
500 Hz loop and flash cost, not a compiler flag. See §09 1.1.

`std::expected` would be the natural fit but is C++23; the toolchain resolves to
C++20 (§09 §1).

### 1.3 `Vec3.h`, `Units.h`

```cpp
namespace arduflite {

struct Vec3f {
    float x = 0.0f, y = 0.0f, z = 0.0f;

    Vec3f  operator+(const Vec3f& o) const;
    Vec3f  operator-(const Vec3f& o) const;
    Vec3f  operator*(float s) const;
    float  magnitudeSquared() const;                 // prefer this in hot paths
    float  magnitude() const;
    bool   isFinite() const;                         // NaN/Inf guard, one place
};

namespace units {
    constexpr float kGravityMps2   = 9.80665f;
    constexpr float kDegToRad      = 0.017453292f;
    constexpr float kRadToDeg      = 57.29578f;
    constexpr float kPaToHpa       = 0.01f;
    float altitudeFromPressure(float pressurePa, float referencePa);  // single-precision
}

}   // namespace arduflite
```

**Unit convention (mandatory):** every struct field and every function parameter
carrying a physical quantity is suffixed with its unit —
`accel_g`, `gyro_dps`, `pressure_pa`, `altitude_m`, `climb_mps`, `angle_deg`,
`angle_rad`. This is the cheap 80 % of a unit library; see ADR-008 for why the
full version is deferred. **Time is the exception** — it uses `std::chrono` rather
than a suffix, because the standard library already provides the type (§09 2.2).

### 1.4 `AxisTransform.h` — sensor-to-body alignment

This replaces `ArduFliteIMU::applyOrientation()`. It has to do more than a rotation
enum, for a concrete reason: the aircraft currently flies with a transform whose
determinant is −1 (§00 2.3), which no proper-rotation model can express. A part
whose internal axis sign convention differs from its datasheet — common on
counterfeit MPU-6500 modules — needs a signed axis *map*, not a rotation.

Three layers, each optional, applied in order:

```cpp
namespace arduflite {

// ── Layer 1: signed axis map — all 48 axis-aligned mappings ─────────────────
// (24 proper rotations + 24 reflections). Covers every "the chip is turned round"
// and every "this clone has an inverted axis" case.
enum class SignedAxis : int8_t {
    PlusX = 1, MinusX = -1, PlusY = 2, MinusY = -2, PlusZ = 3, MinusZ = -3
};

struct AxisMap {
    SignedAxis x = SignedAxis::PlusX;   // which sensor axis feeds body X
    SignedAxis y = SignedAxis::PlusY;
    SignedAxis z = SignedAxis::PlusZ;

    constexpr bool isValid()     const;  // each of X/Y/Z appears exactly once
    constexpr int  determinant() const;  // +1 = right-handed, -1 = mirrored
};

// The 24 proper rotations, by name, for the common case. Naming follows ArduPilot
// so datasheets and community mounting advice translate directly.
enum class Rotation : uint8_t {
    None = 0, Yaw90, Yaw180, Yaw270,
    Roll180, Roll180Yaw90, Roll180Yaw180, Roll180Yaw270,
    Roll90, Roll90Yaw90, Roll90Yaw180, Roll90Yaw270,
    Roll270, Roll270Yaw90, Roll270Yaw180, Roll270Yaw270,
    Pitch90, Pitch90Yaw90, Pitch90Yaw180, Pitch90Yaw270,
    Pitch270, Pitch270Yaw90, Pitch270Yaw180, Pitch270Yaw270,
    Count
};
constexpr AxisMap toAxisMap(Rotation r);          // Rotation is a subset of AxisMap

// ── Layer 2: fine alignment trim ────────────────────────────────────────────
// Small Euler offsets for a board that is square-ish but not exactly square in the
// fuselage. Composed into a 3x3 once, at config load — free at runtime.
struct AlignmentTrim { float roll_deg = 0.0f, pitch_deg = 0.0f, yaw_deg = 0.0f; };

// ── Layer 3: the composed transform ─────────────────────────────────────────
class AxisTransform {
public:
    constexpr AxisTransform() = default;
    AxisTransform(AxisMap map, AlignmentTrim trim = {});

    // Per-axis measurements: acceleration, magnetic field.
    Vec3f applyMeasurement(const Vec3f& v) const;

    // Rotation *about* an axis: gyro. Carries det(map), because the right-hand
    // rule flips when the map produces a left-handed frame. Getting this wrong is
    // the classic mirrored-mount bug; making it a separate method makes it
    // impossible to forget.
    Vec3f applyAngularRate(const Vec3f& v) const;

    AxisMap       map()  const;
    AlignmentTrim trim() const;
    bool          isMirrored() const;   // det == -1; surfaced at boot and in the CLI

private:
    float _m[9] = {1,0,0, 0,1,0, 0,0,1};   // map ∘ trim, precomputed
    float _det  = 1.0f;
};

const char*        toString(SignedAxis a);            // "+X", "-Y"
Result<SignedAxis> signedAxisFromString(const char*);
const char*        toString(Rotation r);
Result<Rotation>   rotationFromString(const char*);

}   // namespace arduflite
```

**The current aircraft's transform, expressed exactly:**

```cpp
AxisMap{ SignedAxis::PlusX, SignedAxis::MinusY, SignedAxis::PlusZ }   // det = -1
```

* `applyMeasurement` → `(ax, −ay, az)` — matches `accelY = -accelY`
* `applyAngularRate` → `−(gx, −gy, gz)` = `(−gx, gy, −gz)` — matches
  `gyroX = -gyroX; gyroZ = -gyroZ`

So Phase 2 ports the flying behaviour verbatim as one line of board descriptor,
with no empirical rediscovery needed. The magnetometer becomes `(mx, −my, mz)`,
which corrects the dead-code inconsistency noted in §00 2.3.

**Runtime override.** The board descriptor supplies the default; three
`ConfigRegistry` keys override it without a rebuild:

| Key | Type | Default | Meaning |
|---|---|---|---|
| `imu.axis.x` | string | from board | `"+X"`, `"-Y"`, … — sensor axis feeding body X |
| `imu.axis.y` | string | from board | |
| `imu.axis.z` | string | from board | |
| `imu.align.roll_deg` | float | 0 | fine trim |
| `imu.align.pitch_deg` | float | 0 | |
| `imu.align.yaw_deg` | float | 0 | |

Validation rejects a map that does not use each of X/Y/Z exactly once, so a typo
cannot silently produce a singular transform. Changing an axis requires
`requiresReboot` (or at minimum a disarm) — it must not take effect mid-flight.

**CLI support.** `imu axes` prints the live per-axis readings alongside the active
map, so the correct mapping can be *determined* on the bench rather than guessed:

```
> imu axes
  map    x=+X  y=-Y  z=+Z   (mirrored: yes)   trim r=0.0 p=0.0 y=0.0
  raw    accel  0.02  -0.01   1.00     gyro  -0.1   0.3  -0.2
  body   accel  0.02   0.01   1.00     gyro   0.1   0.3   0.2
  hint   body Z ≈ +1 g with the aircraft level and upright — OK
```

This is the mechanism you asked to keep: whatever the reason your prototype needs
`{+X, −Y, +Z}` — mis-mount, clone part, or an inverted die — you can find it, set
it, and fly, without touching the firmware.

### 1.5 `SeqLock.h`

Lifted from the working seqlock in `ArduFliteIMU`, generalised. Same semantics
including the stale-but-coherent fallback and retry counters.

```cpp
namespace arduflite {

// A seqlock races on its payload by construction, which is only defined behaviour
// for trivially copyable types. C++20 makes that a requirement rather than a comment:
// adding a std::string to ImuState now fails to compile with a readable message,
// instead of corrupting data intermittently in flight.
template <typename T, std::uint32_t kRetryLimit = 8>
    requires std::is_trivially_copyable_v<T>
class SeqLock : private NonCopyable {
public:
    void publish(const T& value) noexcept;               // writer: single thread only

    [[nodiscard]] T    read() const noexcept;            // coherent copy
    [[nodiscard]] bool try_read(T& out) const noexcept;  // caller owns storage

    struct Health { std::uint32_t totalRetries, maxRetries, retryLimitHits; };
    [[nodiscard]] Health health() const noexcept;

private:
    T _current{}, _lastComplete{};
    std::atomic<uint32_t> _version{0}, _lastCompleteVersion{0};
    mutable std::atomic<uint32_t> _totalRetries{0}, _maxRetries{0}, _limitHits{0};
};

}   // namespace arduflite
```

Now unit-testable directly (concurrent read/write stress test on the host), which
the current in-place version is not.

---

## 2. `hal/platform` — Tier 0 interfaces

### 2.1 Time

```cpp
namespace arduflite::hal {

class Clock : private NonCopyable {
public:
    using duration   = std::chrono::microseconds;   // 64-bit: no 71-minute wrap
    using time_point = std::chrono::time_point<Clock, duration>;
    static constexpr bool is_steady = true;

    virtual ~Clock() = default;
    [[nodiscard]] virtual time_point now() const noexcept = 0;
};

}
```

`std::chrono` removes two bug classes at zero runtime cost: the 32-bit wrap in the
current `unsigned long dtMicro = currentMicros - lastMicros`, and "is this
milliseconds or microseconds". Durations subtract to `duration`; the estimator's
`dt_s` is one explicit conversion. See §09 2.2.

### 2.2 Scheduling and synchronisation

```cpp
namespace arduflite::hal {

enum class Priority : uint8_t {
    Cli = 0, Web = 1, Telemetry = 1,
    OuterLoop = 2, RcLink = 3, InnerLoop = 3, Inertial = 4,
};

struct TaskConfig {
    const char* name;
    uint32_t    stackBytes;
    Priority    priority;
    int8_t      core = -1;          // -1 = no affinity
};

class Task {                        // opaque handle
public:
    virtual ~Task() = default;
    virtual void requestStop() = 0;
    virtual bool isRunning() const = 0;
};

class Scheduler {
public:
    virtual ~Scheduler() = default;
    virtual Result<Task*> spawn(const TaskConfig&, void (*entry)(void*), void* arg) = 0;
    virtual void sleepMs(uint32_t ms) = 0;
    // Fixed-cadence sleep (vTaskDelayUntil). `lastWake` is opaque scheduler state.
    virtual void sleepUntil(uint64_t& lastWakeUs, uint32_t periodMs) = 0;
    virtual void yield() = 0;
};

// Satisfies the standard Lockable / TimedLockable requirements, so std::lock_guard,
// std::unique_lock and std::scoped_lock work directly. There is no bespoke lock
// wrapper — SemaphoreLock and the earlier draft's ScopedLock are both deleted.
class Mutex : private NonCopyable {
public:
    virtual ~Mutex() = default;

    virtual void lock() = 0;                             // unbounded — init paths only
    [[nodiscard]] virtual bool try_lock() = 0;
    virtual void unlock() noexcept = 0;

    // TimedLockable. Cannot be virtual (templated on duration), so it forwards.
    template <class Rep, class Period>
    [[nodiscard]] bool try_lock_for(std::chrono::duration<Rep, Period> d) {
        return try_lock_for_us(
            std::chrono::duration_cast<std::chrono::microseconds>(d).count());
    }

protected:
    [[nodiscard]] virtual bool try_lock_for_us(std::int64_t us) = 0;
};

class Watchdog {
public:
    virtual ~Watchdog() = default;
    virtual Status registerCurrentTask() = 0;
    virtual void   feed() = 0;
    virtual Status unregisterCurrentTask() = 0;
};

}
```

### 2.3 Buses

`RegisterDevice` is the key abstraction: a driver written against it works over I2C
or SPI unchanged. Directly inspired by `AP_HAL::Device`.

```cpp
namespace arduflite::hal {

class RegisterDevice {
public:
    virtual ~RegisterDevice() = default;

    virtual Status readRegs (uint8_t reg, uint8_t* dst, size_t len) = 0;
    virtual Status writeRegs(uint8_t reg, const uint8_t* src, size_t len) = 0;

    Status readReg (uint8_t reg, uint8_t& out) { return readRegs(reg, &out, 1); }
    Status writeReg(uint8_t reg, uint8_t val)  { return writeRegs(reg, &val, 1); }

    // The bus-wide mutex. A driver that needs several transactions to be atomic
    // takes std::unique_lock(dev.busLock()). Single transactions lock internally.
    virtual Mutex& busLock() = 0;

    virtual const char* busName() const = 0;    // "i2c0@0x68" — for logs
};

class I2cBus {
public:
    virtual ~I2cBus() = default;
    virtual Status begin(uint32_t clockHz) = 0;
    // Handles are carved out at composition time and live as long as the bus.
    virtual Result<RegisterDevice*> openDevice(uint8_t address7bit) = 0;
    virtual Status probe(uint8_t address7bit) = 0;      // ACK test only
};

class SpiBus {
public:
    virtual ~SpiBus() = default;
    virtual Status begin() = 0;
    virtual Result<RegisterDevice*> openDevice(uint8_t csPinIndex, uint32_t clockHz, uint8_t mode) = 0;
};

// Declared now so the actuator abstraction (§3.3) is demonstrably transport-neutral.
// Not implemented until something needs it; the ESP32-C3's TWAI controller makes
// Esp32CanBus a real option rather than a hypothetical one.
struct CanFrame {
    uint32_t id;
    uint8_t  data[8];
    uint8_t  length;
    bool     extendedId;
};

class CanBus {
public:
    virtual ~CanBus() = default;
    virtual Status begin(uint32_t bitrate_bps) = 0;
    virtual Status send(const CanFrame& frame) = 0;
    virtual bool   receive(CanFrame& out) = 0;      // true if a frame was waiting
    virtual Mutex& busLock() = 0;
};

class Uart {
public:
    virtual ~Uart() = default;
    virtual Status begin(uint32_t baud, bool invertRx = false) = 0;
    virtual size_t available() = 0;
    virtual size_t read (uint8_t* dst, size_t maxLen) = 0;
    virtual size_t write(const uint8_t* src, size_t len) = 0;
    virtual void   flush() = 0;
};

}
```

No `Print`, no `Stream`, no `String` — G3.

### 2.4 Pins and outputs

```cpp
namespace arduflite::hal {

enum class PinMode : uint8_t { Input, InputPullUp, InputPullDown, Output };

class GpioPin {
public:
    virtual ~GpioPin() = default;
    virtual void setMode(PinMode) = 0;
    virtual bool read() const = 0;
    virtual void write(bool high) = 0;
};

class PwmOut {
public:
    virtual ~PwmOut() = default;
    virtual Status attach(uint16_t minUs, uint16_t maxUs, uint16_t frameHz = 50) = 0;
    virtual void   writeMicroseconds(uint16_t us) = 0;
    virtual void   idle() = 0;      // stop pulsing — used by ActuatorBank::disable()
    virtual void   detach() = 0;
};

}
```

`PwmOut` works in microseconds, not degrees. The current `Servo::write(int degrees)`
path quantises to 1° (~11 µs) before the slew limiter sees it; microseconds are the
hardware's actual unit and remove that quantisation.

### 2.5 Storage and system

```cpp
namespace arduflite::hal {

class KeyValueStore {               // NVS on ESP32
public:
    virtual ~KeyValueStore() = default;
    virtual Status read (const char* key, void* dst, size_t capacity, size_t& outLen) = 0;
    virtual Status write(const char* key, const void* src, size_t len) = 0;
    virtual Status erase(const char* key) = 0;
    virtual Status commit() = 0;
};

struct FileInfo { char name[32]; uint32_t sizeBytes; };

class FileStore {                   // LittleFS on ESP32
public:
    virtual ~FileStore() = default;
    virtual Status begin() = 0;
    virtual Result<int>  open  (const char* path, bool forWrite) = 0;   // returns handle
    virtual Status       append(int handle, const void* src, size_t len) = 0;
    virtual Result<size_t> read(int handle, void* dst, size_t maxLen) = 0;
    virtual Status       close (int handle) = 0;
    virtual Status       remove(const char* path) = 0;
    virtual size_t       list  (FileInfo* out, size_t maxEntries) = 0;
    virtual Status       format() = 0;
    virtual Status       usage (uint32_t& usedBytes, uint32_t& totalBytes) = 0;
};

enum class ResetCause : uint8_t { PowerOn, Software, Panic, Watchdog, Brownout, Unknown };

class System {
public:
    virtual ~System() = default;
    virtual ResetCause  resetCause() const = 0;
    virtual uint32_t    freeHeapBytes() const = 0;
    virtual uint32_t    minFreeHeapBytes() const = 0;
    virtual const char* uniqueId() const = 0;
    [[noreturn]] virtual void reboot() = 0;
};

}
```

---

## 3. `hal/device` — Tier 2 interfaces

### 3.1 Sensors — one interface per *measurement*, not per *chip*

**Design rule:** an interface describes a quantity that can be measured, never a
part number and never a bundle of quantities that happen to share a package. A 6-DOF
IMU is not a thing the flight code knows about; it is a chip that happens to
implement `Accelerometer` **and** `Gyroscope`. Custom hardware with a discrete
accelerometer and a discrete gyro produces the same view of the world, and
redundant sensors are just more entries in a list.

#### The chip / measurement split

Two orthogonal concepts, deliberately not in an inheritance relationship:

```cpp
namespace arduflite::device {

enum class SensorHealth : uint8_t { Unknown, Ok, Degraded, Failed, NotPresent };

// ── A physical chip on a bus. One per part, whatever it measures. ───────────
class Sensor {
public:
    virtual ~Sensor() = default;

    virtual Status   probe() = 0;          // WHO_AM_I / ACK; NotPresent if absent
    virtual Status   begin() = 0;

    // Perform the bus transaction(s) that refresh EVERY reading this chip
    // provides. The sampling task calls this once per device per tick.
    // This is the ONLY method that touches the bus.
    virtual Status   sample() = 0;

    virtual uint16_t nativeRate_hz() const = 0;
    virtual const char*  name() const = 0;     // "MPU-6500", "ADXL345"
    virtual SensorHealth health() const = 0;
};

// ── Measurement interfaces. Independent; no common base. ───────────────────
// read() returns the value cached by the last sample(). It never touches the
// bus, is const, and is safe to call from any task.

struct AccelSample { Vec3f accel_g;      hal::Clock::time_point time; };
struct GyroSample  { Vec3f rate_dps;     hal::Clock::time_point time; };
struct MagSample   { Vec3f field_ut;     hal::Clock::time_point time; };
struct BaroSample  { float pressure_pa;  float temp_c; hal::Clock::time_point time; };

class Accelerometer {
public:
    virtual ~Accelerometer() = default;
    virtual Status  read(AccelSample& out) const = 0;
    virtual Status  setRange_g(uint8_t g) = 0;
    virtual uint8_t range_g() const = 0;
};

class Gyroscope {
public:
    virtual ~Gyroscope() = default;
    virtual Status   read(GyroSample& out) const = 0;
    virtual Status   setRange_dps(uint16_t dps) = 0;
    virtual uint16_t range_dps() const = 0;
};

class Magnetometer {
public:
    virtual ~Magnetometer() = default;
    virtual Status read(MagSample& out) const = 0;
};

class Barometer {
public:
    virtual ~Barometer() = default;
    virtual Status read(BaroSample& out) const = 0;
};

}   // namespace arduflite::device
```

A combined part implements several:

```cpp
class Mpu6500 final : public device::Sensor,
                      public device::Accelerometer,
                      public device::Gyroscope
{
public:
    Status sample() override;                          // ONE 14-byte burst read
    Status read(device::AccelSample& out) const override;   // cached, free
    Status read(device::GyroSample&  out) const override;   // cached, free
};

class Mpu9250 final : public device::Sensor,
                      public device::Accelerometer,
                      public device::Gyroscope,
                      public device::Magnetometer { /* … */ };
```

Multiple inheritance of pure interfaces — no state, no diamond, no virtual bases,
so no layout or dispatch penalty beyond one vtable pointer per base.

**The `sample()` / `read()` split is what makes this efficient**, and it is a
strict improvement over the combined `ImuSensor` in the previous draft. The
MPU-6500 delivers accel + temperature + gyro in a single 14-byte burst from
`ACCEL_XOUT_H`. With separate `Accelerometer::read()` / `Gyroscope::read()` calls
that each hit the bus, that would be **two transactions per tick instead of one** —
a real cost at 500 Hz on a 400 kHz bus. Splitting "refresh from hardware" from
"give me the value" gets per-measurement interfaces *and* the burst read.

The same split generalises: a board with a discrete ADXL345 and a discrete L3GD20
has two `Sensor`s, each sampling itself. The flight code above cannot tell the
difference — which is exactly the point.

#### The rest of the sensor set

Specified now so the shape is fixed, even where no driver exists yet:

```cpp
namespace arduflite::device {

struct GnssFix {
    double   latitude_deg, longitude_deg;    // double: 1e-7 deg needs the precision
    float    altitudeMsl_m;
    float    groundSpeed_mps, courseOverGround_deg;
    float    hdop, vdop;
    uint8_t  satellites;
    enum class Type : uint8_t { None, Dead, Fix2D, Fix3D, Dgps, Rtk } type;
    hal::Clock::time_point time;
    std::chrono::milliseconds utcEpoch{};
};

// GNSS implements Sensor like everything else: sample() drains the UART and
// parses any complete frames. The ONE justified deviation is readFix()'s return
// type — a fix is an event that may or may not have arrived, not a continuously
// available quantity, so "did I get a new one" is part of the answer.
class Gnss {
public:
    virtual ~Gnss() = default;
    virtual bool readFix(GnssFix& out) const = 0;   // true if newer than last call
};
// Concrete driver: class UbloxGnss : public Sensor, public Gnss { … };

struct AirspeedSample { float differential_pa; float indicated_mps; hal::Clock::time_point time; };
class Airspeed     { public: virtual Status read(AirspeedSample&) const = 0; /* + Sensor */ };

struct RangeSample  { float distance_m; uint8_t quality_pct; hal::Clock::time_point time; };
class RangeFinder  { public: virtual Status read(RangeSample&) const = 0;
                             virtual float maxRange_m() const = 0; };

struct PowerSample  { float voltage_v; float current_a; float consumed_mah; hal::Clock::time_point time; };
class PowerMonitor { public: virtual Status read(PowerSample&) const = 0; };

struct TempSample   { float temp_c; hal::Clock::time_point time; };
class Thermometer  { public: virtual Status read(TempSample&) const = 0; };

}   // namespace arduflite::device
```

`PowerMonitor` is worth having early: `ArdufliteCRSFTelemetry` already sends a
battery frame, currently with placeholder values.

#### Redundancy and selection

The board exposes **lists**, not single pointers:

```cpp
namespace arduflite::board {

class Board {
    // …
    std::span<device::Sensor* const>  sensors();   // everything needing sample()
    std::span<device::Accelerometer* const> accelerometers();
    std::span<device::Gyroscope* const>     gyroscopes();
    std::span<device::Magnetometer* const>  magnetometers();
    std::span<device::Barometer* const>     barometers();
    device::Gnss*                gnss();             // nullptr if not fitted
    device::Airspeed*            airspeed();
    device::PowerMonitor*        powerMonitor();
};

}
```

Returned as `std::span<T* const>` — the toolchain is C++20 (§09 §1), so no bespoke
`Span` type is needed. `const` on the pointee-pointer stops a caller rebinding the
board's list.

#### Threading contract — read this before using any sensor interface

**A driver's cached sample is unsynchronised state.** `sample()` writes it; `read()`
reads it. There is no lock, and adding one would put a mutex in the 500 Hz path.
The contract that makes this safe:

> `sample()` and `read()` may be called **only from the sampling task that owns the
> device**. Every other consumer — control loops, telemetry, CLI, web, state machine —
> reads the published `SeqLock<ImuState>` (§3.8), never a driver.

`Board`'s sensor spans exist for **composition-time wiring** — they are how
`InertialSubsystem` is handed its devices at construction. They are not a general
data source, and no other subsystem should call them. Without this rule the design
would reintroduce exactly the torn-read hazard the existing `ArduFliteIMU` seqlock
was built to prevent, one layer further down.

Enforced by an L1 host test: `TrackingMutex`-style thread-id assertion on the fake
bus, failing if `sample()`/`read()` are ever entered from two threads.

#### The sampling loop is rate-aware

Sensors run at wildly different rates — 500 Hz inertial, ~25 Hz barometric, 5–10 Hz
GNSS. `sample()` must be called at each device's own rate, not at the task rate:

```cpp
// InertialSubsystem, once per tick. Each device carries its own accumulator,
// sized at begin() from nativeRate_hz() — no #define, no mirrored constant.
for (uint8_t i = 0; i < _deviceCount; ++i) {
    if (++_tickAccumulator[i] < _decimation[i]) continue;
    _tickAccumulator[i] = 0;
    const Status s = _devices[i]->sample();          // the only bus access
    if (s != Status::Ok) _health.recordFailure(i, s);
}

AccelSample a; _selector.primaryAccel()->read(a);    // cached, free, no bus
GyroSample  g; _selector.primaryGyro()->read(g);
```

with `_decimation[i] = max(1, taskRate_hz / device->nativeRate_hz())`.

This is what replaces `BARO_DECIMATION_FACTOR`, its two `static_assert`s and the
constants mirrored into `test_baro_decimation.cpp`. A device that reports 25 Hz gets
sampled every 20th tick automatically; adding a 5 Hz GNSS needs no new constant.

**Selection is a flight-layer concern, not a HAL one.** `estimation::SensorSelector`
holds the policy: pick the first `SensorHealth::Ok` instance, fall over on failure
with hysteresis so a marginal sensor cannot flap. Phase 2 ships the trivial version
(index 0, fall back on `Failed`); full voting or median-of-three can be added later
without touching a single interface. The important property is that the interfaces
**do not preclude it** — which single-pointer accessors would.

Each sensor also carries its own `AxisTransform` (§1.4), because on custom hardware
a discrete gyro and a discrete accelerometer can be mounted at different angles.
The board descriptor gives one per sensor instance, not one per board.

### 3.2 RC input

```cpp
namespace arduflite::device {

struct RcFrame {
    static constexpr uint8_t kMaxChannels = 16;
    uint16_t channel_us[kMaxChannels] = {};   // normalised by the driver to 988..2012 µs
    uint8_t  channelCount = 0;
    hal::Clock::time_point time{};
};

struct RcLinkStats {
    uint8_t linkQuality_pct = 0;
    int8_t  rssi_dbm        = 0;
    int8_t  snr_db          = 0;
    bool    valid           = false;          // false until the first stats frame
};

class RcLink {
public:
    virtual ~RcLink() = default;

    virtual Status begin() = 0;
    // True if a frame newer than the last call is available.
    virtual bool   readFrame(RcFrame& out) = 0;
    virtual bool   isFailsafe() const = 0;
    virtual void   setFailsafeTimeout(std::chrono::milliseconds) = 0;
    virtual RcLinkStats stats() const = 0;
    virtual const char* name() const = 0;     // "CRSF", "PWM"
};

}
```

Deliberately absent: `ChannelConfig`, `ChannelType`, `ChannelCallback`,
`configureChannel()`. All of that moves to `input::RcMapper` — see §3.6.

`RcLink` yields microseconds so PWM, CRSF and SBUS are genuinely interchangeable
(CRSF's 11-bit values are converted by `CrsfLink`).

### 3.3 Actuator output — two levels, transport-neutral

**Revised after review.** The first draft had a single `ActuatorBank` with
index-based `write(idx, value)`. That conflated two genuinely different things: *one
output*, and *one transport's worth of outputs*. Splitting them is the maintainer's
suggestion and it is right — see ADR-025 for what my original counter-arguments got
wrong.

```cpp
namespace arduflite::device {

enum class OutputRange : uint8_t { Bipolar, Unipolar };   // [-1,1] vs [0,1]

// What KIND of thing this output drives. Orthogonal to the transport: a binary
// retract can hang off PWM or CAN. Affects slew, quantisation and failsafe.
enum class ActuatorKind : uint8_t {
    Proportional,  // control surface, throttle - continuous, slew-limited
    Binary,        // retract, relay, cowl flap - snaps to min/max, slew ignored
    Latching,      // parachute, payload release - one-shot; disable() must NOT
                   // actuate it, and it is never handed to the mixer at all
};

enum class FailsafeAction : uint8_t {
    Hold,       // freeze at the last commanded value
    Neutral,    // drive to the configured neutral
    Release,    // stop driving entirely (no pulse / no CAN command / torque off)
};

enum class ActuatorState : uint8_t {
    Ok,
    Saturated,   // command clipped by travel limits
    Stale,       // last commit() did not reach the hardware
    Fault,       // device reported a fault
    Offline,     // device not responding (CAN node dropped, etc.)
};

// Transport-NEUTRAL per-channel configuration.
struct ActuatorChannelConfig {
    const char*    role           = "";        // "elevator", "throttle", "retract"
    ActuatorKind   kind           = ActuatorKind::Proportional;
    OutputRange    range          = OutputRange::Bipolar;
    bool           invert         = false;
    float          trim           = 0.0f;
    float          minOutput      = -1.0f;
    float          maxOutput      =  1.0f;
    float          maxSlew_perSec = 4.0f;      // 0 = unlimited; ignored for Binary
    FailsafeAction onDisable      = FailsafeAction::Neutral;
};

struct ActuatorFeedback {
    float         position;     // normalised, MEASURED (vs lastCommand())
    float         current_a;
    float         temp_c;
    std::uint16_t faultFlags;
    hal::Clock::time_point time;
};

// ─────────────────────────────────────────────────────────────────────────────
// ONE output. This is what flight code holds — by NAME, not by index.
// ─────────────────────────────────────────────────────────────────────────────
class Actuator : private NonCopyable {
public:
    virtual ~Actuator() = default;

    // Stage a command. Normalised per the channel's range. Applies trim, travel
    // limits, inversion and slew. NaN/Inf holds the previous value.
    // Purely local — no bus access, cannot fail. The owning bank's commit() is
    // what reaches the hardware.
    virtual void stage(float normalised) = 0;

    [[nodiscard]] virtual float         lastCommand() const = 0;  // post-slew, post-clamp
    [[nodiscard]] virtual ActuatorState state()       const = 0;
    [[nodiscard]] virtual ActuatorKind  kind()        const = 0;
    [[nodiscard]] virtual const char*   role()        const = 0;

    [[nodiscard]] virtual bool hasFeedback() const { return false; }
    virtual Status readFeedback(ActuatorFeedback& out) const {
        (void)out; return Status::NotSupported;
    }
};

// Returned by commit(). staleMask bit N set = the Nth actuator of this bank is
// still holding a previous command because its transport failed.
struct CommitResult {
    Status        status         = Status::Ok;
    std::uint32_t staleMask      = 0;
    std::uint8_t  committedCount = 0;
    [[nodiscard]] constexpr bool allOk() const {
        return status == Status::Ok && staleMask == 0;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// ONE TRANSPORT'S worth of outputs. A bank exists because commit and disable are
// batched BUS operations — they belong to a transport, not to an output.
// ─────────────────────────────────────────────────────────────────────────────
class ActuatorBank : private NonCopyable {
public:
    virtual ~ActuatorBank() = default;

    virtual Status begin(std::span<const ActuatorChannelConfig> cfgs) = 0;

    // Flight code resolves these ONCE at composition time and then holds
    // Actuator& — it does not index into the bank per tick.
    [[nodiscard]] virtual std::span<Actuator* const> actuators() = 0;
    [[nodiscard]] virtual Actuator* byRole(std::string_view role) = 0;  // nullptr if absent

    // Push every staged command to the hardware, atomically WITHIN this bank.
    //   PWM:     writes the LEDC duty registers.
    //   CANopen: transmits the mapped PDOs, then a SYNC.
    //   DShot:   emits the frame burst.
    virtual CommitResult commit() = 0;

    // Applies each channel's FailsafeAction. Safe to call from ANY task — the
    // failsafe path, CommandSystem and the CLI all need it. Latching channels are
    // never actuated by this.
    virtual Status disable() = 0;

    // The rate this transport wants to be committed at. PWM 50-333 Hz, DShot up to
    // 32 kHz, CANopen whatever the PDO budget allows. Symmetric with
    // Sensor::nativeRate_hz(); lets the caller decimate rather than over-driving a
    // 50 Hz servo bus from a 500 Hz loop.
    [[nodiscard]] virtual uint16_t nativeRate_hz() const = 0;

    [[nodiscard]] virtual const char* transport() const = 0;   // "PWM", "CANopen"
};

}   // namespace arduflite::device
```

#### Why two levels, and what each is for

| | `Actuator` | `ActuatorBank` |
|---|---|---|
| Is | one output | one **transport's** outputs |
| Flight code holds | `Actuator&`, by name | nothing, per tick |
| Operations | `stage()`, `state()`, `readFeedback()` | `commit()`, `disable()`, `begin()` |
| Exists because | an output must be nameable, typed and individually fed back | commit and disable are **batched bus operations** |

**The bank is not an arbitrary collection — it is a transport group.** That is the
answer to "why control the whole bank at once": CANopen's SYNC makes every servo on
that bus act on the same control cycle, and a CAN disarm should be one broadcast
rather than five SDO writes. Those are properties of the *bus*, and there is nowhere
else to put them. Everything genuinely per-output moved to `Actuator`.

#### What this buys at the call site

```cpp
// Composition time, in arduflite_init() — resolved once, by role, checked once.
struct ControlOutputs {
    Actuator& aileronLeft;
    Actuator& aileronRight;
    Actuator& elevator;
    Actuator& rudder;
    Actuator* throttle;      // nullptr on a glider
};

// Per tick — no magic indices, no index/role mismatch possible.
outputs.elevator.stage(cmd.pitch);
outputs.aileronLeft.stage(cmd.rollLeft);
// …
const auto result = bank.commit();
```

Three concrete wins over the index-based bank:

1. **No magic indices.** `write(2, x)` becomes `elevator.stage(x)`. An index/role
   mismatch after a board-descriptor edit is a whole bug class that stops existing.
2. **Per-output feedback without gymnastics.** `readFeedback(idx, out)` becomes
   `elevator.readFeedback(out)` — a CAN servo reporting position and current is
   queried where it is used.
3. **Latching actuators become *structurally* safe.** A parachute release is simply
   never placed in `ControlOutputs`, so the mixer cannot reach it — a compile-time
   guarantee rather than a runtime `ActuatorKind` check someone must remember to
   honour. This is the strongest argument for the split.

#### Threading contract

`Actuator` holds slew state and `ActuatorBank` owns a bus, so neither is
free-threaded:

> `stage()`, `commit()`, `begin()` are **single-writer** — the control loop only.
> `disable()` is the **one method callable from any task** and must remain so: a
> disarm that has to wait for a lock is not a disarm.

`disable()` is an atomic latch plus a direct hardware-idle path, so it never
interleaves destructively with an in-flight `commit()`: once latched, `commit()`
becomes a no-op until re-armed.

#### Where the transport-specific settings went

Into the **driver's construction**, supplied by the board descriptor — never into the
shared interface:

```cpp
// drivers/out/PwmActuatorBank.h
struct PwmChannelTuning { uint16_t minPulse_us, neutralPulse_us, maxPulse_us, frameRate_hz; };
class PwmActuatorBank final : public device::ActuatorBank {
public:
    PwmActuatorBank(std::span<hal::PwmOut* const> pins,
                    std::span<const PwmChannelTuning> tuning);
};

// drivers/out/CanOpenActuatorBank.h
struct CanServoNode {
    uint8_t  nodeId;
    uint16_t targetIndex;  uint8_t targetSubIndex;   // object dictionary entry
    int32_t  countsAtMin,  countsAtMax;              // encoder counts <-> normalised
    uint16_t heartbeat_ms;
};
class CanOpenActuatorBank final : public device::ActuatorBank {
public:
    CanOpenActuatorBank(hal::CanBus& bus, std::span<const CanServoNode> nodes);
};
```

`AirframeMixer` and `ArduFliteController` see neither struct. They see `Actuator&`,
resolved by role at composition time.

#### Mixed transports on one aircraft

Four PWM surfaces, a CAN throttle, a PWM retract and a latching payload release is a
realistic configuration, so it must not require a special case.

```cpp
// A bank of banks. Composite pattern: it IS an ActuatorBank, so anything that
// takes one takes this.
class CompositeActuatorBank final : public device::ActuatorBank {
public:
    CompositeActuatorBank(std::span<device::ActuatorBank* const> banks);

    std::span<Actuator* const> actuators() override;      // concatenation, bank order
    Actuator* byRole(std::string_view role) override;     // searches every sub-bank

    // Issues each sub-commit back to back with no work in between, then ORs the
    // per-bank staleMasks into the composite index space. A failure in one bank
    // does NOT abort the others — the remaining banks must still be driven.
    CommitResult commit() override;

    // Disables EVERY bank unconditionally, even if an earlier one errors.
    // A partial disarm is worse than a failed one.
    Status disable() override;

    // Slowest sub-bank wins: committing faster than the slowest transport wants
    // gains nothing and costs bus bandwidth.
    uint16_t nativeRate_hz() const override;
};
```

Because flight code holds `Actuator&` resolved by role, **it cannot tell which bank
an output came from** — which is exactly what makes mixed transports a non-event
above the HAL. The elevator being PWM and the throttle being CAN is a fact about
`Board.cpp` and nothing else.

#### Three things a composite bank cannot hide

**1 — `commit()` is atomic *within* a bank, never *across* banks.** CANopen's SYNC
makes every servo on that bus act on the same control cycle. It does nothing for the
PWM surfaces, which moved microseconds earlier. At 1 Mbit a five-node PDO group plus
SYNC is roughly 0.6 ms against a 2 ms loop budget — real but small, and bounded by
the transports themselves because the composite issues sub-commits back to back.

> **Design guidance the code cannot enforce:** keep the *primary flight surfaces on
> one bank*. Split roll across a PWM aileron and a CAN aileron and you have built a
> rolling-moment asymmetry that appears only under bus load. Put ailerons + elevator
> + rudder on one transport; hang throttle, retracts and payload on whatever else is
> convenient.

**2 — Partial failure is a real flight state and needs a policy.** If the CAN bank
drops and the PWM bank does not, some surfaces follow the controller and some are
frozen. `CommitResult::staleMask` and `Actuator::state()` exist so the flight layer
can tell *which*, and decide: a stale flap is a log line, a stale elevator is a
failsafe. **That policy lives above the HAL** — the bank reports accurately, it does
not choose.

**3 — Rate mismatch is the caller's problem, and now it has the data.**
`nativeRate_hz()` lets the controller decimate per bank exactly as the inertial task
decimates the barometer.

Endpoint mapping, inversion, trim, travel limits and slew stay on the `Actuator` for
every transport — per-output calibration that must apply to *every* writer, including
manual passthrough and failsafe. Airframe mixing does not.


### 3.4 Indicator

```cpp
namespace arduflite::device {

struct Rgb { uint8_t r, g, b; };
struct BlinkPattern { Rgb colour; uint16_t onMs, offMs; uint8_t repeats; };  // 0 = forever

class Indicator {
public:
    virtual ~Indicator() = default;
    virtual Status begin() = 0;
    virtual void   setColour(Rgb) = 0;
    virtual void   setPattern(const BlinkPattern&) = 0;
    virtual void   off() = 0;
};

}
```

### 3.5 Storage and console

```cpp
namespace arduflite::device {

class LogStore {                 // flight logs
public:
    virtual ~LogStore() = default;
    virtual Status begin() = 0;
    virtual Result<uint16_t> startSession(const char* prefix) = 0;   // returns index
    virtual Status appendLine(const char* line, size_t len) = 0;
    virtual Status endSession() = 0;
    virtual bool   isRecording() const = 0;
    virtual size_t listSessions(hal::FileInfo* out, size_t maxEntries) = 0;
    virtual Status readSession(uint16_t index, void* dst, size_t maxLen, size_t& outLen) = 0;
    virtual Status removeSession(uint16_t index) = 0;
    virtual Status usage(uint32_t& used, uint32_t& total) = 0;
    virtual Status formatAll() = 0;
};

class SettingsStore {            // calibration blobs, NVS-backed
public:
    virtual ~SettingsStore() = default;
    virtual Status load(const char* key, void* dst, size_t len) = 0;   // Corrupt on CRC fail
    virtual Status save(const char* key, const void* src, size_t len) = 0;
    virtual Status erase(const char* key) = 0;
};

class Console {                  // CLI + logging transport
public:
    virtual ~Console() = default;
    virtual size_t write(const char* s, size_t len) = 0;
    virtual size_t readLine(char* dst, size_t maxLen) = 0;   // 0 if no complete line
};

}
```

`SettingsStore` replaces raw EEPROM for IMU calibration (§00 2.9), adds a CRC, and
gives one persistence mechanism instead of two.

### 3.6 What moves *out* of the device layer

| Was | Becomes | Lives in |
|---|---|---|
| `ChannelConfig` + `ChannelCallback` inside the CRSF driver | `input::RcMapper` — role mapping, tri-state thresholds, deadband, callbacks | `src/input/` |
| `WingDesign` mixing inside `ServoManager` | `actuators::AirframeMixer` — pure function `(roll, pitch, yaw, throttle) → float[]` | `src/actuators/` |
| Madgwick calls inside `ArduFliteIMU` | `estimation::AttitudeEstimator` interface + `MadgwickEstimator` | `src/estimation/` |
| `updateMotionSignals()` | `estimation::MotionDetector` | `src/estimation/` |
| `FlightState` in `ImuSnapshot` | owned solely by `StateManagement` | `src/state/` |

`AirframeMixer` is a pure function with no state and no hardware — the tests in
`test_servo_math.cpp` stop mirroring the formulas and call the real one.

### 3.7 `estimation::AttitudeEstimator` — the seam for replacing Adafruit_AHRS

Not part of the HAL tiers (it touches no hardware), but specified here because it is
the interface that makes ADR-017 possible: swapping the fusion filter must be a
one-line change in the composition root, provable by log replay.

```cpp
namespace arduflite::estimation {

class AttitudeEstimator {
public:
    virtual ~AttitudeEstimator() = default;

    // sampleRate_hz seeds any internal rate assumptions; dt is still passed per
    // update so a jittery task period stays correct.
    virtual Status begin(uint16_t sampleRate_hz) = 0;
    virtual void   reset() = 0;

    // Gyro/accel only (no magnetometer) — the current MPU-6500 path.
    virtual void update(const Vec3f& gyro_dps,
                        const Vec3f& accel_g,
                        float dt_s) = 0;

    // With magnetometer, for a 9-axis part. Default: ignore mag and fall through,
    // so a 6-axis-only estimator need not implement it.
    virtual void updateWithMag(const Vec3f& gyro_dps,
                               const Vec3f& accel_g,
                               const Vec3f& mag_ut,
                               float dt_s) { update(gyro_dps, accel_g, dt_s); }

    virtual Quaternion  orientation()  const = 0;
    virtual EulerAngles euler_deg()    const = 0;

    // Filter gain — Madgwick beta today. Named generically so a Mahony or
    // complementary filter can implement it without an interface change.
    virtual void  setGain(float gain) = 0;
    virtual float gain() const = 0;

    virtual const char* name() const = 0;   // "Madgwick (Adafruit)", "Madgwick"
};

}   // namespace arduflite::estimation
```

Two implementations, in this order:

| Phase | Class | Notes |
|---|---|---|
| 6 | `AdafruitMadgwickEstimator` | Wraps `Adafruit_Madgwick` unchanged. **Must replicate its quirks exactly**, including `getYaw()`'s `+180.0f` offset and the `anglesComputed` caching semantics — otherwise the Phase 7 replay comparison has no valid baseline |
| 9 | `MadgwickEstimator` | Own implementation (ADR-017). Validated by replaying `FL001`/`FL002` through both and comparing quaternions |

Selection is one line in `Board.cpp` / `InertialSubsystem` construction, so Phase 9
is revertible without touching flight code.

**Consequence worth stating plainly:** `Vec3f`/`Quaternion`/`EulerAngles` in and out,
`float dt_s` — no `Wire`, no `micros()`, no FreeRTOS. The estimator is a pure
function of its inputs, so a host test can drive it from a CSV column and assert on
the output. That is the whole mechanism behind L3 replay (§05 §7).

`FliteQuaternion` is renamed `arduflite::Quaternion` and moves to `hal/core/` in
Phase 0. Mechanical, and it is already covered by `test_quaternion.cpp`, so the
rename is safe.

### 3.8 `estimation::InertialSubsystem` — the sampling task

The largest single component in the migration, and absent from the first draft of
this spec (review finding R5). It absorbs most of `ArduFliteIMU`.

```cpp
namespace arduflite::estimation {

// Published to every other task. Sensor data only — FlightState is NOT here;
// StateManagement owns it (§00 2.8).
struct ImuState {
    Vec3f       accel_g;         // body frame, calibrated, filtered
    Vec3f       gyro_dps;        // body frame, calibrated, filtered
    Vec3f       mag_ut;          // body frame; zero if no magnetometer
    Quaternion  orientation;
    EulerAngles euler_deg;
    float       altitude_m;      // above the calibrated ground reference
    float       climbRate_mps;
    MotionSignals motion;        // launchDetected / stableDetected
    bool        healthy;
    hal::Clock::time_point time;
};

class InertialSubsystem {
public:
    struct Dependencies {
        std::span<device::Sensor* const>  devices;      // everything to sample()
        std::span<device::Accelerometer* const> accelerometers;
        std::span<device::Gyroscope* const>     gyroscopes;
        std::span<device::Magnetometer* const>  magnetometers;
        std::span<device::Barometer* const>     barometers;
        AttitudeEstimator&           estimator;
        device::SettingsStore&       settings;     // calibration blobs
        hal::Clock&                  clock;
        hal::Scheduler&              scheduler;
        hal::Watchdog&               watchdog;
    };

    explicit InertialSubsystem(const Dependencies&);

    Status begin(uint16_t taskRate_hz);      // probes, loads calibration, sizes decimation
    Status startTask();

    ImuState                 state() const;  // lock-free seqlock read — THE public API
    SeqLock<ImuState>::Health snapshotHealth() const;

    CalibrationService&      calibration();
};

}   // namespace arduflite::estimation
```

**Ordering contract inside one tick** — fixed, and depended upon by the tests:

1. feed the watchdog
2. rate-aware `sample()` pass (§3.1) — the only bus access in the system
3. `read()` from the selected instances — cached, no bus
4. apply each sensor's `AxisTransform` (`applyMeasurement` / `applyAngularRate`)
5. subtract calibration offsets
6. low-pass filter bank
7. validate (NaN/Inf, range) → update health
8. on baro ticks only: altitude EMA and climb-rate derivative
9. `estimator.update(gyro_dps, accel_g, dt_s)`
10. `motionDetector.update(...)`
11. service any pending calibration request (below)
12. `_state.publish(...)` — one seqlock write, last

Step 4 happens **before** selection is meaningful: each instance is transformed with
its own map, so a failover between differently-mounted sensors still yields body-frame
data. (The transient this causes is review finding R14 — deferred, not solved.)

#### Calibration without a pause protocol

The current code pauses the IMU task via `_pauseRequested`/`_taskPaused` spin-waits
with watchdog feeding, because calibration needs the bus that the task owns. That
protocol exists to solve a problem the new structure does not have.

```cpp
class CalibrationService {
public:
    enum class Kind  : uint8_t { Gyro, Accel, Barometric };
    enum class State : uint8_t { Idle, Requested, Running, Complete, Failed };

    Status request(Kind);            // thread-safe; any task may call
    State  state() const;
    uint8_t progressPercent() const;
    Status lastError() const;
};
```

`request()` sets an atomic flag. **Step 11 of the tick runs the accumulation** — the
task that owns the bus is the task that calibrates, so there is nothing to pause and
no deadlock to avoid. A 10-second gyro calibration is 5000 ticks of accumulation, not
a blocking loop, so the control loops keep running on the last good state and the
watchdog keeps being fed by the normal path.

Results persist through `device::SettingsStore` (NVS + CRC), replacing the raw
EEPROM blob. This is a real simplification the new structure earns — but it had to be
designed, not asserted.

---

## 4. `hal/board` — descriptor and composition root

Covered in detail in [04-board-descriptors.md](04-board-descriptors.md). The
flight-facing surface is small:

```cpp
namespace arduflite::board {

class Board {
public:
    static Board& instance();                 // the ONLY singleton in the design
    Status begin();                           // constructs, probes, wires. Once.

    // Platform
    hal::Clock&     clock();
    hal::Scheduler& scheduler();
    hal::Watchdog&  watchdog();
    hal::System&    system();

    // Sensors — lists, so redundant instances need no interface change.
    // Empty span = none fitted. See §3.1.
    std::span<device::Sensor* const>  sensors();   // everything needing sample()
    std::span<device::Accelerometer* const> accelerometers();
    std::span<device::Gyroscope* const>     gyroscopes();
    std::span<device::Magnetometer* const>  magnetometers();
    std::span<device::Barometer* const>     barometers();

    // Single-instance devices — nullptr means "not fitted on this board"
    device::Gnss*             gnss();
    device::Airspeed*         airspeed();
    device::RangeFinder*      rangeFinder();
    device::PowerMonitor*     powerMonitor();
    device::Indicator*        indicator();

    device::RcLink&           rc();
    device::ActuatorBank&     outputs();     // may be a CompositeActuatorBank
    device::LogStore&         logs();
    device::SettingsStore&    settings();
    device::Console&          console();

    const BoardDescriptor& descriptor() const;
};

}
```

**`Board.h` must not leak driver types (review finding R3).** ADR-011 makes drivers
members of `Board` for static storage — but that would force `Board.h` to include
every driver header, and flight code includes `Board.h`. The layering check in §05 §4
would have been unenforceable. So `Board` is split:

```
Board.h            interface-typed accessors + `BoardStorage& _storage`  (opaque)
Board_Internal.h   struct BoardStorage — the concrete driver members
Board.cpp          the only translation unit that includes Board_Internal.h
```

`BoardStorage` is one file-scope object in `Board.cpp`, so static allocation and the
no-heap-after-boot property are unchanged. Flight code now physically cannot name a
driver type.

**Why one singleton is acceptable here and `hal` in AP_HAL is not:** `Board` is
touched by exactly one function — `arduflite_init()` — which pulls references out
and injects them into constructors. No driver, no controller and no telemetry
backend includes `Board.h`. That is enforceable by the include-direction check, and
it keeps every other class testable with fakes.

# 09 — C++ Conventions and Toolchain

**Status:** Draft · **Date:** 2026-08-02

What language level is actually available, and how the HAL uses it. Written after
checking the toolchain rather than assuming it — two of the assumptions in earlier
drafts were wrong.

---

## 1. What the toolchain actually provides

From `~/Library/Arduino15/packages/esp32/tools/esp32c3-libs/3.3.10/flags/cpp_flags`:

```
-fno-rtti
-std=gnu++2b        ← overridden
-fexceptions
-std=gnu++2a        ← last -std wins in GCC
```

| Fact | Value | Consequence |
|---|---|---|
| C++ standard | **C++20** (`gnu++2a`) | `std::span`, concepts, `constinit`, `consteval`, designated initialisers, `[[likely]]`, `std::bit_cast` are all available |
| RTTI | **off** (`-fno-rtti`) | No `dynamic_cast`, no `typeid`. Not needed |
| Exceptions | **on** (`-fexceptions`) | See §1.1 — this contradicts an earlier claim |
| Host tests | **C++17** | Mismatch. Fixed in Phase 0 |
| FPU | **none** on ESP32-C3 (`rv32imc`, soft-float ABI) | §00 2.9b. Float is emulated |

### 1.1 Correction: exceptions are enabled, not disabled

ADR-009 justified `Status`/`Result<T>` partly on the grounds that it "matches the
ESP32 Arduino default build (`-fno-exceptions`)". **That was wrong** — the core
enables exceptions.

The decision does not change, but the honest reasoning does:

* Exceptions have **unbounded latency**. A 500 Hz control loop cannot have a path
  whose worst-case timing depends on stack unwinding.
* Unwind tables cost flash. The lite build's budget is the constraint that matters.
* `Result<T>` composes with `Status` returns from C-style ESP-IDF APIs, which is what
  the drivers actually wrap.

**Worth measuring in Phase 0, not asserting:** adding `-fno-exceptions` to
`build.sh`'s extra flags may reclaim meaningful flash. It is *not* obviously safe —
the prebuilt core libraries were compiled with exceptions on, so anything that
throws across that boundary would terminate rather than unwind. Measure the size
delta, and only adopt it if the build is clean and the aircraft flies. If in doubt,
leave exceptions enabled and simply never throw.

### 1.2 Host and firmware must agree

The host test suite is C++17 (`tests/unit/CMakeLists.txt:4`). That is a real hazard:
host-only code could compile against C++17 rules and behave differently, and
`std::span`/concepts/`constinit` would be unavailable in tests but present in
firmware. **Phase 0 raises it to C++20** and adds a CI check that the two match.

---

## 2. Conventions

### 2.1 Mutexes — use the standard, delete the bespoke type

`hal::Mutex` satisfies the standard *Lockable* named requirement, so the standard
lock types work directly and `SemaphoreLock`/`ScopedLock` are deleted:

```cpp
namespace arduflite::hal {

class Mutex : private NonCopyable {
public:
    virtual ~Mutex() = default;

    // Lockable — exact names matter; this is what makes std:: locks work.
    virtual void lock() = 0;
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

}   // namespace arduflite::hal
```

Usage, replacing the hand-rolled RAII wrapper:

```cpp
using namespace std::chrono_literals;

// Bounded wait — the common control-loop case.
std::unique_lock lock(bus.busLock(), 5ms);
if (!lock) { /* timed out — use last known values */ return; }

// Unbounded — init paths only.
std::lock_guard guard(bus.busLock());

// Two mutexes, deadlock-free.
std::scoped_lock both(a, b);
```

**Why this is better than the current `SemaphoreLock`:** no FreeRTOS type in any
signature; `std::scoped_lock` gives deadlock-free multi-lock for free; the
`[[nodiscard]]` on `try_lock` makes ignoring a failed acquisition a warning; and
readers already know the semantics.

### 2.2 Time — `std::chrono`, not raw integers

```cpp
class Clock : private NonCopyable {
public:
    using duration   = std::chrono::microseconds;   // 64-bit: no 71-minute wrap
    using time_point = std::chrono::time_point<Clock, duration>;
    static constexpr bool is_steady = true;

    virtual ~Clock() = default;
    [[nodiscard]] virtual time_point now() const noexcept = 0;
};
```

Sample timestamps become `Clock::time_point`, not `uint64_t time_us`. Durations
subtract to `duration`, and `dt` for the estimator is an explicit conversion:

```cpp
const auto dt = clock.now() - _lastSample;
const float dt_s = std::chrono::duration<float>(dt).count();
```

This removes two whole bug classes — integer wrap and "is this milliseconds or
microseconds" — at zero runtime cost, since `chrono` is entirely compile-time.

**Scope note:** ADR-008 chose unit *suffixes* over strong unit types. `chrono` is not
an exception to that so much as the obvious case where the standard library already
provides the type. Physical quantities with no standard type (`gyro_dps`,
`pressure_pa`) keep suffixes.

### 2.3 `Result<T>` and `Status` — move-aware, hard to ignore

```cpp
enum class [[nodiscard]] Status : std::uint8_t { Ok = 0, NotPresent, /* … */ };

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
    [[nodiscard]] constexpr const T&  value() const &  noexcept { return _value; }
    [[nodiscard]] constexpr T&&       value() &&       noexcept { return std::move(_value); }

    template <class U>
    [[nodiscard]] constexpr T value_or(U&& fallback) const& {
        return ok() ? _value : static_cast<T>(std::forward<U>(fallback));
    }

private:
    T      _value{};
    Status _status;
};
```

`enum class [[nodiscard]]` is the highest-value line here: **every** function
returning `Status` now warns if its result is dropped. In a codebase whose current
failure mode is "log it and carry on", that is worth more than the rest of this
section combined.

`std::expected` would be the natural fit but is C++23; the toolchain is C++20.
Revisit if the core moves to `gnu++2b` as its final `-std`.

### 2.4 `SeqLock<T>` — constrained by concept

A seqlock races on the payload by construction, which is only defined behaviour for
trivially copyable types. C++20 lets that be a requirement rather than a comment:

```cpp
template <typename T, std::uint32_t kRetryLimit = 8>
    requires std::is_trivially_copyable_v<T>
class SeqLock : private NonCopyable {
public:
    void publish(const T& value) noexcept;              // writer: one thread only

    [[nodiscard]] T read() const noexcept;              // returns a coherent copy
    [[nodiscard]] bool try_read(T& out) const noexcept; // no temporary, no fallback

    struct Health { std::uint32_t totalRetries, maxRetries, retryLimitHits; };
    [[nodiscard]] Health health() const noexcept;
};
```

Adding a `std::string` or a `std::vector` to `ImuState` now fails to compile with a
readable message, instead of producing an intermittent corruption in flight.

`try_read()` exists for callers that already own storage — telemetry copies
`ImuState` once per publish cycle, and there is no reason to build a temporary first.

### 2.5 Copies, moves and what actually matters here

Being honest about this, because it is easy to cargo-cult:

**Most "copies" in this design are of trivially copyable PODs, where move is
identical to copy.** `ImuState`, `AccelSample`, `RcFrame` and `TelemetryData` are all
memcpy-able aggregates. Adding `std::move` around them achieves nothing, and
`SeqLock` *requires* them to stay that way (§2.4). Do not "optimise" these.

Where copying genuinely matters, and what is done about it:

| Case | Approach |
|---|---|
| `SeqLock::read()` returning ~100 B by value | Necessary for coherence. `try_read(out)` offered where the caller owns storage |
| Passing sample structs down a call chain | `const T&`. Never by value below the interface boundary |
| Lists of device pointers | `std::span<T* const>` — a pointer and a length, never a container copy |
| Strings in CLI/config parsing | `std::string_view`. No `String`, no allocation |
| `Result<T>` with a non-trivial `T` | Ref-qualified `value() &&` moves out (§2.3) |
| Board descriptor | `constexpr` — lives in flash, never copied at all |

The one place real move semantics earn their keep is `Result<T>`; everything else is
solved by not copying in the first place.

### 2.6 `std::span` instead of a bespoke `Span`

Earlier drafts specified a hand-written `Span<T>` "because C++17 has no
`std::span`". C++20 does. Use it:

```cpp
[[nodiscard]] std::span<device::Accelerometer* const> accelerometers() const noexcept;
```

`const` on the pointee-pointer prevents a caller rebinding the board's list.

### 2.7 `constinit` for the composition root

```cpp
// Board.cpp
constinit Board g_board{};
```

`constinit` guarantees constant initialisation — a compile error if the object would
need dynamic init. That is a direct, checked answer to the static-initialisation-order
problem that the existing `initFromConfig()` deferred-init pattern works around by
convention. The pattern stays (config still loads after FreeRTOS starts), but the
*ordering hazard* is now the compiler's problem.

### 2.8 `consteval` for board validation

```cpp
consteval bool isValid(const BoardDescriptor& b);
static_assert(isValid(kBoard), "Board descriptor invalid — see specs/hal/04");
```

`consteval` guarantees the check runs at compile time. With `constexpr` it *may*
be deferred to runtime under some circumstances; `consteval` cannot be.

### 2.9 Interfaces are non-copyable

```cpp
class NonCopyable {
protected:
    constexpr NonCopyable() noexcept = default;
    ~NonCopyable() = default;
public:
    NonCopyable(const NonCopyable&)            = delete;
    NonCopyable& operator=(const NonCopyable&) = delete;
};
```

Every Tier 0 and Tier 2 interface derives from it. Prevents accidental slicing of a
driver into its base — a silent, hard-to-find bug otherwise. The protected
non-virtual destructor also stops deletion through a `NonCopyable*`.

### 2.10 RAII for hardware resources

The watchdog is currently managed by manual `esp_task_wdt_add()` / `_reset()` pairs
scattered across five task functions:

```cpp
class WatchdogGuard : private NonCopyable {
public:
    explicit WatchdogGuard(hal::Watchdog& wd) : _wd(wd) { _wd.registerCurrentTask(); }
    ~WatchdogGuard()                                    { _wd.unregisterCurrentTask(); }
    void feed() noexcept { _wd.feed(); }
private:
    hal::Watchdog& _wd;
};

void InertialSubsystem::task(void* arg) {
    auto& self = *static_cast<InertialSubsystem*>(arg);
    WatchdogGuard wdt(self._watchdog);       // registered for the task's lifetime
    for (;;) { wdt.feed(); self.tick(); /* … */ }
}
```

### 2.11 Smaller rules

* `[[nodiscard]]` on every getter and every fallible call.
* `noexcept` on anything that cannot fail — it is free documentation, and with
  exceptions enabled (§1.1) it is not merely decorative.
* `[[likely]]` / `[[unlikely]]` on the seqlock retry branch and the health-check
  fast path. Marginal, free.
* Designated initialisers are standard in C++20 — the board descriptors in §04 rely
  on them and no longer need to be a GNU extension.
* `std::bit_cast` for register decoding instead of `union` or `reinterpret_cast`.
* `std::array` over C arrays in the board descriptor — same layout, real `size()`,
  no decay.
* Prefer `enum class` with an explicit underlying type everywhere (already done).
* **No** `std::function` in the hot path — it may allocate. Use a plain function
  pointer plus a `void*` context, as the CRSF callbacks already do.
* **No** heap after `Board::begin()` returns (ADR-011), verified by a host test that
  installs an aborting `operator new`.

---

## 3. What this changes in the spec

| Item | Was | Now |
|---|---|---|
| `SemaphoreLock` / `ScopedLock` | bespoke RAII wrapper | deleted; `std::unique_lock<hal::Mutex>` |
| `Mutex::tryLock(uint32_t ms)` | ad-hoc API | Lockable: `lock` / `try_lock` / `try_lock_for` |
| `Clock::micros()` → `uint64_t` | raw integer | `Clock::now()` → `chrono::time_point` |
| `Span<T>` | hand-written | `std::span` |
| `SeqLock<T>` | unconstrained template | `requires std::is_trivially_copyable_v<T>` |
| `Status` | plain enum class | `enum class [[nodiscard]]` |
| Board object | plain global | `constinit` |
| `validate::isValid` | `constexpr` | `consteval` |
| Watchdog | manual add/reset calls | `WatchdogGuard` RAII |
| Host tests | C++17 | C++20, matched to firmware and CI-checked |

All of these land in **Phase 0** (core types and interfaces) and **Phase 2**
(platform implementations), which is where they are cheapest — before there are call
sites to migrate.

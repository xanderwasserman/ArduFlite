/**
 * HostPlatform.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Host implementations of the Tier 0 interfaces, for tests.
 *
 * Deliberately NOT under src/ — arduino-cli compiles src/ recursively, so this
 * would link std::mutex and std::thread into the flight firmware. See
 * specs/hal/08-review.md R20.
 */
#ifndef ARDUFLITE_TESTS_HAL_HOST_PLATFORM_H
#define ARDUFLITE_TESTS_HAL_HOST_PLATFORM_H

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <vector>

#include "src/hal/platform/Clock.h"
#include "src/hal/platform/Io.h"
#include "src/hal/platform/Mutex.h"
#include "src/hal/platform/RegisterDevice.h"
#include <cstring>
#include <algorithm>
#include <map>
#include <optional>
#include <string>

#include "src/hal/core/Crc32.h"
#include "src/hal/device/Peripherals.h"
#include "src/hal/platform/Scheduler.h"
#include "src/hal/platform/Storage.h"
#include "src/hal/platform/Watchdog.h"

namespace arduflite::hal::host {

/**
 * @brief Time advances only when the test says so.
 *
 * Deterministic dt, no sleeps, no flake — a 10-second flight runs in
 * microseconds.
 */
class VirtualClock final : public Clock
{
public:
    [[nodiscard]] time_point now() const noexcept override
    {
        return time_point{ duration{ _us.load(std::memory_order_relaxed) } };
    }

    void advance(duration d) noexcept
    {
        _us.fetch_add(d.count(), std::memory_order_relaxed);
    }

    void advanceMs(std::int64_t ms) noexcept { advance(duration{ ms * 1000 }); }
    void setUs(std::int64_t us) noexcept { _us.store(us, std::memory_order_relaxed); }

private:
    std::atomic<std::int64_t> _us{ 0 };
};

/// std::mutex behind the HAL interface.
class HostMutex final : public Mutex
{
public:
    void lock() override                     { _m.lock(); }
    [[nodiscard]] bool try_lock() override   { return _m.try_lock(); }
    void unlock() noexcept override          { _m.unlock(); }

protected:
    [[nodiscard]] bool try_lock_for_us(std::int64_t us) override
    {
        return _m.try_lock_for(std::chrono::microseconds{ us });
    }

private:
    std::timed_mutex _m;
};

/// Recursive variant, matching the RegisterDevice::busLock() guarantee.
class HostRecursiveMutex final : public Mutex
{
public:
    void lock() override                     { _m.lock(); }
    [[nodiscard]] bool try_lock() override   { return _m.try_lock(); }
    void unlock() noexcept override          { _m.unlock(); }

protected:
    [[nodiscard]] bool try_lock_for_us(std::int64_t us) override
    {
        return _m.try_lock_for(std::chrono::microseconds{ us });
    }

private:
    std::recursive_timed_mutex _m;
};

/**
 * @brief Records every pulse written, so servo behaviour is assertable.
 */
class RecordingPwmOut final : public PwmOut
{
public:
    struct Write { std::uint16_t us; Clock::time_point at; };

    explicit RecordingPwmOut(const Clock* clock = nullptr) : _clock(clock) {}

    Status attach(std::uint16_t minUs, std::uint16_t maxUs,
                  std::uint16_t frameRate_hz = 50) override
    {
        if (minUs >= maxUs) { return Status::InvalidArg; }
        _minUs = minUs; _maxUs = maxUs; _frameHz = frameRate_hz;
        _attached = true;
        return Status::Ok;
    }

    void writeMicroseconds(std::uint16_t us) override
    {
        if (!_attached) { return; }
        if (us < _minUs) { us = _minUs; }
        if (us > _maxUs) { us = _maxUs; }
        _lastUs = us;
        _idle   = false;
        writes.push_back({ us, _clock ? _clock->now() : Clock::time_point{} });
    }

    void idle() override    { _idle = true;  _lastUs = 0; }
    void detach() override  { _attached = false; }

    [[nodiscard]] std::uint16_t lastMicroseconds() const override { return _lastUs; }
    [[nodiscard]] bool isIdle()     const noexcept { return _idle; }
    [[nodiscard]] bool isAttached() const noexcept { return _attached; }

    std::vector<Write> writes;

private:
    const Clock*  _clock    = nullptr;
    std::uint16_t _minUs    = 1000;
    std::uint16_t _maxUs    = 2000;
    std::uint16_t _frameHz  = 50;
    std::uint16_t _lastUs   = 0;
    bool          _attached = false;
    bool          _idle     = false;
};

/**
 * @brief Scriptable register map with a transaction counter.
 *
 * The counter is what lets a test assert the ADR-019 contract directly:
 * sample() performs exactly ONE bus transaction and read() performs none.
 */
class FakeRegisterDevice final : public RegisterDevice
{
public:
    Status readRegs(std::uint8_t reg, std::uint8_t* dst, std::size_t len) override
    {
        ++readCount;
        ++transactions;
        if (failNextRead > 0) { --failNextRead; return Status::IoError; }

        // Lets a test model a device whose registers change as it is polled.
        if (onRead) { onRead(reg); }

        // Some parts prepend filler bytes to every read — the BMI323 emits two
        // on I2C. It is a BUS artefact, generated on read, not something stored
        // in the register file, so it is modelled here rather than by shifting
        // the register contents. Modelling it the other way makes a
        // read-modify-write read a different address than it writes.
        if (dummyPrefixBytes > 0)
        {
            std::size_t i = 0;
            for (; i < dummyPrefixBytes && i < len; ++i)
            {
                dst[i] = 0xAA;   // poison: obvious if it reaches a caller
            }
            for (std::size_t j = 0; i < len; ++i, ++j)
            {
                dst[i] = regs[byteIndex(reg, j)];
            }
            lastReadLen = len;
            return Status::Ok;
        }
        if (failNextRead > 0) { --failNextRead; return Status::IoError; }
        for (std::size_t i = 0; i < len; ++i)
        {
            dst[i] = regs[byteIndex(reg, i)];
        }
        lastReadLen = len;
        return Status::Ok;
    }

    Status writeRegs(std::uint8_t reg, const std::uint8_t* src, std::size_t len) override
    {
        ++writeCount;
        ++transactions;
        if (failNextWrite > 0) { --failNextWrite; return Status::IoError; }
        for (std::size_t i = 0; i < len; ++i)
        {
            regs[byteIndex(reg, i)] = src[i];
            writeLog.push_back({ static_cast<std::uint8_t>(reg + i), src[i] });
        }
        return Status::Ok;
    }

    [[nodiscard]] Mutex& busLock() override { return _lock; }
    [[nodiscard]] const char* busName() const override { return "fake"; }

    /// Store a big-endian 16-bit value, the way the MPU emits sensor data.
    void setBe16(std::uint8_t reg, std::int16_t v)
    {
        regs[reg]     = static_cast<std::uint8_t>((static_cast<std::uint16_t>(v) >> 8) & 0xFF);
        regs[reg + 1] = static_cast<std::uint8_t>(static_cast<std::uint16_t>(v) & 0xFF);
    }

    struct Write { std::uint8_t reg; std::uint8_t value; };

    /// Called before each read, with the starting register. Optional.
    std::function<void(std::uint8_t)> onRead;

    std::uint8_t       regs[512]{};
    std::vector<Write> writeLog;
    int  transactions  = 0;
    int  readCount     = 0;
    int  writeCount    = 0;
    int  failNextRead  = 0;
    int  failNextWrite = 0;
    std::size_t lastReadLen = 0;

    /// Filler bytes this device prepends to every read. BMI323 on I2C: 2.
    std::size_t dummyPrefixBytes = 0;

    /**
     * @brief Register addresses index 16-bit WORDS, not bytes.
     *
     * The BMI323 is word-addressed: ACC_CONF is register 0x20 and GYR_CONF is
     * 0x21, and they are adjacent REGISTERS, not adjacent bytes. Modelling them
     * as bytes makes a two-byte write to 0x20 spill into 0x21 — which is what
     * a byte-addressed fake did, silently corrupting the accelerometer
     * configuration when the gyroscope was configured.
     */
    bool wordAddressed = false;

private:
    /// Byte slot for offset `n` from register `reg`, honouring wordAddressed.
    [[nodiscard]] std::size_t byteIndex(std::uint8_t reg, std::size_t n) const
    {
        return wordAddressed ? ((static_cast<std::size_t>(reg) * 2 + n) & 0x1FF)
                             : static_cast<std::size_t>(static_cast<std::uint8_t>(reg + n));
    }

    HostRecursiveMutex _lock;
};

/**
 * @brief A task that never actually runs, but honours the stop contract.
 *
 * Enough to test what callers do around a task — request a stop, observe it,
 * see the body leave — without a thread. `runBody()` invokes the entry
 * synchronously, so a loop that checks stopRequested() can be driven from a
 * test in a defined order rather than raced against.
 */
class HostTask final : public Task
{
public:
    void requestStop() override { _stopRequested = true; }
    [[nodiscard]] bool stopRequested() const override { return _stopRequested; }
    [[nodiscard]] bool isRunning() const override { return _running; }

    /// Run the spawned body here and now, then mark the task finished exactly
    /// as the real trampoline does.
    void runBody()
    {
        if (_entry == nullptr) { return; }
        _running = true;
        _entry(_arg);
        _running = false;
    }

    void (*_entry)(void*) = nullptr;
    void*  _arg           = nullptr;
    bool   _running       = false;
    bool   _stopRequested = false;
};

/**
 * @brief Records sleeps instead of performing them, and hands out a HostTask.
 *
 * Two jobs. Tests stay fast — a driver power-up sequence with 400 ms of
 * settling delays runs instantly. And the delays become *assertable*: a
 * mandatory wait after a chip reset is invisible on a bench until the day it is
 * not, so sleepLog is how a test proves the driver waits at all.
 */
class RecordingScheduler final : public Scheduler
{
public:
    Result<Task*> spawn(const TaskConfig& cfg, void (*entry)(void*), void* arg) override
    {
        if (entry == nullptr) { return Status::InvalidArg; }
        if (spawnFails)       { return Status::NoSpace; }

        spawned.push_back(cfg.name);
        lastTask._entry   = entry;
        lastTask._arg     = arg;
        lastTask._running = true;
        return static_cast<Task*>(&lastTask);
    }

    void sleepFor(std::chrono::milliseconds d) override
    {
        sleepLog.push_back(d);
        totalSlept += d;
    }

    void sleepUntil(std::uint64_t&, std::chrono::milliseconds period) override
    {
        sleepLog.push_back(period);
        totalSlept += period;
    }

    void yield() override { ++yieldCount; }

    std::vector<std::chrono::milliseconds> sleepLog;
    std::chrono::milliseconds              totalSlept{ 0 };
    int                                    yieldCount = 0;

    /// One slot is enough: no test spawns two, and a vector would invalidate
    /// the pointer handed back by spawn() when it grew.
    HostTask                 lastTask;
    std::vector<const char*> spawned;
    bool                     spawnFails = false;
};

/// Watchdog that records rather than resets anything.
class NullWatchdog final : public Watchdog
{
public:
    Status registerCurrentTask() override   { ++registrations; return Status::Ok; }
    void   feed() noexcept override         { ++feeds; }
    Status unregisterCurrentTask() override { return Status::Ok; }

    int registrations = 0;
    int feeds = 0;
};

/**
 * @brief SettingsStore in RAM, with the same CRC the real one uses.
 *
 * The CRC is included deliberately rather than stubbed out: a fake that always
 * accepts what it stored would never exercise the corruption path, which is the
 * only reason the store carries a CRC at all.
 */
class MemorySettingsStore final : public device::SettingsStore
{
public:
    Status load(const char* key, void* dst, std::size_t len) override
    {
        const auto it = _blobs.find(key);
        if (it == _blobs.end()) { return Status::NotPresent; }
        if (it->second.size() != len + sizeof(std::uint32_t)) { return Status::Corrupt; }

        std::uint32_t stored = 0;
        std::memcpy(&stored, it->second.data() + len, sizeof(stored));
        if (stored != crc32(it->second.data(), len)) { return Status::Corrupt; }

        std::memcpy(dst, it->second.data(), len);
        return Status::Ok;
    }

    Status save(const char* key, const void* src, std::size_t len) override
    {
        std::vector<std::uint8_t> blob(len + sizeof(std::uint32_t));
        std::memcpy(blob.data(), src, len);
        const std::uint32_t crc = crc32(src, len);
        std::memcpy(blob.data() + len, &crc, sizeof(crc));
        _blobs[key] = std::move(blob);
        return Status::Ok;
    }

    Status erase(const char* key) override
    {
        return _blobs.erase(key) > 0 ? Status::Ok : Status::NotPresent;
    }

    /// Flip a bit, the way flash decay would.
    void corrupt(const char* key, std::size_t byteIndex)
    {
        auto it = _blobs.find(key);
        if (it != _blobs.end() && byteIndex < it->second.size())
        {
            it->second[byteIndex] ^= 0x01;
        }
    }

    [[nodiscard]] bool has(const char* key) const { return _blobs.count(key) > 0; }

private:
    std::map<std::string, std::vector<std::uint8_t>> _blobs;
};

/**
 * @brief KeyValueStore in RAM, matching Esp32KeyValueStore's contract exactly.
 *
 * The contract details matter more than the storage: a read whose buffer is too
 * small must REFUSE rather than truncate, and a length mismatch must be
 * detectable by the caller. Those are the behaviours ConfigPersistence relies
 * on to tell "absent" from "written by a build where this key had another type".
 */
class MemoryKeyValueStore final : public KeyValueStore
{
public:
    Status begin() override { ++beginCalls; return beginResult; }

    Status read(const char* key, void* dst, std::size_t capacity, std::size_t& outLen) override
    {
        outLen = 0;
        // Guarded to match Esp32KeyValueStore exactly. Without this,
        // _entries.find(nullptr) constructs a std::string from null — undefined
        // behaviour, and in practice a segfault. A fake that diverges from the
        // implementation it stands in for is worse than no fake at all.
        if (key == nullptr || dst == nullptr) { return Status::InvalidArg; }

        const auto it = _entries.find(key);
        if (it == _entries.end()) { return Status::NotPresent; }

        if (it->second.size() > capacity)
        {
            outLen = it->second.size();
            return Status::NoSpace;   // refuse, never truncate
        }
        std::memcpy(dst, it->second.data(), it->second.size());
        outLen = it->second.size();
        return Status::Ok;
    }

    Status write(const char* key, const void* src, std::size_t len) override
    {
        if (key == nullptr || src == nullptr || len == 0) { return Status::InvalidArg; }
        if (writeResult != Status::Ok) { return writeResult; }
        const auto* bytes = static_cast<const std::uint8_t*>(src);
        _entries[key] = std::vector<std::uint8_t>(bytes, bytes + len);
        return Status::Ok;
    }

    Status erase(const char* key) override
    {
        if (key == nullptr) { return Status::InvalidArg; }
        return _entries.erase(key) > 0 ? Status::Ok : Status::NotPresent;
    }

    Status eraseAll() override { _entries.clear(); return Status::Ok; }
    Status commit() override { ++commitCalls; return Status::Ok; }

    [[nodiscard]] bool has(const char* key) const { return _entries.count(key) > 0; }
    [[nodiscard]] std::size_t size() const { return _entries.size(); }

    /// Write a value of the wrong width, as a build with a different type for
    /// this key would have left behind.
    void writeRaw(const char* key, std::vector<std::uint8_t> bytes)
    {
        _entries[key] = std::move(bytes);
    }

    int    beginCalls  = 0;
    int    commitCalls = 0;
    Status beginResult = Status::Ok;
    Status writeResult = Status::Ok;

private:
    std::map<std::string, std::vector<std::uint8_t>> _entries;
};

/**
 * @brief device::LogStore in RAM, with a settable capacity.
 *
 * The capacity is the point. On hardware the full-disk and purge paths need a
 * 1.9 MB partition filled to reach; here `setCapacity(200)` reaches them in a
 * line. Those are the paths that fail on a long flying day and fail at
 * startLogging(), so the flight is simply not recorded.
 */
class MemoryLogStore final : public device::LogStore
{
public:
    Status begin() override { _mounted = true; return Status::Ok; }

    std::size_t listSessions(std::uint16_t* out, std::size_t maxEntries) override
    {
        std::size_t n = 0;
        for (const auto& [index, data] : _sessions)
        {
            if (n >= maxEntries) { break; }
            out[n++] = index;
        }
        return n;
    }

    Status openSession(std::uint16_t index) override
    {
        if (!_mounted)   { return Status::NotPresent; }
        if (_openIndex)  { return Status::Busy; }
        _sessions[index].clear();          // create or truncate
        _openIndex = index;
        return Status::Ok;
    }

    Status append(const char* data, std::size_t len) override
    {
        if (!_openIndex)                 { return Status::NotPresent; }
        if (data == nullptr || len == 0) { return Status::InvalidArg; }

        std::uint32_t used = 0, total = 0;
        (void)usage(used, total);
        if (total > 0 && used + len > total) { return Status::NoSpace; }

        auto& blob = _sessions[*_openIndex];
        blob.insert(blob.end(), data, data + len);
        return Status::Ok;
    }

    Status flush() override { return _openIndex ? Status::Ok : Status::NotPresent; }

    Status closeSession() override { _openIndex.reset(); return Status::Ok; }
    [[nodiscard]] bool isOpen() const override { return _openIndex.has_value(); }

    Status readSession(std::uint16_t index, void* dst, std::size_t maxLen,
                       std::size_t offset, std::size_t& outLen) override
    {
        outLen = 0;
        const auto it = _sessions.find(index);
        if (it == _sessions.end()) { return Status::NotPresent; }

        if (offset >= it->second.size()) { return Status::Ok; }   // past the end

        outLen = std::min(maxLen, it->second.size() - offset);
        std::memcpy(dst, it->second.data() + offset, outLen);
        return Status::Ok;
    }

    Status sessionSize(std::uint16_t index, std::uint32_t& bytes) override
    {
        bytes = 0;
        const auto it = _sessions.find(index);
        if (it == _sessions.end()) { return Status::NotPresent; }
        bytes = static_cast<std::uint32_t>(it->second.size());
        return Status::Ok;
    }

    Status removeSession(std::uint16_t index) override
    {
        if (_openIndex && *_openIndex == index) { return Status::Busy; }
        return _sessions.erase(index) > 0 ? Status::Ok : Status::NotPresent;
    }

    Status usage(std::uint32_t& used, std::uint32_t& total) override
    {
        std::size_t sum = 0;
        for (const auto& [index, data] : _sessions) { sum += data.size(); }
        used  = static_cast<std::uint32_t>(sum);
        total = _capacity;
        return Status::Ok;
    }

    Status formatAll() override
    {
        _sessions.clear();
        _openIndex.reset();
        return Status::Ok;
    }

    /// 0 means unbounded.
    void setCapacity(std::uint32_t bytes) { _capacity = bytes; }

    /// Pre-load a session of a given size, to set up a purge scenario.
    void seedSession(std::uint16_t index, std::size_t bytes)
    {
        _sessions[index] = std::vector<char>(bytes, 'x');
    }

    [[nodiscard]] std::size_t sessionCount() const { return _sessions.size(); }
    [[nodiscard]] bool has(std::uint16_t index) const { return _sessions.count(index) > 0; }

private:
    std::map<std::uint16_t, std::vector<char>> _sessions;
    std::optional<std::uint16_t>               _openIndex;
    std::uint32_t                              _capacity = 0;
    bool                                       _mounted  = false;
};

} // namespace arduflite::hal::host

#endif // ARDUFLITE_TESTS_HAL_HOST_PLATFORM_H

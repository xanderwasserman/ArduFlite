/**
 * SeqLock.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Versioned lock-free snapshot. Generalised from the working seqlock in
 *        ArduFliteIMU, including its stale-but-coherent fallback.
 *
 * Single writer, any number of readers. The writer marks the version odd while
 * writing and even when done; readers retry if the version moved mid-copy.
 *
 * The trivially-copyable constraint is a CORRECTNESS requirement, not a style
 * choice: a seqlock races on its payload by construction. Adding a std::string to
 * a seqlocked struct now fails to compile instead of corrupting in flight.
 */
#ifndef ARDUFLITE_HAL_CORE_SEQLOCK_H
#define ARDUFLITE_HAL_CORE_SEQLOCK_H

#include <atomic>
#include <cstdint>
#include <type_traits>

#include "src/hal/core/NonCopyable.h"

namespace arduflite {

template <typename T, std::uint32_t kRetryLimit = 8>
    requires std::is_trivially_copyable_v<T>
class SeqLock : private NonCopyable
{
public:
    struct Health
    {
        std::uint32_t totalRetries   = 0;
        std::uint32_t maxRetries     = 0;
        std::uint32_t retryLimitHits = 0;
    };

    /// Writer. Single thread only.
    ///
    /// @note The standalone fences are load-bearing, not decoration. A plain
    ///       store(release) on the counter does NOT stop the following data write
    ///       from being hoisted above it — release only orders *prior* accesses.
    ///       Without the fence the payload can be written before the counter goes
    ///       odd, and a concurrent reader tears. See specs/hal/08-review.md R18.
    void publish(const T& value) noexcept
    {
        // Keep the previous complete value as a coherent fallback first.
        const std::uint32_t lastVersion = _lastCompleteVersion.load(std::memory_order_relaxed);
        _lastCompleteVersion.store(lastVersion + 1, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
        _lastComplete = _current;
        std::atomic_thread_fence(std::memory_order_release);
        _lastCompleteVersion.store(lastVersion + 2, std::memory_order_relaxed);

        const std::uint32_t version = _version.load(std::memory_order_relaxed);
        _version.store(version + 1, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
        _current = value;
        std::atomic_thread_fence(std::memory_order_release);
        _version.store(version + 2, std::memory_order_relaxed);
    }

    /// Reader. Returns a coherent copy — the current value if one can be read
    /// within kRetryLimit attempts, otherwise the one-cycle-old fallback.
    [[nodiscard]] T read() const noexcept
    {
        T out{};
        std::uint32_t retries = 0;

        for (;;)
        {
            const std::uint32_t before = _version.load(std::memory_order_relaxed);
            if ((before & 1U) == 0U)
            {
                // Fence, not load(acquire): acquire orders only SUBSEQUENT
                // accesses, so the payload copy could otherwise sink past the
                // second counter load and escape the check.
                std::atomic_thread_fence(std::memory_order_acquire);
                out = _current;
                std::atomic_thread_fence(std::memory_order_acquire);

                const std::uint32_t after = _version.load(std::memory_order_relaxed);
                if (before == after)
                {
                    recordRetries(retries, false);
                    return out;
                }
            }

            ++retries;
            if (retries >= kRetryLimit) [[unlikely]]
            {
                recordRetries(retries, true);
                return readLastComplete();
            }
        }
    }

    /// Reader for callers that already own storage. Returns false if the retry
    /// limit was hit, in which case `out` holds the stale coherent fallback.
    [[nodiscard]] bool tryRead(T& out) const noexcept
    {
        std::uint32_t retries = 0;

        for (;;)
        {
            const std::uint32_t before = _version.load(std::memory_order_relaxed);
            if ((before & 1U) == 0U)
            {
                std::atomic_thread_fence(std::memory_order_acquire);
                out = _current;
                std::atomic_thread_fence(std::memory_order_acquire);

                const std::uint32_t after = _version.load(std::memory_order_relaxed);
                if (before == after)
                {
                    recordRetries(retries, false);
                    return true;
                }
            }

            ++retries;
            if (retries >= kRetryLimit) [[unlikely]]
            {
                recordRetries(retries, true);
                out = readLastComplete();
                return false;
            }
        }
    }

    [[nodiscard]] Health health() const noexcept
    {
        return { _totalRetries.load(std::memory_order_relaxed),
                 _maxRetries.load(std::memory_order_relaxed),
                 _limitHits.load(std::memory_order_relaxed) };
    }

private:
    [[nodiscard]] T readLastComplete() const noexcept
    {
        T out{};
        for (;;)
        {
            const std::uint32_t before = _lastCompleteVersion.load(std::memory_order_relaxed);
            if ((before & 1U) != 0U) { continue; }

            std::atomic_thread_fence(std::memory_order_acquire);
            out = _lastComplete;
            std::atomic_thread_fence(std::memory_order_acquire);

            const std::uint32_t after = _lastCompleteVersion.load(std::memory_order_relaxed);
            if (before == after) { return out; }
        }
    }

    void recordRetries(std::uint32_t retries, bool limitHit) const noexcept
    {
        if (retries > 0)
        {
            _totalRetries.fetch_add(retries, std::memory_order_relaxed);

            std::uint32_t observed = _maxRetries.load(std::memory_order_relaxed);
            while (retries > observed &&
                   !_maxRetries.compare_exchange_weak(observed, retries,
                                                      std::memory_order_relaxed,
                                                      std::memory_order_relaxed))
            {
            }
        }

        if (limitHit) { _limitHits.fetch_add(1, std::memory_order_relaxed); }
    }

    T _current{};
    T _lastComplete{};

    std::atomic<std::uint32_t> _version{ 0 };
    std::atomic<std::uint32_t> _lastCompleteVersion{ 0 };

    mutable std::atomic<std::uint32_t> _totalRetries{ 0 };
    mutable std::atomic<std::uint32_t> _maxRetries{ 0 };
    mutable std::atomic<std::uint32_t> _limitHits{ 0 };
};

} // namespace arduflite

#endif // ARDUFLITE_HAL_CORE_SEQLOCK_H

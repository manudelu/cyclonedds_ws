#pragma once

#include <cstdint>
#include <ctime>
#include <iostream>

/**
 * The EtherCAT master stamps every PDO with CLOCK_MONOTONIC (nanoseconds since system boot) 
 * ROS2 and most logging tools expect CLOCK_REALTIME (nanoseconds since Unix epoch, Jan 1 1970)
 *
 * Both clocks tick at the same hardware rate, so their difference K is a fixed constant:
 *
 *   Realtime(t) = Monotonic(t) + K, where K = (Realtime_at_boot(t) - Monotonic_at_boot(t))
 *
 * We measure K once at startup by sampling both clocks back-to-back, then convert any PDO timestamp with a single addition.
 */
namespace clock_utils {

struct ClockOffset {
    uint64_t realtime_ns  = 0; 
    uint64_t monotonic_ns = 0;
    uint64_t K = 0; 
};
inline ClockOffset g_offset;

inline void init()
{
    struct timespec rt{};
    clock_gettime(CLOCK_REALTIME,  &rt);

    struct timespec mono{};
    clock_gettime(CLOCK_MONOTONIC, &mono);

    g_offset.realtime_ns = static_cast<uint64_t>(rt.tv_sec) * 1'000'000'000ULL + static_cast<uint64_t>(rt.tv_nsec);
    g_offset.monotonic_ns = static_cast<uint64_t>(mono.tv_sec) * 1'000'000'000ULL + static_cast<uint64_t>(mono.tv_nsec);
    g_offset.K = g_offset.realtime_ns - g_offset.monotonic_ns;
}

inline uint64_t monotonic_to_realtime(uint64_t mono_ns)
{
    return mono_ns + g_offset.K;
}

// Fallback function (used when PDO has no timestamp)
inline uint64_t monotonic_now_ns()
{
    struct timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL + static_cast<uint64_t>(ts.tv_nsec);
}

} 
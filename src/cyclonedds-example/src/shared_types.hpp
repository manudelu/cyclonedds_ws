#pragma once
#include <atomic>
#include <array>
#include <cstdint>
#include <cstddef>

// JointState struct - no dynamic allocation
struct JointState {
    int32_t sec;
    uint32_t nanosec;
    double position[12];
    double velocity[12];
    double effort[12];
};

template<typename T, size_t N>
struct SPSCQueue {
    static_assert((N & (N - 1)) == 0);
    static constexpr size_t MASK = N - 1;

    // alignas(64): puts head and tail on two separate cache lines
    alignas(64) std::atomic<size_t> head{0};  // Producer only
    alignas(64) std::atomic<size_t> tail{0};  // Consumer only
    std::array<T, N> buf{};

    // Producer side (DDS process)
    bool try_push(const T& val) {
        // Read head index
        size_t h = head.load(std::memory_order_relaxed);   
        // Calculate the next head index
        size_t next = (h + 1) & MASK;

        if (next == tail.load(std::memory_order_acquire)) 
            return false;

        // Write JointState data on buffer
        buf[h] = val;
        // Update head index
        head.store(next, std::memory_order_release);

        return true;
    }

    // Consumer side (RT process)
    bool try_pop(T& val) {
        // Read tail index
        size_t t = tail.load(std::memory_order_relaxed);

        if (t == head.load(std::memory_order_acquire)) 
            return false;
        
        // Read JointState data from buffer 
        val = buf[t];

        // Update tail index
        tail.store((t + 1) & MASK, std::memory_order_release);
        
        return true;
    }
};

struct SharedBridge {
    SPSCQueue<JointState, 64> dds_to_client;   
    SPSCQueue<JointState, 64> client_to_server;
    SPSCQueue<JointState, 64> server_to_dds;  
    std::atomic<bool> dds_ready{false};
    std::atomic<bool> rt_client_ready{false};
    std::atomic<bool> rt_server_ready{false};
};

static constexpr const char* SHM_NAME = "/spot_rt_bridge";
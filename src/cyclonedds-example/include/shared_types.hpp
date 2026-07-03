#pragma once
#include <atomic>
#include <array>
#include <cstdint>
#include <cstddef>
#include <cstring>

static constexpr const char* SHM_NAME = "/rt_bridge";

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

static constexpr size_t PROTO_MAX_BYTES = 512;
struct ProtoSlot {
    uint32_t size{0};
    uint8_t  data[PROTO_MAX_BYTES]{};
};

struct SharedBridge {
    SPSCQueue<ProtoSlot, 64> cmd;   
    SPSCQueue<ProtoSlot, 64> joint_state;  
    SPSCQueue<ProtoSlot, 64> imu;

    alignas(64) std::atomic<bool> mw_ready{false};
    alignas(64) std::atomic<bool> rt_ready{false};
};

// Owns the intermediate serialization buffers 
// and provides drain/push logic shared by both processes.
struct ShmProtoHelper {
    ProtoSlot slot {};
    uint8_t ser_buf[PROTO_MAX_BYTES] {};

    // Drain the queue and process every message (NRT usage)
    template<size_t N, typename Proto, typename Fn>
    void drain(SPSCQueue<ProtoSlot, N>& queue, Proto& msg, Fn&& on_msg) {
        while (queue.try_pop(slot)) {
            if (slot.size == 0 || slot.size > PROTO_MAX_BYTES) 
                continue;
            if (msg.ParseFromArray(slot.data, static_cast<int>(slot.size)))
                on_msg(msg);
        }
    }

    // Drain the queue but keep only latest (for RT usage)
    template<size_t N, typename Proto>
    void parse_latest(SPSCQueue<ProtoSlot, N>& queue, Proto& msg) {
        while (queue.try_pop(slot)) {
            if (slot.size > 0 && slot.size <= PROTO_MAX_BYTES)
                msg.ParseFromArray(slot.data, static_cast<int>(slot.size));
        }
    }

    template<size_t N, typename Proto>
    void push(SPSCQueue<ProtoSlot, N>& queue, const Proto& msg) {
        // Ask Protobuf how many bytes the message will occupy once serialized
        int bytes = static_cast<int>(msg.ByteSizeLong());
        if (bytes <= 0 || bytes > static_cast<int>(PROTO_MAX_BYTES)) 
            return;
        // Serialize the message in ser_buf
        msg.SerializeToArray(ser_buf, bytes);
        slot.size = static_cast<uint32_t>(bytes);
        std::memcpy(slot.data, ser_buf, bytes);
        queue.try_push(slot);
    }
};
#include "shared_types.hpp"
#include "shm_utils.hpp"
#include "motor.pb.h"
#include "imu.pb.h"

#include <signal.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <cstring>
#include <iostream>

static std::atomic<bool> g_stop{false};
void handle_sig(int) { g_stop = true; }

int main() {
    signal(SIGINT,  handle_sig);
    signal(SIGTERM, handle_sig);

    // Lock all current and future memory pages — mandatory for hard RT
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        perror("[RT] mlockall failed!");
        return 1;
    }

    SharedMemoryOwner shm(SHM_NAME, sizeof(SharedBridge));
    SharedBridge* bridge = new(shm.raw_ptr()) SharedBridge();

    // Wait for DDS process to be ready
    std::cout << "[RT] Waiting for DDS process...\n";
    while (!bridge->dds_ready.load(std::memory_order_acquire) && !g_stop)
        usleep(1000);

    bridge->rt_ready.store(true, std::memory_order_release);
    std::cout << "[RT] Bridge active. Promoting process to Hard Real-Time. Running control loop.\n";

    // Become RT
    struct sched_param param;
    param.sched_priority = 80;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    pthread_setname_np(pthread_self(), "RT_Control");

    // Pre-allocate all working state before entering the loop

    // Outbound
    iit::advrf::MotorState out_state;
    out_state.mutable_motors()->Reserve(12);
    for (int i = 0; i < 12; ++i)
        out_state.add_motors();

    iit::advrf::ImuState out_imu;  
    ProtoSlot imu_slot{};

    // Inbound
    iit::advrf::MotorCmd in_cmd;

    uint8_t ser_buf[PROTO_MAX_BYTES];
    ProtoSlot slot{};

    double pos = 0.0;
    bool incremental = true;

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    const long long period_ns = 1'000'000LL;

    while (!g_stop.load(std::memory_order_relaxed)) {

        // Drain inbound — keep freshest
        ProtoSlot in_slot{};
        while (bridge->cmd.try_pop(in_slot)) {
            if (in_slot.size > 0 && in_slot.size <= PROTO_MAX_BYTES)
                in_cmd.ParseFromArray(in_slot.data, static_cast<int>(in_slot.size));
        }

        // Control logic
        if (incremental) { 
            pos += 0.01; 
            if (pos > 0.3) 
                incremental = false; 
        }
        else { 
            pos -= 0.01; 
            if (pos < 0.0) 
                incremental = true;  
        }

        // Joint State
        struct timespec now;
        clock_gettime(CLOCK_REALTIME, &now);
        out_state.set_sec(static_cast<int32_t>(now.tv_sec));
        out_state.set_nanosec(static_cast<uint32_t>(now.tv_nsec));

        for (int i = 0; i < 12; ++i) {
            out_state.mutable_motors(i)->set_link_pos(static_cast<float>(pos));
            out_state.mutable_motors(i)->set_link_vel(0.0f);
            out_state.mutable_motors(i)->set_torque(0.0f);
        }

        const int bytes = static_cast<int>(out_state.ByteSizeLong());
        if (bytes > 0 && bytes <= static_cast<int>(PROTO_MAX_BYTES)) {
            out_state.SerializeToArray(ser_buf, bytes);
            slot.size = static_cast<uint32_t>(bytes);
            std::memcpy(slot.data, ser_buf, bytes);
            bridge->joint_state.try_push(slot);
        }

        // IMU
        out_imu.set_sec(static_cast<int32_t>(now.tv_sec));
        out_imu.set_nanosec(static_cast<uint32_t>(now.tv_nsec));
        out_imu.set_orient_w(1.0f);  
        out_imu.set_orient_x(0.0f);
        out_imu.set_orient_y(0.0f);
        out_imu.set_orient_z(0.0f);

        const int imu_bytes = static_cast<int>(out_imu.ByteSizeLong());
        if (imu_bytes > 0 && imu_bytes <= static_cast<int>(PROTO_MAX_BYTES)) {
            out_imu.SerializeToArray(ser_buf, imu_bytes);
            imu_slot.size = static_cast<uint32_t>(imu_bytes);
            std::memcpy(imu_slot.data, ser_buf, imu_bytes);
            bridge->imu.try_push(imu_slot);
        }

        next.tv_nsec += period_ns;
        if (next.tv_nsec >= 1'000'000'000L) {
            next.tv_sec  += 1;
            next.tv_nsec -= 1'000'000'000L;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    }
    bridge->~SharedBridge();
    std::cout << "[RT] Shutting down.\n";
    
    return 0;
}
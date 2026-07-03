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
    if (!shm.is_valid()) {
        std::cerr << "[RT] Failed to create Shared Memory.\n";
        return 1;
    }
    SharedBridge* bridge = new(shm.raw_ptr()) SharedBridge();

    // Handshake: Wait for process to be ready 
    // Note: here we will have mode switches since we are still non real time
    std::cout << "[RT] Waiting for process...\n";
    while (!bridge->mw_ready.load(std::memory_order_acquire) && !g_stop)
        usleep(1000);

    if (g_stop) {
        bridge->~SharedBridge();
        return 0;
    }

    // Temporal offset 
    // Used for ROS2 compliance, since in hard real-time it's better to use CLOCK_MONOTONIC rather than CLOCK_REALTIME
    struct timespec ts_mono, ts_real;
    clock_gettime(CLOCK_MONOTONIC, &ts_mono);
    clock_gettime(CLOCK_REALTIME,  &ts_real);
    long long mono_ns = (long long)ts_mono.tv_sec * 1'000'000'000LL + ts_mono.tv_nsec;
    long long real_ns = (long long)ts_real.tv_sec * 1'000'000'000LL + ts_real.tv_nsec;
    const long long ros2_time_offset_ns = real_ns - mono_ns;

    bridge->rt_ready.store(true, std::memory_order_release);
    std::cout << "[RT] Bridge active. Promoting process to Hard Real-Time.\n";

    // Hard Real-Time Promotion !!
    struct sched_param param;
    param.sched_priority = 0;//80;
    if (pthread_setschedparam(pthread_self(), SCHED_OTHER, &param) != 0) {
        perror("[RT] Failed to set thread priority.");
        bridge->~SharedBridge();
        return 1;
    }
    pthread_setname_np(pthread_self(), "RT_Control");

    // Outbound (data sent to DDS)
    iit::advrf::MotorState joint_state;
    joint_state.mutable_motors()->Reserve(12);
    for (int i = 0; i < 12; ++i)
        joint_state.add_motors();

    iit::advrf::ImuState imu;  

    // Inbound (data received from DDS)
    iit::advrf::MotorCmd joint_trajectory;

    ShmProtoHelper proto;

    // "Control" variables
    double pos = 0.0;
    bool incremental = true;

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (!g_stop.load(std::memory_order_relaxed)) {

        // Drain inbound — keep freshest (read commands from ROS2)
        proto.parse_latest(bridge->cmd, joint_trajectory);

        // Joint State ""Control Logic""
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

        // Calculate ROS2 compatible timestamps
        long long current_loop_mono_ns = (long long)next.tv_sec * 1'000'000'000LL + next.tv_nsec;
        long long ros2_compliant_ns = current_loop_mono_ns + ros2_time_offset_ns;

        int32_t  ros2_sec  = static_cast<int32_t>(ros2_compliant_ns / 1'000'000'000LL);
        uint32_t ros2_nsec = static_cast<uint32_t>(ros2_compliant_ns % 1'000'000'000LL);

        // Ex: Write Joint State to DDS
        joint_state.set_sec(ros2_sec);
        joint_state.set_nanosec(ros2_nsec);
        for (int i = 0; i < 12; ++i) {
            auto* motor = joint_state.mutable_motors(i);
            motor->set_link_pos(static_cast<float>(pos));
            motor->set_link_vel(0.0f);
            motor->set_torque(0.0f);
        }
        proto.push(bridge->joint_state, joint_state);

        // Ex: Write Imu to DDS
        imu.set_sec(ros2_sec);
        imu.set_nanosec(ros2_nsec);
        imu.set_orient_w(1.0f);  
        imu.set_orient_x(0.0f);
        imu.set_orient_y(0.0f);
        imu.set_orient_z(0.0f);
        proto.push(bridge->imu, imu);

        next.tv_nsec += 1'000'000LL;
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
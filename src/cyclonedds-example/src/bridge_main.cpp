#include "shared_types.hpp"
#include "shm_utils.hpp"
// #include "zmq_adapter.hpp"
#include "dds_adapter.hpp"
#include "motor.pb.h"
#include "imu.pb.h"

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <iostream>

#include <dds/dds.hpp>
#include "JointState.hpp"
#include "Imu.hpp"

static const std::string ROBOT_NAME {"kyon"};
static std::atomic<bool> g_stop{false};
void handle_sig(int) { g_stop = true; }

int main() {
    signal(SIGINT,  handle_sig);
    signal(SIGTERM, handle_sig);

    // Middleware Selection
    std::unique_ptr<MiddlewareAdapter> adapter = std::make_unique<DdsAdapter>();
    //std::unique_ptr<MiddlewareAdapter> adapter = std::make_unique<ZmqAdapter>();
    if (!adapter->init(ROBOT_NAME)) {
        std::cerr << "[Bridge] Adapter init failed.\n";
        return 1;
    }

    // Shared Memory
    SharedMemoryClient shm(SHM_NAME, sizeof(SharedBridge));
    if (!shm.is_valid()) {
        std::cerr << "[Bridge] Failed to open Shared Memory.\n";
        return 1;
    }
    SharedBridge* bridge = shm.get<SharedBridge>();

    // Inbound (data received from RT process and then published)
    iit::advrf::MotorState joint_state;
    iit::advrf::ImuState imu;

    // Outbound (data sent to RT process received from subscribing)
    iit::advrf::MotorCmd joint_trajectory;
    joint_trajectory.mutable_motors()->Reserve(12);
    for (int i = 0; i < 12; ++i)
        joint_trajectory.add_motors();
    
    // Startup handshake - DDS signals readiness only after its setup
    bridge->mw_ready.store(true, std::memory_order_release);
    std::cout << "[Bridge] Bridge process started...\n";

    const struct timespec dt{0, 500000}; // poll at 2 kHz

    ShmProtoHelper proto;
    while (!g_stop) {
        // Drain through SHM from RT and Publish (from Hardware to ROS2)
        proto.drain(bridge->joint_state, 
                    joint_state, 
                    [&](auto& m){ adapter->publish_joint_state(m); }
        );
        proto.drain(bridge->imu, 
                    imu, 
                    [&](auto& m){ adapter->publish_imu(m); }
        );

        // Subscribe and send to RT through SHM (from ROS2 to Hardware)
        //while (adapter->take_joint_trajectory(jt_msg)) {
        //    if (jt_msg.points().empty()) 
        //        continue;
        //    const auto& pt = jt_msg.points()[0];
        //    for (int i = 0; i < 12; ++i) {
        //        auto* m = joint_trajectory.mutable_motors(i);
        //       m->set_pos_ref(static_cast<float>(pt.positions()[i]));
        //        m->set_vel_ref(static_cast<float>(pt.velocities()[i]));
        //        m->set_torque_ffwd(static_cast<float>(pt.effort()[i]));
        //    }
        //    proto.push(bridge->cmd, joint_trajectory);
        //}

        nanosleep(&dt, nullptr);
    }
    std::cout << "[Bridge] Shutting down.\n";

    return 0;
}
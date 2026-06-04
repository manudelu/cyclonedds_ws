#include "shared_types.hpp"
#include "shm_utils.hpp"
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

int32_t DOMAIN_ID {42};

static std::atomic<bool> g_stop{false};
void handle_sig(int) { g_stop = true; }

int main() {
    signal(SIGINT,  handle_sig);
    signal(SIGTERM, handle_sig);

    SharedMemoryClient shm(SHM_NAME, sizeof(SharedBridge));
    SharedBridge* bridge = shm.get<SharedBridge>();

    // Set up DDS publishers
    dds::domain::DomainParticipant dp(DOMAIN_ID);
    dds::pub::Publisher pub(dp);
    auto writer_qos = dds::pub::qos::DataWriterQos()
        << dds::core::policy::Reliability::BestEffort()
        << dds::core::policy::History::KeepLast(1);

    // JointState
    dds::topic::Topic<::sensor_msgs::msg::dds_::JointState_> js_topic(dp, "rt/advrf/spot/joint_states");
    dds::pub::DataWriter<::sensor_msgs::msg::dds_::JointState_> js_writer(pub, js_topic, writer_qos);

    ::sensor_msgs::msg::dds_::JointState_ js_msg;
    js_msg.name() = {
        "front_left_hip_x",  "front_left_hip_y",  "front_left_knee",
        "front_right_hip_x", "front_right_hip_y", "front_right_knee",
        "rear_left_hip_x",   "rear_left_hip_y",   "rear_left_knee",
        "rear_right_hip_x",  "rear_right_hip_y",  "rear_right_knee"
    };
    js_msg.position().resize(12, 0.0);
    js_msg.velocity().assign(12, 0.0);
    js_msg.effort().assign(12, 0.0);

    // Imu
    dds::topic::Topic<::sensor_msgs::msg::dds_::Imu_> imu_topic(dp, "rt/advrf/spot/imu");
    dds::pub::DataWriter<::sensor_msgs::msg::dds_::Imu_> imu_writer(pub, imu_topic, writer_qos);

    ::sensor_msgs::msg::dds_::Imu_ imu_msg;
    imu_msg.header().frame_id("imu_link");

    // Inbound (data received)
    iit::advrf::MotorState in_state;
    iit::advrf::ImuState in_imu;

    // Outbound (data sent)
    iit::advrf::MotorCmd out_cmd;
    out_cmd.mutable_motors()->Reserve(12);
    for (int i = 0; i < 12; ++i)
        out_cmd.add_motors();
    
    uint8_t ser_buf[PROTO_MAX_BYTES];
    ProtoSlot slot{};

    // Startup handshake - DDS signals readiness only after its setup
    bridge->dds_ready.store(true, std::memory_order_release);
    std::cout << "[DDS] Ready. Waiting for RT process...\n";
    while (!bridge->rt_ready.load(std::memory_order_acquire) && !g_stop)
        usleep(1000);

    std::cout << "[DDS] Bridge active.\n";

    const struct timespec dt{0, 500000}; // poll at 2 kHz

    while (!g_stop) {
        // Retrieve data from RT SHM and Publish JointState
        while (bridge->joint_state.try_pop(slot)) {
            if (slot.size == 0 || slot.size > PROTO_MAX_BYTES) 
                continue;

            if (!in_state.ParseFromArray(slot.data, static_cast<int>(slot.size))) 
                continue;  

            js_msg.header().stamp().sec(in_state.sec());
            js_msg.header().stamp().nanosec(in_state.nanosec());
            for (int i = 0; i < 12; ++i) {
                const auto& m = in_state.motors(i);
                js_msg.position()[i] = static_cast<double>(m.link_pos());
                js_msg.velocity()[i] = static_cast<double>(m.link_vel());
                js_msg.effort()[i]   = static_cast<double>(m.torque());
            }
            js_writer.write(js_msg);
        }

        // Retrieve data from RT SHM and Publish Imu
        while (bridge->imu.try_pop(slot)) {
            if (slot.size == 0 || slot.size > PROTO_MAX_BYTES) 
                
            continue;
            if (!in_imu.ParseFromArray(slot.data, static_cast<int>(slot.size))) 
                continue;

            imu_msg.header().stamp().sec(in_imu.sec());
            imu_msg.header().stamp().nanosec(in_imu.nanosec());
            imu_msg.orientation().x(static_cast<double>(in_imu.orient_x()));
            imu_msg.orientation().y(static_cast<double>(in_imu.orient_y()));
            imu_msg.orientation().z(static_cast<double>(in_imu.orient_z()));
            imu_msg.orientation().w(static_cast<double>(in_imu.orient_w()));
            imu_msg.angular_velocity().x(static_cast<double>(in_imu.ang_vel_x()));
            imu_msg.angular_velocity().y(static_cast<double>(in_imu.ang_vel_y()));
            imu_msg.angular_velocity().z(static_cast<double>(in_imu.ang_vel_z()));
            imu_msg.linear_acceleration().x(static_cast<double>(in_imu.lin_acc_x()));
            imu_msg.linear_acceleration().y(static_cast<double>(in_imu.lin_acc_y()));
            imu_msg.linear_acceleration().z(static_cast<double>(in_imu.lin_acc_z()));
            imu_writer.write(imu_msg);
        }

        nanosleep(&dt, nullptr);
    }
    std::cout << "[DDS] Shutting down.\n";

    return 0;
}
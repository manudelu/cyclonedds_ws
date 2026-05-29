#include "shared_types.hpp"
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <iostream>
#include <dds/dds.hpp>
#include "JointState.hpp"

int32_t DOMAIN_ID {42};

static std::atomic<bool> g_stop{false};
void handle_sig(int) { 
    g_stop = true; 
}

int main() {
    signal(SIGINT,  handle_sig);
    signal(SIGTERM, handle_sig);

    // Creates shared memory object in /dev/shm called /spot_rt_bridge
    int fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (fd < 0) { 
        perror("shm_open"); 
        return 1; 
    }

    // Set its size to the size of our structure
    if (ftruncate(fd, sizeof(SharedBridge)) == -1) {
        perror("ftruncate");
        return 1;
    }

    // Map the object into the caller's address space
    void* ptr = mmap(nullptr, sizeof(SharedBridge),
                     PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (ptr == MAP_FAILED) { 
        perror("mmap"); 
        return 1; 
    }

    // Placement-new to initialise atomics correctly in shared memory
    SharedBridge* bridge = new(ptr) SharedBridge();

    // Set up DDS publisher and subscriber
    dds::domain::DomainParticipant dp(DOMAIN_ID);
    
    dds::topic::Topic<::sensor_msgs::msg::dds_::JointState_> topic_sub(dp, "rt/advrf/spot/joint_cmd");
    dds::sub::Subscriber sub(dp);
    dds::sub::qos::DataReaderQos rqos = dds::sub::qos::DataReaderQos()
        << dds::core::policy::Reliability::BestEffort()
        << dds::core::policy::History::KeepLast(1);
    dds::sub::DataReader<::sensor_msgs::msg::dds_::JointState_> reader(sub, topic_sub, rqos);

    dds::topic::Topic<::sensor_msgs::msg::dds_::JointState_> topic_pub(dp, "rt/advrf/spot/joint_state");
    dds::pub::Publisher pub(dp);
    dds::pub::qos::DataWriterQos wqos = dds::pub::qos::DataWriterQos()
        << dds::core::policy::Reliability::BestEffort()
        << dds::core::policy::History::KeepLast(1);
    dds::pub::DataWriter<::sensor_msgs::msg::dds_::JointState_> writer(pub, topic_pub, wqos);

    ::sensor_msgs::msg::dds_::JointState_ out_msg;
    out_msg.name() = {
        "front_left_hip_x",  "front_left_hip_y",  "front_left_knee",
        "front_right_hip_x", "front_right_hip_y", "front_right_knee",
        "rear_left_hip_x",   "rear_left_hip_y",   "rear_left_knee",
        "rear_right_hip_x",  "rear_right_hip_y",  "rear_right_knee"
    };
    out_msg.position().resize(12, 0.0);
    out_msg.velocity().assign(12, 0.0);
    out_msg.effort().assign(12, 0.0);

    // Startup handshake - DDS signals readiness only after its setup
    bridge->dds_ready.store(true, std::memory_order_release);
    std::cout << "[DDS] Ready. Waiting for BOTH RT processes to connect...\n";

    while ((!bridge->rt_client_ready.load(std::memory_order_acquire) || 
            !bridge->rt_server_ready.load(std::memory_order_acquire)) && !g_stop)
        usleep(1000);

    std::cout << "[DDS] All systems active. Bridge spinning at 2 kHz.\n";
    const struct timespec dt{0, 500000};

    while (!g_stop) {
        // Subscribe from ROS2 ----> Push data to RT domain
        auto samples = reader.take();
        for (auto const& s : samples) {
            if (!s.info().valid()) 
                continue;

            const auto& msg = s.data();
            JointState cmd{};
            cmd.sec = msg.header().stamp().sec();
            cmd.nanosec = msg.header().stamp().nanosec();
            for (size_t i = 0; i < 12; ++i) {
                cmd.position[i] = (msg.position().size() > i) ? msg.position()[i] : 0.0;
                cmd.velocity[i] = (msg.velocity().size() > i) ? msg.velocity()[i] : 0.0;
                cmd.effort[i]   = (msg.effort().size() > i)   ? msg.effort()[i]   : 0.0;
            }
            bridge->dds_to_client.try_push(cmd);
        }

        // Pop data from RT domain ---> Publish to ROS2
        JointState joint_state{};
        while (bridge->server_to_dds.try_pop(joint_state)) {
            out_msg.header().stamp().sec(joint_state.sec);
            out_msg.header().stamp().nanosec(joint_state.nanosec);
            for (int i = 0; i < 12; ++i) {
                out_msg.position()[i] = joint_state.position[i];
                out_msg.velocity()[i] = joint_state.velocity[i];
                out_msg.effort()[i]   = joint_state.effort[i];
            }
            writer.write(out_msg);
        }

        nanosleep(&dt, nullptr);
    }

    std::cout << "[DDS] Shutting down.\n";
    bridge->~SharedBridge();
    munmap(ptr, sizeof(SharedBridge));

    // Unlink the shared memory object (removes /spot_rt_bridge from /dev/shm).
    // Even if the peer process is still using the object, this is okay. 
    // The object will be removed only after all open references are closed
    shm_unlink(SHM_NAME);
    
    return 0;
}
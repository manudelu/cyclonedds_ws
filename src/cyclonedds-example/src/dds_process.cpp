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
void handle_sig(int) { g_stop = true; }

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
    dds::topic::Topic<::sensor_msgs::msg::dds_::JointState_> topic(dp, "rt/advrf/spot/joint_states");

    dds::sub::Subscriber sub(dp);
    dds::sub::qos::DataReaderQos rqos = dds::sub::qos::DataReaderQos()
        << dds::core::policy::Reliability::BestEffort()
        << dds::core::policy::History::KeepLast(1);
    dds::sub::DataReader<::sensor_msgs::msg::dds_::JointState_> reader(sub, topic, rqos);

    dds::pub::Publisher pub(dp);
    dds::pub::qos::DataWriterQos wqos = dds::pub::qos::DataWriterQos()
        << dds::core::policy::Reliability::BestEffort()
        << dds::core::policy::History::KeepLast(1);
    dds::pub::DataWriter<::sensor_msgs::msg::dds_::JointState_> writer(pub, topic, wqos);

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
    std::cout << "[DDS] Ready. Waiting for RT process...\n";
    while (!bridge->rt_ready.load(std::memory_order_acquire) && !g_stop)
        usleep(1000);

    std::cout << "[DDS] Bridge active.\n";

    const struct timespec dt{0, 500000}; // poll at 2 kHz

    while (!g_stop) {
        auto samples = reader.take();
        for (auto const& s : samples) {
            if (!s.info().valid()) 
                continue;

            const auto& msg = s.data();
            JointState joint_state{};
            joint_state.sec = msg.header().stamp().sec();
            joint_state.nanosec = msg.header().stamp().nanosec();
            const auto& pos = msg.position();
            const auto& vel = msg.velocity();
            const auto& eff = msg.effort();
            for (size_t i = 0; i < 12; ++i) {
                joint_state.position[i] = pos[i];
                joint_state.velocity[i] = vel[i];
                joint_state.effort[i]   = eff[i];
            }
            bridge->dds_to_rt.try_push(joint_state);
        }

        JointState cmd{};
        while (bridge->rt_to_dds.try_pop(cmd)) {
            out_msg.header().stamp().sec(cmd.sec);
            out_msg.header().stamp().nanosec(cmd.nanosec);
            for (int i = 0; i < 12; ++i) {
                out_msg.position()[i] = cmd.position[i];
                out_msg.velocity()[i] = cmd.velocity[i];
                out_msg.effort()[i]   = cmd.effort[i];
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
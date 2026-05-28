// Terminal A
// export CYCLONEDDS_URI=file:///home/embedded/cyclonedds_ws/src/cyclonedds-example/cyclonedds.xml
// export LD_LIBRARY_PATH=~/cyclonedds_ws/install/lib/${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
// cd cyclonedds_ws/build/cyclonedds_example
// ./publisher

// Terminal B
// cd cyclonedds_ws/build/iceoryx/
// ./iox-roudi -c /home/embedded/cyclonedds_ws/src/cyclonedds-example/iox_config.toml

#include <iostream>
#include <atomic>
#include <cstring>
#include <pthread.h>
#include <sched.h>
#include <time.h>

#include <dds/dds.hpp>
#include "JointState.hpp"

#include <boost/lockfree/spsc_queue.hpp>

int32_t DOMAIN_ID {42};

// SPSC Queues for JointState msg
struct JointState {
    int32_t sec;
    uint32_t nanosec;
    double position[12];
    double velocity[12];
    double effort[12];
};
boost::lockfree::spsc_queue<JointState, boost::lockfree::capacity<64>> g_inbound;
boost::lockfree::spsc_queue<JointState, boost::lockfree::capacity<64>> g_outbound;

// Global Atomic Variable
std::atomic<bool> g_running{true};

void* dds_worker(void* arg) {
    pthread_setname_np(pthread_self(), "DDS_Linux");

    dds::domain::DomainParticipant dp(DOMAIN_ID);
    dds::topic::Topic<::sensor_msgs::msg::dds_::JointState_> topic(dp, "rt/advrf/spot/joint_states");

    // Subscriber
    dds::sub::Subscriber sub(dp);
    dds::sub::qos::DataReaderQos rqos = dds::sub::qos::DataReaderQos()
        << dds::core::policy::Reliability::BestEffort()
        << dds::core::policy::History::KeepLast(1);
    dds::sub::DataReader<::sensor_msgs::msg::dds_::JointState_> reader(sub, topic, rqos);

    // Publisher
    dds::pub::Publisher pub(dp);
    dds::pub::qos::DataWriterQos wqos = dds::pub::qos::DataWriterQos()
        << dds::core::policy::Reliability::BestEffort()
        << dds::core::policy::History::KeepLast(1);
    dds::pub::DataWriter<::sensor_msgs::msg::dds_::JointState_> writer(pub, topic, wqos);

    // Pre-build the outgoing DDS message shell — no alloc in loop
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

    // Poll at 2 kHz
    const struct timespec dt{0, 500000}; // 0.5 ms

    while (g_running.load(std::memory_order_relaxed)) {

        // Subscriber //
        // Receive from DDS --> push to RT thread
        auto samples = reader.take(); // SysCall - recvmsg()
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

            // Copy JointState data and push into inbound queue
            g_inbound.push(joint_state);
        }

        // Publisher //
        // Drain outbound queue --> publish to DDS
        JointState cmd{};
        while (g_outbound.pop(cmd)) {
            out_msg.header().stamp().sec(cmd.sec);
            out_msg.header().stamp().nanosec(cmd.nanosec);
            for (int i = 0; i < 12; ++i) {
                out_msg.position()[i] = cmd.position[i];
                out_msg.velocity()[i] = cmd.velocity[i];
                out_msg.effort()[i]   = cmd.effort[i];
            }
            writer.write(out_msg);  // SysCall - sendmsg()
        }

        nanosleep(&dt, nullptr);
    }

    return nullptr;
}

void* rt_worker(void*) {
    pthread_setname_np(pthread_self(), "RT_Control");

    // Pre-allocate everything before entering the RT loop
    JointState state{};
    JointState cmd{};

    double pos = 0.0;
    bool incrementing = true;

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    const long long period_ns = 1'000'000LL; // 1 ms

    while (g_running.load(std::memory_order_relaxed)) {

        // Drain inbound queue to keep only the freshest sample
        JointState tmp{};
        while (g_inbound.pop(tmp))
            state = tmp;

        // Control logic 
        if (incrementing) { 
            pos += 0.01; 
            if (pos > 0.3) 
                incrementing = false; 
        } else { 
            pos -= 0.01; 
            if (pos < 0.0) 
                incrementing = true; 
        }

        struct timespec now;
        clock_gettime(CLOCK_REALTIME, &now);
        cmd.sec = static_cast<int32_t>(now.tv_sec);
        cmd.nanosec = static_cast<uint32_t>(now.tv_nsec);
        cmd.position[0] = pos;

        // lock-free write
        g_outbound.push(cmd); 

        next.tv_nsec += period_ns;
        if (next.tv_nsec >= 1'000'000'000L) {
            next.tv_sec  += 1;
            next.tv_nsec -= 1'000'000'000L;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    }

    return nullptr;
}

int main() {
    pthread_t rt_thread, dds_thread;
    pthread_attr_t rt_attr, dds_attr;
    struct sched_param param;

    // RT thread (SCHED_FIFO) 
    pthread_attr_init(&rt_attr);
    pthread_attr_setinheritsched(&rt_attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&rt_attr, SCHED_FIFO);
    param.sched_priority = 80;
    pthread_attr_setschedparam(&rt_attr, &param);
    if (pthread_create(&rt_thread, &rt_attr, rt_worker, nullptr) != 0) {
        std::perror("Failed to create RT thread");
        return 1;
    }
    pthread_attr_destroy(&rt_attr);

    // Non-RT DDS thread (default: SCHED_OTHER) 
    pthread_attr_init(&dds_attr);
    if (pthread_create(&dds_thread, &dds_attr, dds_worker, nullptr) != 0) {
        std::perror("Failed to create DDS thread");
        g_running = false;
        pthread_join(rt_thread, nullptr);
        return 1;
    }
    pthread_attr_destroy(&dds_attr);

    std::cout << "RT control thread: SCHED_FIFO priority 80\n";
    std::cout << "DDS I/O thread:    SCHED_OTHER (Linux)\n";
    std::cout << "Press Enter to stop...\n";
    std::cin.get();

    g_running = false;
    pthread_join(rt_thread,  nullptr);
    pthread_join(dds_thread, nullptr);
    return 0;
}
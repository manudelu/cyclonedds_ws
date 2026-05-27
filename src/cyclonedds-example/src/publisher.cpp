#include <iostream>
#include <chrono>
#include <atomic>
#include <pthread.h>
#include <sched.h>
#include <time.h>

#include <dds/dds.hpp>
#include "Time.hpp"
#include "Header.hpp"
#include "JointState.hpp"

int DOMAIN_ID {42};
std::atomic<bool> running{true};

// Context structure to pass DDS entities to raw POSIX thread wrappers
struct ThreadContext {
    dds::domain::DomainParticipant dp;
    dds::topic::Topic<::sensor_msgs::msg::dds_::JointState_> topic;
};

// --- REAL-TIME SUBSCRIBER THREAD ---
void* subscriber_worker(void* arg) {
    ThreadContext* ctx = static_cast<ThreadContext*>(arg);
    
    pthread_setname_np(pthread_self(), "DDS_RT_Sub");

    dds::sub::Subscriber sub(ctx->dp);
    dds::sub::qos::DataReaderQos qos = dds::sub::qos::DataReaderQos()
        << dds::core::policy::Reliability::BestEffort()
        << dds::core::policy::History::KeepLast(1);
    
    dds::sub::DataReader<::sensor_msgs::msg::dds_::JointState_> reader(sub, ctx->topic, qos);

    struct timespec next_period;
    clock_gettime(CLOCK_MONOTONIC, &next_period);
    const long long period_ns = 1000000LL; // 1ms

    while (running) {
        auto samples = reader.take();
        for (auto const& sample : samples) {
            if (sample.info().valid()) {
                const auto& msg = sample.data();
                //std::cout << msg << '\n';
            }
        }

        next_period.tv_nsec += period_ns;
        if (next_period.tv_nsec >= 1000000000L) {
            next_period.tv_sec += 1;
            next_period.tv_nsec -= 1000000000L;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, NULL);
    }

    return nullptr;
}

// --- REAL-TIME PUBLISHER THREAD ---
void* publisher_worker(void* arg) {
    ThreadContext* ctx = static_cast<ThreadContext*>(arg);
    
    pthread_setname_np(pthread_self(), "DDS_RT_Pub");

    dds::pub::Publisher pub(ctx->dp);
    dds::pub::qos::DataWriterQos qos = dds::pub::qos::DataWriterQos() 
        << dds::core::policy::Reliability::BestEffort()
        << dds::core::policy::History::KeepLast(1);
        
    dds::pub::DataWriter<::sensor_msgs::msg::dds_::JointState_> writer(pub, ctx->topic, qos);

    ::sensor_msgs::msg::dds_::JointState_ msg;
    msg.name() = { "front_left_hip_x",  "front_left_hip_y",  "front_left_knee",
                   "front_right_hip_x", "front_right_hip_y", "front_right_knee",
                   "rear_left_hip_x",   "rear_left_hip_y",   "rear_left_knee",
                   "rear_right_hip_x",  "rear_right_hip_y",  "rear_right_knee" 
    };
    
    // Pre-allocate vector capacity before entering the loop to avoid 
    // real-time memory allocations (which cause mode switches)
    msg.position().resize(12, 0.0);
    msg.velocity().assign(12, 0.0);
    msg.effort().assign(12, 0.0);

    double pos = 0.0;
    bool incrementing = true;
    
    struct timespec next_period;
    clock_gettime(CLOCK_MONOTONIC, &next_period);
    const long long period_ns = 1000000LL; // 1ms

    while(running) {
        struct timespec now;
        clock_gettime(CLOCK_REALTIME, &now);
        msg.header().stamp().sec(static_cast<int32_t>(now.tv_sec));
        msg.header().stamp().nanosec(static_cast<uint32_t>(now.tv_nsec));
        
        if (incrementing) {
            pos += 0.01;
            if (pos > 0.3) incrementing = false;
        } else {
            pos -= 0.01;
            if (pos < 0.0) incrementing = true;
        }
        
        msg.position()[0] = pos; // Modify values directly without resizing/reallocating
        
        writer.write(msg);
        
        next_period.tv_nsec += period_ns;
        if (next_period.tv_nsec >= 1000000000L) {
            next_period.tv_sec += 1;
            next_period.tv_nsec -= 1000000000L;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, NULL);
    }

    return nullptr;
}

int main(int argc, char** argv) {
    dds::domain::DomainParticipant dp(DOMAIN_ID);
    dds::topic::Topic<::sensor_msgs::msg::dds_::JointState_> topic(dp, "rt/advrf/spot/joint_states");
    ThreadContext ctx{dp, topic};

    pthread_t pub_tid, sub_tid;
    pthread_attr_t attr;
    struct sched_param param;

    // Initialize POSIX thread attributes
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);

    // Spawn Subscriber Thread with High Priority (80)
    param.sched_priority = 80;
    pthread_attr_setschedparam(&attr, &param);
    if (pthread_create(&sub_tid, &attr, subscriber_worker, &ctx) != 0) {
        std::perror("Failed to create RT Subscriber thread");
        return 1;
    }

    // Spawn Publisher Thread with Slightly Lower Priority (79)
    param.sched_priority = 79;
    pthread_attr_setschedparam(&attr, &param);
    if (pthread_create(&pub_tid, &attr, publisher_worker, &ctx) != 0) {
        std::perror("Failed to create RT Publisher thread");
        return 1;
    }

    pthread_attr_destroy(&attr);

    std::cout << "Xenomai Native threads deployed. Check /proc/xenomai/sched/stat\n";
    std::cout << "Press Enter to exit...\n";
    std::cin.get();

    running = false;

    pthread_join(sub_tid, nullptr);
    pthread_join(pub_tid, nullptr);

    return 0;
}
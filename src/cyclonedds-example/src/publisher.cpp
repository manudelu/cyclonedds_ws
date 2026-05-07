// ~/CycloneDDS/install/bin/idlc -l cxx Time.idl
// ~/CycloneDDS/install/bin/idlc -l cxx Header.idl
// ~/CycloneDDS/install/bin/idlc -l cxx JointState.idl
// cmake .. -DCMAKE_PREFIX_PATH=$HOME/CycloneDDS/install
// cmake --build . -j

#include <iostream>
#include <thread>
#include <chrono>

#include <dds/dds.hpp>
#include "Time.hpp"
#include "Header.hpp"
#include "JointState.hpp"

int DOMAIN_ID {0};

int main(int argc, char** argv) {

    dds::domain::DomainParticipant dp(DOMAIN_ID);
    dds::topic::Topic<::sensor_msgs::msg::dds_::JointState_> topic(dp, "rt/advrf/spot/joint_states");
    dds::pub::Publisher pub(dp);
    dds::pub::DataWriter<::sensor_msgs::msg::dds_::JointState_> writer(
        pub, topic, dds::pub::qos::DataWriterQos() 
            << dds::core::policy::Reliability::BestEffort()
            << dds::core::policy::History::KeepLast(1)
    );

    ::sensor_msgs::msg::dds_::JointState_ msg;
    msg.name() = { "front_left_hip_x",  "front_left_hip_y",  "front_left_knee",
                   "front_right_hip_x", "front_right_hip_y", "front_right_knee",
                   "rear_left_hip_x",   "rear_left_hip_y",   "rear_left_knee",
                   "rear_right_hip_x",  "rear_right_hip_y",  "rear_right_knee" 
    };
    double pos = 0.0;
    bool incrementing = true;
    
    while(true) {
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
        auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);
        msg.header().stamp().sec(static_cast<int32_t>(seconds.count()));
        msg.header().stamp().nanosec(static_cast<uint32_t>(nanoseconds.count()));
        msg.header().frame_id("");
        
        if (incrementing) {
            pos+=0.01;
            if (pos > 0.3) incrementing = false;
        }
        else {
            pos-=0.01;
            if (pos < 0.0) incrementing = true;
        }
        msg.position() = {pos, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        msg.velocity().assign(12, 0.0);
        msg.effort().assign(12, 0.0);
        writer.write(msg);
        std::cout << "Pose: " << msg.position()[0] << " on joint " <<  msg.name()[0] << '\n';
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
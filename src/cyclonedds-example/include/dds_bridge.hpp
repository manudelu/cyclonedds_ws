#pragma once

#include <dds/dds.hpp>
#include <string>
#include <vector>
#include <iostream>

#include "JointState.hpp"
#include "Imu.hpp"

#include "motor.pb.h"
#include "imu.pb.h"

using TimeMsg       = ::builtin_interfaces::msg::dds_::Time_;
using JointStateMsg = ::sensor_msgs::msg::dds_::JointState_;
using ImuMsg        = ::sensor_msgs::msg::dds_::Imu_;

// ============================================================================
//  BASE CLASS PUBLISHER
// ============================================================================
template <typename Msg, typename Derived>
class DdsPublisher {
public:
    DdsPublisher()
        : publisher_(dds::core::null),
          topic_(dds::core::null),
          writer_(dds::core::null)
    {}

    virtual ~DdsPublisher() = default;

protected:
    bool init_dds(const std::string& topic_name, dds::domain::DomainParticipant& participant) {
        try {
            /* To publish something, a topic is needed. */
            topic_ = dds::topic::Topic<Msg>(participant, topic_name);

            /* A writer also needs a publisher. */
            publisher_ = dds::pub::Publisher(participant);

            /* It is possible to modify writer default QoS */
            dds::pub::qos::DataWriterQos qos = static_cast<Derived*>(this)->writer_qos();

            /* Now, the writer can be created to publish a message. */
            writer_ = dds::pub::DataWriter<Msg>(publisher_, topic_, qos);

            return true;
        } 
        catch (const dds::core::Exception& e) {
            std::cerr << "DDS Init Error on topic [" << topic_name << "]: " << e.what() << '\n';
            return false;
        }
    }

    /* QoS here is set as default to Best Effort and Keep Last = 1 for every writer */
    dds::pub::qos::DataWriterQos writer_qos() {
        return dds::pub::qos::DataWriterQos()
            << dds::core::policy::Reliability::BestEffort()
            << dds::core::policy::History::KeepLast(1);
    }

    static void set_timestamp(TimeMsg& stamp, int32_t sec, uint32_t nanosec) {
        stamp.sec(sec);
        stamp.nanosec(nanosec);
    }

    dds::pub::Publisher publisher_;
    dds::topic::Topic<Msg> topic_;
    dds::pub::DataWriter<Msg> writer_;
};

// ============================================================================
// CLASSE BASE SUBSCRIBER
// ============================================================================
template <typename Msg, typename Derived>
class DdsSubscriber {
public:
    DdsSubscriber() 
        : subscriber_(dds::core::null), 
          topic_(dds::core::null), 
          reader_(dds::core::null) 
    {}

    virtual ~DdsSubscriber() = default;

protected:
    bool init_dds(const std::string& topic_name, dds::domain::DomainParticipant& participant) {
        try {
            /* To subscribe to something, a topic is needed. */
            topic_ = dds::topic::Topic<Msg>(participant, topic_name);

            /* A reader also needs a publisher. */
            subscriber_ = dds::sub::Subscriber(participant);

            /* It is possible to modify writer default QoS */
            dds::sub::qos::DataReaderQos qos = static_cast<Derived*>(this)->reader_qos();
    
            /* Now, the reader can be created to subscribe to a message. */
            reader_ = dds::sub::DataReader<Msg>(subscriber_, topic_, qos);
            reader_.listener(static_cast<Derived*>(this), dds::core::status::StatusMask::data_available());
            
            return true;
        } 
        catch (const dds::core::Exception& e) {
            std::cerr << "DDS Sub Init Error [" << topic_name << "]: " << e.what() << '\n';
            return false;
        }
    }

    /* QoS here is set as default to Best Effort and Keep Last = 1 for every writer */
    dds::sub::qos::DataReaderQos reader_qos() {
        return dds::sub::qos::DataReaderQos()
            << dds::core::policy::Reliability::BestEffort()
            << dds::core::policy::History::KeepLast(1);
    }

    dds::sub::Subscriber subscriber_;
    dds::topic::Topic<Msg> topic_;
    dds::sub::DataReader<Msg> reader_;
};

// ============================================================================
// JOINT STATE PUBLISHER
// ============================================================================
class JointStatePublisher : public DdsPublisher<JointStateMsg, JointStatePublisher> {
public:
    using Base = DdsPublisher<JointStateMsg, JointStatePublisher>;
    friend Base;

    JointStatePublisher() : Base() {}

    bool init(const std::string& robot_name, dds::domain::DomainParticipant& participant) {
        const std::string topic_name = "rt/advrf/" + robot_name + "/joint_states";
        if (!Base::init_dds(topic_name, participant))
            return false;

        js_msg_.name() = {
            "front_left_hip_x",  "front_left_hip_y",  "front_left_knee",
            "front_right_hip_x", "front_right_hip_y", "front_right_knee",
            "rear_left_hip_x",   "rear_left_hip_y",   "rear_left_knee",
            "rear_right_hip_x",  "rear_right_hip_y",  "rear_right_knee"
        };
        js_msg_.position().resize(12, 0.0);
        js_msg_.velocity().assign(12, 0.0);
        js_msg_.effort().assign(12, 0.0);
        js_msg_.header().frame_id() = "";
        return true;
    }

    void publish(const iit::advrf::MotorState& proto_msg) {
        Base::set_timestamp(js_msg_.header().stamp(), proto_msg.sec(), proto_msg.nanosec());
        
        for (int i = 0; i < 12; ++i) {
            const auto& m = proto_msg.motors(i);
            js_msg_.position()[i] = static_cast<double>(m.link_pos());
            js_msg_.velocity()[i] = static_cast<double>(m.link_vel());
            js_msg_.effort()[i]   = static_cast<double>(m.torque());
        }

        try {
            writer_.write(js_msg_);
        } catch (const dds::core::Exception& e) {
            std::cerr << "[JointStatePublisher] Write error: " << e.what() << '\n';
        }
    }

private:
    JointStateMsg js_msg_;
};

// ============================================================================
// IMU PUBLISHER
// ============================================================================
class ImuPublisher : public DdsPublisher<ImuMsg, ImuPublisher> {
public:
    using Base = DdsPublisher<ImuMsg, ImuPublisher>;
    friend Base;

    ImuPublisher() : Base() {}

    bool init(const std::string& robot_name, dds::domain::DomainParticipant& participant) {
        const std::string topic_name = "rt/advrf/" + robot_name + "/imu";
        if (!Base::init_dds(topic_name, participant))
            return false;

        imu_msg_.header().frame_id() = "imu_link";
        return true;
    }

    void publish(const iit::advrf::ImuState& proto_msg) {
        Base::set_timestamp(imu_msg_.header().stamp(), proto_msg.sec(), proto_msg.nanosec());

        imu_msg_.orientation().x(static_cast<double>(proto_msg.orient_x()));
        imu_msg_.orientation().y(static_cast<double>(proto_msg.orient_y()));
        imu_msg_.orientation().z(static_cast<double>(proto_msg.orient_z()));
        imu_msg_.orientation().w(static_cast<double>(proto_msg.orient_w()));

        imu_msg_.angular_velocity().x(static_cast<double>(proto_msg.ang_vel_x()));
        imu_msg_.angular_velocity().y(static_cast<double>(proto_msg.ang_vel_y()));
        imu_msg_.angular_velocity().z(static_cast<double>(proto_msg.ang_vel_z()));

        imu_msg_.linear_acceleration().x(static_cast<double>(proto_msg.lin_acc_x()));
        imu_msg_.linear_acceleration().y(static_cast<double>(proto_msg.lin_acc_y()));
        imu_msg_.linear_acceleration().z(static_cast<double>(proto_msg.lin_acc_z()));

        try {
            writer_.write(imu_msg_);
        } catch (const dds::core::Exception& e) {
            std::cerr << "[ImuPublisher] Write error: " << e.what() << '\n';
        }
    }

private:
    ImuMsg imu_msg_;
};

// ============================================================================
// CENTRAL MANAGER
// ============================================================================
class DDSPublisherManager {
private:
    dds::domain::DomainParticipant dp_;
    JointStatePublisher js_pub_;
    ImuPublisher imu_pub_;

public:
    DDSPublisherManager(uint32_t domain_id, const std::string& robot_name)
        : dp_(domain_id) 
    {
        if (!js_pub_.init(robot_name, dp_)) {
            std::cerr << "[DDS Manager] Fallimento init JointStatePublisher\n";
        }
        if (!imu_pub_.init(robot_name, dp_)) {
            std::cerr << "[DDS Manager] Fallimento init ImuPublisher\n";
        }
    }

    void publish_joint_state(const iit::advrf::MotorState& proto) { js_pub_.publish(proto); }
    void publish_imu(const iit::advrf::ImuState& proto)         { imu_pub_.publish(proto); }
};
#pragma once
#include "mw_adapter.hpp"

#include <dds/dds.hpp>
#include "JointState.hpp"
#include "Imu.hpp"
#include "JointTrajectory.hpp"

#include <iostream>

using TimeMsg            = ::builtin_interfaces::msg::dds_::Time_;
using JointStateMsg      = ::sensor_msgs::msg::dds_::JointState_;
using ImuMsg             = ::sensor_msgs::msg::dds_::Imu_;
using JointTrajectoryMsg = ::trajectory_msgs::msg::dds_::JointTrajectory_;

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
// JOINT TRAJECTORY SUBSCRIBER
// ============================================================================
class JointTrajectorySubscriber : public DdsSubscriber<JointTrajectoryMsg, JointTrajectorySubscriber> {
public:
    using Base = DdsSubscriber<JointTrajectoryMsg, JointTrajectorySubscriber>;
    friend Base;

    JointTrajectorySubscriber() : Base() {}

    bool init(const std::string& robot_name, dds::domain::DomainParticipant& participant) {
        const std::string topic_name = "rt/advrf/" + robot_name + "/joint_trajectory";
        return Base::init_dds(topic_name, participant);
    }

    // ????
    bool take(JointTrajectoryMsg& out_msg) {
        try {
            auto samples = reader_.take();
            for (const auto& sample : samples) {
                if (sample.info().valid()) {
                    out_msg = sample.data();
                    return true; 
                }
            }
        } catch (const dds::core::Exception& e) {
            std::cerr << "[JointTrajectorySubscriber] Read error: " << e.what() << '\n';
        }
        return false;
    }
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
// DDS ADAPTER
// ============================================================================

class DdsAdapter : public MiddlewareAdapter {
public:
    static constexpr uint32_t DOMAIN_ID = 0;

    bool init(const std::string& robot_name) override {
        dp_ = dds::domain::DomainParticipant(DOMAIN_ID);

        // TODO: Choose which to create based on what devices I have
        if (!js_pub_.init(robot_name, dp_))
            std::cerr << "[DDS] Failed to init JointStatePublisher\n";
        if (!imu_pub_.init(robot_name, dp_))
            std::cerr << "[DDS] Failed to init ImuPublisher\n";
        if (!jt_sub_.init(robot_name, dp_))
            std::cerr << "[DDS] Failed to init JointTrajectorySubscriber\n";  

        cmd_.mutable_motors()->Reserve(12);
        for (int i = 0; i < 12; ++i)
            cmd_.add_motors();

        return true;
    }

    void publish_joint_state(const iit::advrf::MotorState& msg) override {
        js_pub_.publish(msg);
    }

    void publish_imu(const iit::advrf::ImuState& msg) override {
        imu_pub_.publish(msg);
    }

    bool take_joint_command(iit::advrf::MotorCmd& out) override {
        JointTrajectoryMsg jt_msg;
        if (!jt_sub_.take(jt_msg) || jt_msg.points().empty()) 
            return false;

        const auto& pt = jt_msg.points()[0];
        for (int i = 0; i < 12; ++i) {
            auto* m = out.mutable_motors(i);
            m->set_pos_ref(i < (int)pt.positions().size()  ? static_cast<float>(pt.positions()[i])  : 0.0f);
            m->set_vel_ref(i < (int)pt.velocities().size() ? static_cast<float>(pt.velocities()[i]) : 0.0f);
            m->set_torque_ffwd(i < (int)pt.effort().size() ? static_cast<float>(pt.effort()[i])     : 0.0f);
        }
        out = cmd_;  // carry pre-allocated structure
        return true;
    }

private:
    dds::domain::DomainParticipant dp_{dds::core::null};
    JointStatePublisher js_pub_;
    ImuPublisher imu_pub_;
    JointTrajectorySubscriber jt_sub_;
    iit::advrf::MotorCmd cmd_;
};
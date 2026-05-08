#pragma once

#include <dds/dds.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <cstdint>

#include "msg.hpp"

#include "JointState.hpp"
#include "Imu.hpp"
#include "ForceTorque.hpp"
#include "Motor.hpp"
#include "PowerBoard.hpp"
#include "Pump.hpp"
#include "Valve.hpp"

using TimeMsg        = ::builtin_interfaces::msg::dds_::Time_;
using JointStateMsg  = ::sensor_msgs::msg::dds_::JointState_;
using ImuMsg         = ::advrf_msgs::msg::dds_::Imu_;
using ForceTorqueMsg = ::advrf_msgs::msg::dds_::ForceTorque_;
using MotorMsg       = ::advrf_msgs::msg::dds_::Motor_;
using PowerBoardMsg  = ::advrf_msgs::msg::dds_::PowerBoard_;
using PumpMsg        = ::advrf_msgs::msg::dds_::Pump_;
using ValveMsg       = ::advrf_msgs::msg::dds_::Valve_;

////////////////////////////
//  Base Class Publisher  //
////////////////////////////

template <typename Msg, typename Derived>
class DdsPublisher {
    public:

        explicit DdsPublisher() 
            : participant_(dds::core::null)
            , publisher_(dds::core::null)
            , topic_(dds::core::null)
            , writer_(dds::core::null)
        {}
        virtual ~DdsPublisher() = default;

    protected:

        bool init_dds(const std::string& topic_name, int domain_id) {
            try {
                participant_ = dds::domain::DomainParticipant(domain_id);
                topic_ = dds::topic::Topic<Msg>(participant_, topic_name);
                publisher_ = dds::pub::Publisher(participant_);
                dds::pub::qos::DataWriterQos qos = static_cast<Derived*>(this)->writer_qos();
                writer_ = dds::pub::DataWriter<Msg>(publisher_, topic_, qos);
                return true;
            } catch (const dds::core::Exception& e) {
                std::cerr << "DDS Init Error: " << e.what() << '\n';
                return false;
            }
        }

        dds::pub::qos::DataWriterQos writer_qos() {
            return dds::pub::qos::DataWriterQos()
                << dds::core::policy::Reliability::BestEffort()
                << dds::core::policy::History::KeepLast(1);
        }

        static void set_timestamp(TimeMsg& stamp, uint64_t timestamp_ns) {
            stamp.sec(static_cast<int32_t>(timestamp_ns / 1'000'000'000ULL));
            stamp.nanosec(static_cast<uint32_t>(timestamp_ns % 1'000'000'000ULL));
        }

        dds::domain::DomainParticipant participant_;
        dds::pub::Publisher publisher_;
        dds::topic::Topic<Msg> topic_;
        dds::pub::DataWriter<Msg> writer_;
};

///////////////////////////
// Joint State Publisher //
///////////////////////////

class JointStatePublisher : public DdsPublisher<JointStateMsg, JointStatePublisher> {

public:

    using Base = DdsPublisher<JointStateMsg, JointStatePublisher>;
    friend Base;

    JointStatePublisher() : Base() {}
    ~JointStatePublisher() = default;

    bool init(const std::vector<std::string>& joint_names, 
              const std::string& robot_name,
              int domain_id)
    {
        const std::string topic_name = "rt/advrf/" + robot_name + "/joint_states";
        if (!Base::init_dds(topic_name, domain_id))
            return false;

        joint_state_.name() = joint_names;
        joint_state_.position().assign(joint_names.size(), 0.0);
        joint_state_.velocity().assign(joint_names.size(), 0.0);
        joint_state_.effort().assign(joint_names.size(), 0.0);
        joint_state_.header().frame_id() = "";

        return true;
    }

    void publish(const joint_state::rt_joint_state_msg& msg)
    {
        joint_state_.header().frame_id("");
        Base::set_timestamp(joint_state_.header().stamp(), msg.header.timestamp_ns);    
        joint_state_.position() = msg.positions;
        joint_state_.velocity() = msg.velocities;
        joint_state_.effort() = msg.efforts;

        try {
            writer_.write(joint_state_);
        } catch (const dds::core::Exception& e) {
            std::cerr << "[JointStatePublisher] Write error: " << e.what() << '\n';
        }
    }

    // If you want to change the behaviour of the default QoS method:
    /* 
    static dds::pub::qos::DataWriterQos writer_qos() {
        auto qos = DdsPublisher::writer_qos(); 
        qos << dds::core::policy::Reliability::Reliable()
            << dds::core::policy::History::KeepLast(10);
        return qos;
    }
    */

    private:

        JointStateMsg joint_state_;
};

///////////////////////////
//     Imu Publisher     //
///////////////////////////

class ImuPublisher : public DdsPublisher<ImuMsg, ImuPublisher> {

public:

    using Base = DdsPublisher<ImuMsg, ImuPublisher>;
    friend Base;

    ImuPublisher() : Base() {}
    ~ImuPublisher() = default;

    bool init(const std::string& robot_name,
              int domain_id)
    {
        const std::string topic_name = "rt/advrf/" + robot_name + "/imu";
        if (!Base::init_dds(topic_name, domain_id))
            return false;

        imu_.header().frame_id() = "";

        return true;
    }

    void publish(const imu::rt_imu_msg& msg)
    {
        Base::set_timestamp(imu_.header().stamp(), msg.header.timestamp_ns); 

        imu_.linear_acceleration().x()  = msg.linear_acceleration.x;
        imu_.linear_acceleration().y()  = msg.linear_acceleration.y;
        imu_.linear_acceleration().z()  = msg.linear_acceleration.z;
        
        imu_.angular_velocity().x()     = msg.angular_velocity.x;
        imu_.angular_velocity().y()     = msg.angular_velocity.y;
        imu_.angular_velocity().z()     = msg.angular_velocity.z;
        
        imu_.orientation().x()          = msg.orientation.x;
        imu_.orientation().y()          = msg.orientation.y;
        imu_.orientation().z()          = msg.orientation.z;
        imu_.orientation().w()          = msg.orientation.w;
        
        imu_.imu_ts()                   = msg.imu_ts;
        imu_.temperature()              = msg.temperature;
        imu_.digital_in()               = msg.digital_in;
        imu_.fault()                    = msg.fault;
        imu_.rtt()                      = msg.rtt;

        try {
            writer_.write(imu_);
        } catch (const dds::core::Exception& e) {
            std::cerr << "[ImuPublisher] Write error: " << e.what() << '\n';
        }
    }

    private:

        ImuMsg imu_;
};

////////////////////////////
// Force Torque Publisher //
////////////////////////////

class ForceTorquePublisher : public DdsPublisher<ForceTorqueMsg, ForceTorquePublisher> {

public:

    using Base = DdsPublisher<ForceTorqueMsg, ForceTorquePublisher>;
    friend Base;

    ForceTorquePublisher() : Base() {}
    ~ForceTorquePublisher() = default;

    bool init(const std::string& robot_name,
              int domain_id)
    {
        const std::string topic_name = "rt/advrf/" + robot_name + "/force_torque";
        if (!Base::init_dds(topic_name, domain_id))
            return false;

        ft_.header().frame_id() = "";

        return true;
    }

    void publish(const force_torque::rt_force_torque_msg& msg)
    {
        Base::set_timestamp(ft_.header().stamp(), msg.header.timestamp_ns); 

        ft_.wrench().force().x()       = msg.wrench.force.x;
        ft_.wrench().force().y()       = msg.wrench.force.y;
        ft_.wrench().force().z()       = msg.wrench.force.z;
        
        ft_.wrench().torque().x()      = msg.wrench.torque.x;
        ft_.wrench().torque().y()      = msg.wrench.torque.y;
        ft_.wrench().torque().z()      = msg.wrench.torque.z;
    
        ft_.fault()                    = msg.fault;
        ft_.rtt()                      = msg.rtt;
        ft_.op_idx_ack()               = msg.op_idx_ack;
        ft_.aux()                      = msg.aux;

        try {
            writer_.write(ft_);
        } catch (const dds::core::Exception& e) {
            std::cerr << "[ForceTorquePublisher] Write error: " << e.what() << '\n';
        }
    }

    private:

        ForceTorqueMsg ft_;
};

////////////////////////////
//     Motor Publisher    //
////////////////////////////

class MotorPublisher : public DdsPublisher<MotorMsg, MotorPublisher> {

public:

    using Base = DdsPublisher<MotorMsg, MotorPublisher>;
    friend Base;

    MotorPublisher() : Base() {}
    ~MotorPublisher() = default;

    bool init(const std::vector<std::string>& motor_names,
              const std::string& robot_name,
              int domain_id)
    {
        const std::string topic_name = "rt/advrf/" + robot_name + "/motor";
        if (!Base::init_dds(topic_name, domain_id))
            return false;

        const size_t n = motor_names.size();
        motor_.name() = motor_names;
        motor_.statusword().assign(n, 0);
        motor_.modes_of_op().assign(n, 0);
        motor_.motor_pos().assign(n, 0.0f);
        motor_.motor_vel().assign(n, 0.0f);
        motor_.link_pos().assign(n, 0.0f);
        motor_.link_vel().assign(n, 0.0f);
        motor_.current().assign(n, 0.0f);
        motor_.torque().assign(n, 0.0f);
        motor_.demanded_pos().assign(n, 0.0f);
        motor_.demanded_vel().assign(n, 0.0f);
        motor_.demanded_torque().assign(n, 0.0f);
        motor_.demanded_current().assign(n, 0.0f);
        motor_.control_effort().assign(n, 0.0f);
        motor_.motor_temp().assign(n, 0.0f);
        motor_.drive_temp().assign(n, 0);
        motor_.error_code().assign(n, 0);
        motor_.error_report().assign(n, "");
        motor_.fault().assign(n, 0);
        motor_.rtt().assign(n, 0);
        motor_.header().frame_id() = "";

        return true;
    }

    void publish(const motor::rt_motor_msg& msg)
    {
        Base::set_timestamp(motor_.header().stamp(), msg.header.timestamp_ns); 

        motor_.statusword()       = msg.statusword;
        motor_.modes_of_op()      = msg.modes_of_op;
        motor_.motor_pos()        = msg.motor_pos;
        motor_.motor_vel()        = msg.motor_vel;
        motor_.link_pos()         = msg.link_pos;
        motor_.link_vel()         = msg.link_vel;
        motor_.current()          = msg.current;
        motor_.torque()           = msg.torque;
        motor_.demanded_pos()     = msg.demanded_pos;
        motor_.demanded_vel()     = msg.demanded_vel;
        motor_.demanded_torque()  = msg.demanded_torque;
        motor_.demanded_current() = msg.demanded_current;
        motor_.control_effort()   = msg.control_effort;
        motor_.motor_temp()       = msg.motor_temp;
        motor_.drive_temp()       = msg.drive_temp;
        motor_.error_code()       = msg.error_code;
        motor_.fault()            = msg.fault;
        motor_.rtt()              = msg.rtt;

        try {
            writer_.write(motor_);
        } catch (const dds::core::Exception& e) {
            std::cerr << "[MotorPublisher] Write error: " << e.what() << '\n';
        }
    }

    private:

        MotorMsg motor_;
};

///////////////////////////
// Power Board Publisher //
///////////////////////////

class PowerBoardPublisher : public DdsPublisher<PowerBoardMsg, PowerBoardPublisher> {

public:

    using Base = DdsPublisher<PowerBoardMsg, PowerBoardPublisher>;
    friend Base;

    PowerBoardPublisher() : Base() {}
    ~PowerBoardPublisher() = default;

    bool init(const std::string& robot_name,
              int domain_id)
    {
        const std::string topic_name = "rt/advrf/" + robot_name + "/power_board";
        if (!Base::init_dds(topic_name, domain_id))
            return false;

        pb_.header().frame_id() = "";

        return true;
    }

    void publish(const power_board::rt_power_board_msg& msg)
    {
        Base::set_timestamp(pb_.header().stamp(), msg.header.timestamp_ns); 

        pb_.v_batt()        = msg.v_batt;
        pb_.v_load()        = msg.v_load;
        pb_.i_load()        = msg.i_load;

        pb_.temperature()   = msg.temperature;
        pb_.temp_batt()     = msg.temp_batt;
        pb_.temp_heatsink() = msg.temp_heatsink;
        
        pb_.status()        = msg.status;
        pb_.fault()         = msg.fault;
        pb_.rtt()           = msg.rtt;
        pb_.op_idx_ack()    = msg.op_idx_ack;
        pb_.aux()           = msg.aux;

        try {
            writer_.write(pb_);
        } catch (const dds::core::Exception& e) {
            std::cerr << "[PowerBoardPublisher] Write error: " << e.what() << '\n';
        }
    }

    private:

        PowerBoardMsg pb_;
};

///////////////////////////
//     Pump Publisher    //
///////////////////////////

class PumpPublisher : public DdsPublisher<PumpMsg, PumpPublisher> {

public:

    using Base = DdsPublisher<PumpMsg, PumpPublisher>;
    friend Base;

    PumpPublisher() : Base() {}
    ~PumpPublisher() = default;

    bool init(const std::string& robot_name,
              int domain_id)
    {
        const std::string topic_name = "rt/advrf/" + robot_name + "/pump";
        if (!Base::init_dds(topic_name, domain_id))
            return false;

        pump_.header().frame_id() = "";

        return true;
    }

    void publish(const pump::rt_pump_msg& msg)
    {
        Base::set_timestamp(pump_.header().stamp(), msg.header.timestamp_ns); 

        pump_.motor_current()            = msg.motor_current;
        pump_.motor_speed()              = msg.motor_speed;

        pump_.pressure1()                = msg.pressure1;
        pump_.pressure2()                = msg.pressure2;

        pump_.temperature()              = msg.temperature;
        pump_.mosfet_temperature()       = msg.mosfet_temperature;
        pump_.motor_temperature()        = msg.motor_temperature;
        
        pump_.fault()                    = msg.fault;
        pump_.rtt()                      = msg.rtt;
        pump_.op_idx_ack()               = msg.op_idx_ack;
        pump_.aux()                      = msg.aux;

        try {
            writer_.write(pump_);
        } catch (const dds::core::Exception& e) {
            std::cerr << "[PumpPublisher] Write error: " << e.what() << '\n';
        }
    }

    private:

        PumpMsg pump_;
};

////////////////////////////
//     Valve Publisher    //
////////////////////////////

class ValvePublisher : public DdsPublisher<ValveMsg, ValvePublisher> {

public:

    using Base = DdsPublisher<ValveMsg, ValvePublisher>;
    friend Base;

    ValvePublisher() : Base() {}
    ~ValvePublisher() = default;

    bool init(const std::vector<std::string>& valve_names,
              const std::string& robot_name,
              int domain_id)
    {
        const std::string topic_name = "rt/advrf/" + robot_name + "/valve";
        if (!Base::init_dds(topic_name, domain_id))
            return false;

        const size_t n = valve_names.size();
        valve_.name() = valve_names;
        valve_.encoder_position().assign(n, 0.f);
        valve_.force().assign(n, 0.f);
        valve_.pressure1().assign(n, 0.f);
        valve_.pressure2().assign(n, 0.f);
        valve_.current().assign(n, 0.f);
        valve_.temperature().assign(n, 0.f);
        valve_.current_ref_fb().assign(n, 0.f);
        valve_.position_ref_fb().assign(n, 0.f);
        valve_.force_ref_fb().assign(n, 0.f);
        valve_.fault().assign(n, 0);
        valve_.rtt().assign(n, 0);
        valve_.op_idx_ack().assign(n, 0);
        valve_.aux().assign(n, 0.f);
        valve_.header().frame_id() = "";

        return true;
    }

    void publish(const valve::rt_valve_msg& msg)
    {
        Base::set_timestamp(valve_.header().stamp(), msg.header.timestamp_ns); 

        valve_.encoder_position() = msg.encoder_position;
        valve_.force()            = msg.force;
        valve_.pressure1()        = msg.pressure1;
        valve_.pressure2()        = msg.pressure2;
        valve_.current()          = msg.current;
        valve_.temperature()      = msg.temperature;
        valve_.current_ref_fb()   = msg.current_ref_fb;
        valve_.position_ref_fb()  = msg.position_ref_fb;
        valve_.force_ref_fb()     = msg.force_ref_fb;
        valve_.fault()            = msg.fault;
        valve_.rtt()              = msg.rtt;
        valve_.op_idx_ack()       = msg.op_idx_ack;
        valve_.aux()              = msg.aux;

        try {
            writer_.write(valve_);
        } catch (const dds::core::Exception& e) {
            std::cerr << "[ValvePublisher] Write error: " << e.what() << '\n';
        }
    }

    private:

        ValveMsg valve_;
};
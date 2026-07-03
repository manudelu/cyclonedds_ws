#pragma once
#include "motor.pb.h"
#include "imu.pb.h"
#include <string>

class MiddlewareAdapter {
public:
    virtual ~MiddlewareAdapter() = default;

    virtual bool init(const std::string& robot_name) = 0;

    // RT --> middleware
    virtual void publish_joint_state(const iit::advrf::MotorState&) = 0;
    virtual void publish_imu(const iit::advrf::ImuState&) = 0;

    // middleware --> RT
    // Returns false if nothing new
    virtual bool take_joint_command(iit::advrf::MotorCmd&) = 0;
};
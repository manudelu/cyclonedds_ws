#pragma once
#include <string>
#include <vector>
#include <optional>

enum class JointType 
{ 
    Motor,  // default   - CIA402, XT_MOTOR, etc. (revolute joint)
    Valve,  // hydraulic - HYQ_KNEE, etc.         (prismatic joint)
};

struct JointConfig
{
    std::string name;
    std::optional<int> ecat_id;
    std::optional<std::string> pipe;
    JointType type = JointType::Motor;
};

// Default rates — overridable from YAML
struct PublishRates {
    // Joint / Motor / Valve state topics
    int joint_state_hz  = 100;
    int motor_hz        = 100;
    int valve_hz        = 100;

    // Sensor topics
    int imu_hz          = 100;
    int force_torque_hz = 100;
    int power_board_hz  = 100;
    int pump_hz         = 100;
};

struct RobotConfig
{
    std::string name;
    int domain = 0;
    std::vector<JointConfig> joints;
    PublishRates rates;
};
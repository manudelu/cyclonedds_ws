#pragma once
#include "robot_config.hpp"
#include "pipe_discovery.hpp"

#include <yaml-cpp/yaml.h>
#include <stdexcept>
#include <string>

struct LoadedConfig
{
    RobotConfig cfg;
    ResolvedPipes pipes;
};

inline LoadedConfig load_robot_config(const std::string& path)
{
    YAML::Node root;
    try{
        root = YAML::LoadFile(path);
    }catch(const std::exception& e){
        throw std::runtime_error("Failed to load YAML: " + std::string(e.what()));
    }

    const YAML::Node r = root["robot"];

    RobotConfig r_cfg;
    r_cfg.name = r["name"].as<std::string>("NoNe");
    r_cfg.domain = r["domain"].as<int>(0);

    // ***** Joints *****
    if (r["joints"])
        for (const auto& joint : r["joints"])
        {
            JointConfig j_cfg;
            if (joint["name"] && !joint["name"].IsNull())
                j_cfg.name = joint["name"].as<std::string>();

            if (joint["ecat_id"] && !joint["ecat_id"].IsNull())
                j_cfg.ecat_id = joint["ecat_id"].as<int>();

            // type: "valve" → JointType::Valve; anything else → Motor (default)
            if (joint["type"] && !joint["type"].IsNull()) {
                const auto t = joint["type"].as<std::string>();
                j_cfg.type = (t == "valve") ? JointType::Valve : JointType::Motor;
            }

            r_cfg.joints.push_back(std::move(j_cfg));
        }

    // ***** Rates *****
    if (const YAML::Node& rates = r["rates"])
    {
        auto load = [&](const char* key, int& field) {
            if (rates[key] && !rates[key].IsNull()) 
                field = rates[key].as<int>();
        };
        
        load("joint_state_hz",  r_cfg.rates.joint_state_hz);
        load("motor_hz",        r_cfg.rates.motor_hz);
        load("valve_hz",        r_cfg.rates.valve_hz);
        load("imu_hz",          r_cfg.rates.imu_hz);
        load("force_torque_hz", r_cfg.rates.force_torque_hz);
        load("power_board_hz",  r_cfg.rates.power_board_hz);
        load("pump_hz",         r_cfg.rates.pump_hz);
    }

    auto pipes = resolve_pipes(r_cfg);

    return LoadedConfig{std::move(r_cfg), std::move(pipes)};
}
#pragma once
#include "robot_config.hpp"

#include <experimental/filesystem>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>
#include <algorithm>

#ifdef __COBALT__
static const std::string XDDP_PREFIX = "/proc/xenomai/registry/rtipc/xddp/";
#else
static const std::string XDDP_PREFIX = "/tmp/nrt_pipes/xddp/";
#endif

enum class PipeKind
{
    Motor,        
    Imu,          
    PowerBoard,   
    ForceTorque,  
    Valve,        
    Pump,        
    Unknown
};

struct DiscoveredPipe
{
    std::string pipe_name;
    int ecat_id = 0;
    PipeKind kind = PipeKind::Unknown;
};

/**
 * Pipe names: {robot_name}@{EscType}_id_{N}
 *   The ecat_id is always the last '_'-delimited numeric token
 *   The ESC type is everything between '@' and '_id_'
*/
inline bool parse_pipe_name(const std::string& filename,
                            std::string& esc_type_out,
                            int& ecat_id_out)
{
    // Strip robot prefix
    const auto at = filename.find('@');
    const std::string rest = (at != std::string::npos)
                           ? filename.substr(at + 1)
                           : filename;

    // id is the last '_'-delimited token
    const auto last_us = rest.rfind('_');
    if (last_us == std::string::npos) return false;

    ecat_id_out = std::atoi(rest.substr(last_us + 1).c_str());
    if (ecat_id_out <= 0) return false;

    // ESC type is everything before the last "_id_" marker
    const auto id_marker = rest.rfind("_id_");
    if (id_marker == std::string::npos) return false;

    esc_type_out = rest.substr(0, id_marker);
    return true;
}

inline PipeKind classify_esc_type(const std::string& esc_type)
{
    // Transform string to lowercase for case-insensitive matching
    std::string s = esc_type;
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);

    if (s.find("imu") != std::string::npos)
        return PipeKind::Imu;
    else if (s.find("ft") != std::string::npos)
        return PipeKind::ForceTorque;
    else if (s.find("pow") != std::string::npos)
        return PipeKind::PowerBoard;
    else if (s.find("hpu") != std::string::npos)
        return PipeKind::Pump;
    else if (s.find("valve") != std::string::npos)
        return PipeKind::Valve;
    else if (s.find("motor") != std::string::npos)
        return PipeKind::Motor;
    else
        return PipeKind::Unknown;
}

inline std::vector<DiscoveredPipe> discover_all_pipes(const std::string& prefix = XDDP_PREFIX)
{
    std::vector<DiscoveredPipe> result;

    if (!std::experimental::filesystem::exists(prefix))
    {
        std::cerr << "[PipeDiscovery] Path not found: " << prefix << '\n';
        return result;
    }

    // Iterate over all the elements of the directory
    for (const auto& entry : std::experimental::filesystem::directory_iterator(prefix))
    {
        const std::string filename = entry.path().filename().u8string();
        std::string esc_type;
        int ecat_id = 0;

        if (!parse_pipe_name(filename, esc_type, ecat_id)) {
            std::cout << "[PipeDiscovery] Skipping: " << filename << '\n';
            continue;
        }

        const PipeKind kind = classify_esc_type(esc_type);
        if (kind == PipeKind::Unknown) {
            std::cout << "[PipeDiscovery] Unknown ESC type '" << esc_type
                      << "' in pipe " << filename << " — skipping\n";
            continue;
        }

        const char* kind_str =
            kind == PipeKind::Motor       ? "Motor"       :
            kind == PipeKind::Imu         ? "IMU"         :
            kind == PipeKind::PowerBoard  ? "PowerBoard"  :
            kind == PipeKind::ForceTorque ? "ForceTorque" :
            kind == PipeKind::Valve       ? "Valve"       :
                                            "Pump";

        std::cout << "[PipeDiscovery] Found " << filename
                  << "  ecat_id=" << ecat_id
                  << "  kind=" << kind_str << '\n';

        result.push_back({filename, ecat_id, kind});
    }

    return result;
}

struct ResolvedPipes
{
    std::vector<DiscoveredPipe> motor_pipes;    // PipeKind::Motor
    std::vector<DiscoveredPipe> valve_pipes;    // PipeKind::Valve
    std::vector<DiscoveredPipe> sensor_pipes;   // everything else (IMU, FT, …)
};

// Matches joint ecat_ids to discovered pipes.
inline ResolvedPipes resolve_pipes(RobotConfig& cfg)
{
    const auto all = discover_all_pipes();

    std::map<int, std::string> motor_map;
    std::map<int, std::string> valve_map;
    for (const auto& p : all) {
        if (p.kind == PipeKind::Motor) 
            motor_map[p.ecat_id] = p.pipe_name;
        if (p.kind == PipeKind::Valve) 
            valve_map[p.ecat_id] = p.pipe_name;
    }

    // Resolve joint pipes
    for (auto& j : cfg.joints)
    {
        if (!j.ecat_id.has_value()) {
            std::cout << "[PipeDiscovery] Joint '" << j.name
                      << "' has no ecat_id configured\n";
            continue;
        }

        const auto& lookup = (j.type == JointType::Valve) ? valve_map : motor_map;
        auto it = lookup.find(*j.ecat_id);
        if (it != lookup.end()) {
            j.pipe = it->second;
            std::cout << "[PipeDiscovery] Joint '" << j.name
                      << "' ecat_id=" << *j.ecat_id
                      << " -> '" << it->second << "'\n";
        } else {
            std::cout << "[PipeDiscovery] Joint '" << j.name
                      << "' ecat_id=" << *j.ecat_id
                      << " not found on bus — will publish zero\n";
        }
    }

    // Fill ResolvedPipes struct
    ResolvedPipes out;
    for (const auto& p : all) {
        if (p.kind == PipeKind::Motor) 
            out.motor_pipes.push_back(p);
        else if (p.kind == PipeKind::Valve) 
            out.valve_pipes.push_back(p);
        else 
            out.sensor_pipes.push_back(p);
    }
    return out;
}
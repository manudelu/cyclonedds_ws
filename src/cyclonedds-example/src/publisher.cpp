#include <iostream>
#include <vector>
#include <map>
#include <fcntl.h>
#include <unistd.h>

#include "pipe_discovery.hpp"  
#include "robot_config.hpp"
#include "yaml_loader.hpp"
#include "pdo_utils.hpp"
#include "dds_publisher.hpp"

struct JointHandle {
    int fd;
    size_t index; 
    std::string name;
};

int main(int argc, char** argv) {
    std::string config_path = (argc > 1) ? argv[1] : "/home/mdelucchi-iit.local/cyclonedds_ws/src/cyclonedds-example/config/robot.yaml";
    LoadedConfig lc;
    try {
        lc = load_robot_config(config_path);
    } catch (const std::exception& e) {
        std::cerr << "Errore fatale config: " << e.what() << std::endl;
        return -1;
    }

    std::vector<std::string> joint_names;
    for (const auto& j : lc.cfg.joints) {
        joint_names.push_back(j.name);
    }

    JointStatePublisher js_pub;
    if (!js_pub.init(joint_names, lc.cfg.name, lc.cfg.domain)) {
        return -1;
    }

    std::vector<JointHandle> active_joints;
    for (size_t i = 0; i < lc.cfg.joints.size(); ++i) {
        auto& j = lc.cfg.joints[i];
        if (j.pipe.has_value()) {
            std::string full_path = XDDP_PREFIX + j.pipe.value();
            int fd = open(full_path.c_str(), O_RDONLY | O_NONBLOCK);
            if (fd >= 0) {
                active_joints.push_back({fd, i, j.name});
                std::cout << "[Main] Giunto collegato: " << j.name << " su " << j.pipe.value() << std::endl;
            }
        }
    }

    joint_state::rt_joint_state_msg js_msg(joint_names.size());
    motor::rt_motor motor_data; // Buffer per dati motori extra (temp, fault, etc)
    uint8_t buf[1024];

    std::cout << "Bridge avviato su Dominio " << lc.cfg.domain << ". In attesa di dati..." << std::endl;

    while (true) {
        bool updated = false;

        for (auto& handle : active_joints) {
            ssize_t n, last = -1;
            while ((n = read(handle.fd, buf, sizeof(buf))) > 0) {
                last = n;
            }

            if (last > 0) {
                double p, v, e;
                uint64_t ts;
                if (pdo_utils::parse_joint_frame(buf, last, p, v, e, ts, motor_data)) {
                    js_msg.header.timestamp_ns = ts;
                    js_msg.positions[handle.index] = p;
                    js_msg.velocities[handle.index] = v;
                    js_msg.efforts[handle.index] = e;
                    updated = true;
                }
            }
        }

        if (updated) {
            js_pub.publish(js_msg);
        }

    }

    for (auto& h : active_joints) close(h.fd);
    return 0;
}
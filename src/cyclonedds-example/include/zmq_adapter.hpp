#pragma once
#include "mw_adapter.hpp"
#include "motor.pb.h"
#include "imu.pb.h"
#include <zmq.h>
#include <iostream>
#include <array>
#include <cstring>

class ZmqAdapter : public MiddlewareAdapter {
public:

    ~ZmqAdapter() {
        if (js_pub_)  
            zmq_close(js_pub_);
        if (imu_pub_) 
            zmq_close(imu_pub_);
        if (cmd_pull_) 
            zmq_close(cmd_pull_);
        if (ctx_)     
            zmq_ctx_destroy(ctx_);
    }

    bool init(const std::string& robot_name) override {
        robot_name_ = robot_name;

        // Init ZMQ
        ctx_ = zmq_ctx_new();
        if (!ctx_) { 
            std::cerr << "[ZMQ] Failed to create context.\n"; 
            return false; 
        }

        // PUB sockets — outbound sensor data
        js_pub_  = make_socket(ZMQ_PUB, "tcp://*:5550");
        imu_pub_ = make_socket(ZMQ_PUB, "tcp://*:5551");

        // PULL socket — inbound commands
        cmd_pull_ = make_socket(ZMQ_PULL, "tcp://*:5552");
        int zero = 0;
        zmq_setsockopt(cmd_pull_, ZMQ_RCVTIMEO, &zero, sizeof(zero));

        std::cout << "[ZMQ] Sockets bound. Robot: " << robot_name_ << "\n"
                  << "  joint_states -> tcp://*:5550\n"
                  << "  imu          -> tcp://*:5551\n"
                  << "  cmd          <- tcp://*:5552\n";
        return true;
    }

    void publish_joint_state(const iit::advrf::MotorState& msg) override {
        publish_proto(js_pub_, msg);
    }

    void publish_imu(const iit::advrf::ImuState& msg) override {
        publish_proto(imu_pub_, msg);
    }

    bool take_joint_command(iit::advrf::MotorCmd& out) override {
        int bytes = zmq_recv(cmd_pull_, ser_buf_.data(), ser_buf_.size(), ZMQ_DONTWAIT);
        if (bytes <= 0) return false;
        return out.ParseFromArray(ser_buf_.data(), bytes);
    }

private:
    void* make_socket(int type, const char* endpoint) {
        void* sock = zmq_socket(ctx_, type);
        if (!sock) {
            std::cerr << "[ZMQ] Failed to create socket for " << endpoint << "\n";
            return nullptr;
        }
        if (zmq_bind(sock, endpoint) != 0) {
            std::cerr << "[ZMQ] Failed to bind " << endpoint << ": " << zmq_strerror(errno) << "\n";
            zmq_close(sock);
            return nullptr;
        }
        return sock;
    }

    template<typename Proto>
    void publish_proto(void* sock, const Proto& msg) {
        int bytes = static_cast<int>(msg.ByteSizeLong());
        if (bytes <= 0 || bytes > static_cast<int>(ser_buf_.size())) return;
        msg.SerializeToArray(ser_buf_.data(), bytes);
        zmq_send(sock, ser_buf_.data(), bytes, ZMQ_DONTWAIT);
    }

    void* ctx_ {nullptr};
    void* js_pub_ {nullptr};
    void* imu_pub_ {nullptr};
    void* cmd_pull_ {nullptr};

    std::string robot_name_;
    std::array<uint8_t, 512> ser_buf_{};
};
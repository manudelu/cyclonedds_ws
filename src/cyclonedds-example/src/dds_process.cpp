#include "shared_types.hpp"
#include "shm_utils.hpp"
#include "dds_publishers.hpp"
#include "motor.pb.h"
#include "imu.pb.h"

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <iostream>

#include <dds/dds.hpp>
#include "JointState.hpp"
#include "Imu.hpp"

uint32_t DOMAIN_ID {42};

static std::atomic<bool> g_stop{false};
void handle_sig(int) { g_stop = true; }

int main() {
    signal(SIGINT,  handle_sig);
    signal(SIGTERM, handle_sig);

    SharedMemoryClient shm(SHM_NAME, sizeof(SharedBridge));


    SharedBridge* bridge = shm.get<SharedBridge>();
    if (!shm.is_valid()) {
        std::cerr << "[DDS] Failed to open Shared Memory.\n";
        return 1;
    }
    DDSPublisherManager dds_manager(DOMAIN_ID, "spot");

    // Inbound (data received)
    iit::advrf::MotorState in_state;
    iit::advrf::ImuState in_imu;

    // Outbound (data sent)
    iit::advrf::MotorCmd out_cmd;
    out_cmd.mutable_motors()->Reserve(12);
    for (int i = 0; i < 12; ++i)
        out_cmd.add_motors();
    
    uint8_t ser_buf[PROTO_MAX_BYTES];
    ProtoSlot slot{};

    // Startup handshake - DDS signals readiness only after its setup
    bridge->dds_ready.store(true, std::memory_order_release);
    std::cout << "[DDS] Ready. Waiting for RT process...\n";
    while (!bridge->rt_ready.load(std::memory_order_acquire) && !g_stop)
        usleep(1000);

    std::cout << "[DDS] Bridge active.\n";

    const struct timespec dt{0, 500000}; // poll at 2 kHz

    while (!g_stop) {
        // Retrieve data from RT SHM and Publish JointState
        while (bridge->joint_state.try_pop(slot)) {
            if (slot.size == 0 || slot.size > PROTO_MAX_BYTES) 
                continue;

            if (in_state.ParseFromArray(slot.data, static_cast<int>(slot.size))) 
                dds_manager.publish_joint_state(in_state);
        }

        // Retrieve data from RT SHM and Publish Imu
        while (bridge->imu.try_pop(slot)) {
            if (slot.size == 0 || slot.size > PROTO_MAX_BYTES)   
                continue;

            if (in_imu.ParseFromArray(slot.data, static_cast<int>(slot.size))) 
                dds_manager.publish_imu(in_imu);
        }

        nanosleep(&dt, nullptr);
    }
    std::cout << "[DDS] Shutting down.\n";

    return 0;
}
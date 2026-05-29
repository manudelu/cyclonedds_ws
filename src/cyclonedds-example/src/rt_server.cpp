#include "shared_types.hpp"
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <iostream>

static std::atomic<bool> g_stop{false};
void handle_sig(int) { g_stop = true; }

int main() {
    signal(SIGINT,  handle_sig);
    signal(SIGTERM, handle_sig);

    // Lock all current and future memory pages — mandatory for hard RT
    mlockall(MCL_CURRENT | MCL_FUTURE);

    // Open existing shared memory object and map it into the caller's address space
    int fd = shm_open(SHM_NAME, O_RDWR, 0666);
    if (fd < 0) { 
        perror("shm_open (rt server)"); 
        return 1; 
    }

    void* ptr = mmap(nullptr, sizeof(SharedBridge),
                     PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (ptr == MAP_FAILED) { 
        perror("mmap (rt server)"); 
        return 1; 
    }

    SharedBridge* bridge = reinterpret_cast<SharedBridge*>(ptr);

    // Wait for DDS process to be ready
    std::cout << "[RT Server] Waiting for DDS process...\n";
    while (!bridge->dds_ready.load(std::memory_order_acquire) && !g_stop)
        usleep(1000);

    bridge->rt_server_ready.store(true, std::memory_order_release);
    std::cout << "[RT Server] Linked. Running driver hardware loop.\n";

    // Become RT
    struct sched_param param;
    param.sched_priority = 85;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    pthread_setname_np(pthread_self(), "RT_Server");

    // Pre-allocate all working state before entering the loop
    JointState state{};
    JointState cmd{};

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    const long long period_ns = 1'000'000LL;

    while (!g_stop.load(std::memory_order_relaxed)) {

        // Drain inbound — keep freshest
        JointState tmp{};
        while (bridge->client_to_server.try_pop(tmp))
            cmd = tmp;

        struct timespec now;
        clock_gettime(CLOCK_REALTIME, &now);
        state.sec = static_cast<int32_t>(now.tv_sec);
        state.nanosec = static_cast<uint32_t>(now.tv_nsec);

        for(int i = 0; i < 12; ++i) {
            state.position[i] = cmd.position[i];
            state.velocity[i] = cmd.velocity[i];
            state.effort[i]   = cmd.effort[i];
        }

        bridge->server_to_dds.try_push(state);

        next.tv_nsec += period_ns;
        if (next.tv_nsec >= 1'000'000'000L) {
            next.tv_sec  += 1;
            next.tv_nsec -= 1'000'000'000L;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    }

    std::cout << "[RT Server] Shutting down.\n";
    munmap(ptr, sizeof(SharedBridge));
    
    return 0;
}
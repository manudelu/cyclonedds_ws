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
        perror("shm_open (rt)"); 
        return 1; 
    }

    void* ptr = mmap(nullptr, sizeof(SharedBridge),
                     PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (ptr == MAP_FAILED) { 
        perror("mmap (rt)"); 
        return 1; 
    }

    SharedBridge* bridge = reinterpret_cast<SharedBridge*>(ptr);

    // Wait for DDS process to be ready
    std::cout << "[RT] Waiting for DDS process...\n";
    while (!bridge->dds_ready.load(std::memory_order_acquire) && !g_stop)
        usleep(1000);

    bridge->rt_ready.store(true, std::memory_order_release);
    std::cout << "[RT] Bridge active. Running control loop.\n";

    // Become RT
    struct sched_param param;
    param.sched_priority = 80;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    pthread_setname_np(pthread_self(), "RT_Control");

    // Pre-allocate all working state before entering the loop
    JointState state{};
    JointState cmd{};
    double pos = 0.0;
    bool incremental = true;

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    const long long period_ns = 1'000'000LL;

    while (!g_stop.load(std::memory_order_relaxed)) {

        // Drain inbound — keep freshest
        JointState tmp{};
        while (bridge->dds_to_rt.try_pop(tmp))
            state = tmp;

        // Control logic
        if (incremental) { 
            pos += 0.01; 
            if (pos > 0.3) 
                incremental = false; 
        }
        else { 
            pos -= 0.01; 
            if (pos < 0.0) 
                incremental = true;  
        }

        struct timespec now;
        clock_gettime(CLOCK_REALTIME, &now);
        cmd.sec = static_cast<int32_t>(now.tv_sec);
        cmd.nanosec = static_cast<uint32_t>(now.tv_nsec);
        cmd.position[0] = pos;

        bridge->rt_to_dds.try_push(cmd);

        next.tv_nsec += period_ns;
        if (next.tv_nsec >= 1'000'000'000L) {
            next.tv_sec  += 1;
            next.tv_nsec -= 1'000'000'000L;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    }

    std::cout << "[RT] Shutting down.\n";

    munmap(ptr, sizeof(SharedBridge));
    
    return 0;
}
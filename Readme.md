# CycloneDDS (Xenomai hard real-time)

Results
---------------

A real-time experiment was conducted using a fully separated interprocess architecture based on POSIX shared memory and lock-free single-producer single-consumer (SPSC) queues. The objective was to evaluate **mode switch** behavior.

Xenomai implements a dual-kernel architecture (hence, the CPU runs two kernel simultaneoulsy: the Linux kernel and the Xenomai real-time kernel, called Cobalt).

A mode switch is the transition of a thread from the real-time execution domain (Cobalt) to the Linux (non real-time) domain. This typically occurs when a real-time thread invokes a service that is not available in the real-time core, such as certain Linux system calls, I/O operations, or middleware paths that are not fully real-time compatible. When a mode switch happens, the thread temporarily leaves the deterministic real-time environment and is scheduled by the Linux kernel instead. This introduces additional latency and breaks real-time guarantees for the duration of the switch and the operations executed in the Linux domain. In practice, mode switches are important because they directly indicate violations of full real-time execution, and their frequency can be used as a metric to evaluate how “real-time safe” a software stack actually is under Xenomai/Cobalt.

The system was divided into two independent processes, a DDS/Linux process and a real-time Xenomai control process. The two processes communicate through a shared-memory bridge created with POSIX shared memory (shm_open, mmap) and custom lock-free SPSC queues. The shared data structures contain only fixed-size POD types and preallocated buffers. No dynamic allocation occurs during runtime inside the real-time process.

In this configuration, no mode switches were observed in the Xenomai real-time process. All Linux syscalls, DDS middleware execution, socket operations, polling, and dynamic allocations are fully isolated inside the non real-time DDS process. The RT process never invokes Linux networking or DDS APIs directly and therefore remains entirely inside the Xenomai/Cobalt domain during execution.

Installation 
--------------------

Clone the repository and create the environment:

```bash
git clone https://github.com/manudelu/cyclonedds_ws.git --recurse-submodules
cd cyclonedds_ws

mkdir -p build install
mkdir -p build/cyclonedds
mkdir -p build/cyclonedds-cxx
mkdir -p build/cyclonedds-example
```

Build and install CycloneDDS:

```bash
cd ~/cyclonedds_ws/build/cyclonedds
cmake ../../src/cyclonedds \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=~/cyclonedds_ws/install
make -j$(nproc) install
```

Build and install CycloneDDS C++ API:

```bash
cd ~/cyclonedds_ws/build/cyclonedds-cxx
cmake ../../src/cyclonedds-cxx \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH=~/cyclonedds_ws/install \
    -DCMAKE_INSTALL_PREFIX=~/cyclonedds_ws/install
make -j$(nproc) install
```

Build the repository:

```bash
cd ~/cyclonedds_ws/build/cyclonedds-example
cmake ../../src/cyclonedds-example \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH=~/cyclonedds_ws/install
make -j$(nproc)
```

Running
---------------

Terminal A (embedded PC) - check for mode switches (MSW):

```bash
watch -n 0.1 cat /proc/xenomai/sched/stat
```

Terminal B (embedded PC):

```bash
cd ~/cyclonedds_ws/build/cyclonedds-example
./dds_process
```

Terminal C (embedded PC):

```bash
cd ~/cyclonedds_ws/build/cyclonedds-example
./rt_process
```

Terminal D (personal PC):

```bash
export ROS_DOMAIN_ID=42
ros2 topic echo /advrf/spot/joint_states
```


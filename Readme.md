# CycloneDDS (Xenomai hard real-time)

Results
---------------

A real-time experiment was conducted using a split-thread architecture designed to isolate all non real-time DDS operations from the real-time control loop. The objective was to evaluate **mode switch** behavior.

Xenomai implements a dual-kernel architecture (hence, the CPU runs two kernel simultaneoulsy: the Linux kernel and the Xenomai real-time kernel, called Cobalt).

A mode switch is the transition of a thread from the real-time execution domain (Cobalt) to the Linux (non real-time) domain. This typically occurs when a real-time thread invokes a service that is not available in the real-time core, such as certain Linux system calls, I/O operations, or middleware paths that are not fully real-time compatible. When a mode switch happens, the thread temporarily leaves the deterministic real-time environment and is scheduled by the Linux kernel instead. This introduces additional latency and breaks real-time guarantees for the duration of the switch and the operations executed in the Linux domain. In practice, mode switches are important because they directly indicate violations of full real-time execution, and their frequency can be used as a metric to evaluate how “real-time safe” a software stack actually is under Xenomai/Cobalt.

The application was divided into two threads:

* A non real-time DDS thread (SCHED_OTHER) responsible for DDS publish/subscribe operations (recvmsg() / sendmsg() socket syscalls), CycloneDDS memory allocations and middleware handling, DDS polling and serialization.
* A real-time control thread (SCHED_FIFO, priority 80) responsible only for reading the latest joint state sample through a lock-free SPSC queue and computing the control command pushing commands to an outbound lock-free queue.

Communication between the two threads was implemented using *boost::lockfree::spsc_queue*.The real-time thread performs no DDS calls, no socket I/O, and no dynamic memory allocations during execution. All middleware interaction is delegated to the Linux DDS thread.

This architecture significantly reduces the interaction between the Xenomai/Cobalt domain and the Linux kernel. Since the RT thread never invokes Linux-only services or DDS middleware APIs directly, no mode switches are observed in the control loop during execution. The DDS thread still performs Linux syscalls and executes entirely in the Linux domain, but these operations are isolated from the deterministic real-time task.

This separation allows deterministic control execution with no mode switches observed while still enabling DDS-based communication with external ROS 2 systems.

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
./publisher
```

Terminal C (personal PC):

```bash
export ROS_DOMAIN_ID=42
ros2 topic echo /advrf/spot/joint_states
```


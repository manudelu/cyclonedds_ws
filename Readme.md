# CycloneDDS (Xenomai hard real-time)

Results
---------------

A real-time experiment was conducted using a CycloneDDS-based publisher and subscriber exchanging joint_state messages at 1 ms period. The objective was to evaluate **mode switch** behavior under different CycloneDDS configurations, with and without shared memory (Iceoryx).

Xenomai implements a dual-kernel architecture (hence, the CPU runs two kernel simultaneoulsy: the Linux kernel and the Xenomai real-time kernel, called Cobalt).

A mode switch is the transition of a thread from the real-time execution domain (Cobalt) to the Linux (non real-time) domain. This typically occurs when a real-time thread invokes a service that is not available in the real-time core, such as certain Linux system calls, I/O operations, or middleware paths that are not fully real-time compatible. When a mode switch happens, the thread temporarily leaves the deterministic real-time environment and is scheduled by the Linux kernel instead. This introduces additional latency and breaks real-time guarantees for the duration of the switch and the operations executed in the Linux domain. In practice, mode switches are important because they directly indicate violations of full real-time execution, and their frequency can be used as a metric to evaluate how “real-time safe” a software stack actually is under Xenomai/Cobalt.

* **Without Shared Memory:** Mode switches were observed when using ros2 topic echo on the subscriber side. The real-time publisher/subscriber pair remained mostly stable in isolation. The mode switches appear to be triggered by the non-real-time nature of the ROS 2 CLI tool (ros2 topic echo), which introduces scheduling transitions outside the real-time context.
* **With Shared Memory (Iceoryx enabled):** Mode switches were observed even between real-time publisher and subscriber threads. This suggests that enabling shared memory introduces additional real-time constraints or synchronization paths that affect scheduling behavior.

**Additional Observation (Multi-Machine Communication)**
* Inter-machine communication remains unaffected by shared memory configuration.
* DDS discovery and data exchange between machines continue to operate over UDP using the configured DDS Domain ID.
* Shared memory is strictly used for intra-node communication and does not interfere with network-based transport.

Installation (Without Shared Memory)
-------------------------------------

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

Terminal A (embedded PC):

```bash
cd ~/cyclonedds_ws/build/cyclonedds-example
./publisher
```

Terminal A (personal PC):

```bash
export ROS_DOMAIN_ID=42
ros2 topic echo /advrf/spot/joint_states
```

Installation (With Shared Memory / Iceoryx)
---------------------------------------------

Clone the repository and create the environment:

```bash
git clone https://github.com/manudelu/cyclonedds_ws.git --recurse-submodules
cd cyclonedds_ws

mkdir -p build install
mkdir -p build/cyclonedds
mkdir -p build/cyclonedds-cxx
mkdir -p build/cyclonedds-example
mkdir -p build/iceoryx
```

Install the prerequisite packages:

```bash
sudo apt update
sudo apt install cmake libacl1-dev libncurses5-dev pkg-config maven
```

Build and install Iceoryx:

```bash
cd ~/cyclonedds_ws/build/iceoryx
cmake ../../src/iceoryx/iceoryx_meta \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=~/cyclonedds_ws/install \
    -DROUDI_ENVIRONMENT=on \
    -DBUILD_SHARED_LIBS=ON
make -j$(nproc) install
```

Build and install CycloneDDS:

```bash
cd ~/cyclonedds_ws/build/cyclonedds
cmake ../../src/cyclonedds \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=~/cyclonedds_ws/install \
    -DCMAKE_PREFIX_PATH=~/cyclonedds_ws/install \
    -DENABLE_ICEORYX=ON
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
cmake ../../cyclonedds-example \
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
./cyclonedds_ws/build/iceoryx/iox-roudi -c ~/cyclonedds_ws/src/cyclonedds-example/iox_config.toml
```

Terminal C (embedded PC):

```bash
export LD_LIBRARY_PATH=~/cyclonedds_ws/install/lib/${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
export CYCLONEDDS_URI=file:///~/cyclonedds_ws/src/cyclonedds-example/cyclonedds.xml
cd ~/cyclonedds_ws/build/cyclonedds-example
./publisher
```

Terminal D (personal PC):

```bash
export ROS_DOMAIN_ID=42
ros2 topic echo /advrf/spot/joint_states
```
# CycloneDDS (Xenomai hard real-time)

How Does it Work
--------------------

Handshake

* Starts the DDS process: Initializes the Shared Memory with shm_open() (---> /dev/shm/spot_rt_bridge), sets atomic flag dds_ready to true and starts a while waiting for the flags of the other two processes (real-time client and server) to become true.
* The Real-Time Client and the Real-Time Server are started: They both open the Shared Memory object, then they enter in a blocking while loop checking for dds_ready flag. When DDS is ready, the Client sets the rt_client_ready atomic flag to true, and the Server sets the rt_server_ready atomic flag to true.
* The entire pipeline unlocks.

Command Flow: ROS2 to Motors

* In ROS2 side start the publisher on topic /advrf/spot/joint_cmd (e.g., ros2 topic pub -1 /advrf/spot/joint_cmd sensor_msgs/msg/JointState "{position: [1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0]}") . 
* In DDS Process side, the CycloneDDS middleware extract the data from the .idl type, copies them in a struct (ros2-like JointState) and pushes the data to the client (through dds_to_client SPSC queue).
* In the Real-Time Client side (1ms), every cycles it pops the data (from dds_to_client SPSC queue). If more than one data arrives, the while cycle drains the queue keeping only the freshest sample.
* The client can then process this data and then push onto the second queue (client_to_server SPSC queue).
* The Real-Time Server side (1ms, slightly higher priority) pops the data from the client (client_to_server SPSC queue) and takes the command and then sends it, for instance, to the real motor drives.

Telemetry Flow: Motors to ROS2

* The server reads, for instance, the physical encoders (in the example it just simulates the data just received from the client), sets the timestamp and inserts data into the third queue (server_to_dds SPSC queue).
* Then, the DDS process pops the data (from server_to_dds SPSC queue) and publish to ROS2 topic /advrf/spot/joint_state by inserting the popped data into the JointState struct.
* Finally in ROS2, if you check with ros2 topic echo, you can see the data updated.

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


# CycloneDDS

**Note:** Check branches for various implementation, including: 
* Shared Memory usage test
* Hard real-time (Xenomai) test 
* SPSC Queues

Installation
---------------

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
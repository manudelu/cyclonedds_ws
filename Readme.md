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
mkdir build/ install/
cd build
mkdir cyclonedds cyclonedds-cxx cyclonedds-example
```

Build and install CycloneDDS:

```bash
cd ~/cyclonedds_ws/build/cyclonedds
cmake -DCMAKE_INSTALL_PREFIX=/home/<username>/cyclonedds/install ..
make -j$(nproc) install
```

Build and install CycloneDDS C++ API:

```bash
cd ~/cyclonedds_ws/build/cyclonedds-cxx
cmake -DCMAKE_PREFIX_PATH=/home/<username>/cyclonedds/install -DCMAKE_INSTALL_PREFIX=/home/<username>/cyclonedds/install ..
make -j$(nproc) install
```

Build the repository:

```bash
cd ~/cyclonedds_ws/build/cyclonedds-example
cmake -DCMAKE_PREFIX_PATH=/home/<username>/cyclonedds/install -DCMAKE_INSTALL_PREFIX=/home/<username>/cyclonedds/install ..
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
ros2 topic echo /advrf/spot/joint_states
```
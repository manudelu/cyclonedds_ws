cmake_minimum_required(VERSION 3.16)
project(cyclone_jointstate)

set(CMAKE_CXX_STANDARD 17)

find_package(CycloneDDS REQUIRED)
find_package(CycloneDDS-CXX REQUIRED)

set(IDL_FILES 
    "idl/Time.idl" 
    "idl/Header.idl" 
    "idl/JointState.idl"
)

idlcxx_generate(
  TARGET sensor_msgs_types
  FILES ${IDL_FILES}
)

add_executable(publisher src/publisher.cpp)
target_link_libraries(publisher
  CycloneDDS-CXX::ddscxx
  sensor_msgs_types
)

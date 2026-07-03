cmake_minimum_required(VERSION 3.16)
project(cyclone_jointstate)

set(CMAKE_CXX_STANDARD 17)

list(PREPEND CMAKE_PREFIX_PATH /usr/local)

# Find xeno-config (/usr/xenomai/bin/xeno-config)
find_program(XENO_CONFIG xeno-config REQUIRED)

# Get compile and link flags from xeno-config
execute_process(
    COMMAND ${XENO_CONFIG} --skin=posix --cflags
    OUTPUT_VARIABLE XENO_CFLAGS
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

execute_process(
    COMMAND ${XENO_CONFIG} --skin=posix --ldflags
    OUTPUT_VARIABLE XENO_LDFLAGS
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Convert to CMake lists
separate_arguments(XENO_CFLAGS)
separate_arguments(XENO_LDFLAGS)

find_package(CycloneDDS REQUIRED)
find_package(CycloneDDS-CXX REQUIRED)
find_package(Protobuf REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

set(IDL_FILES 
    "idl/Time.idl" 
    "idl/Header.idl" 
    "idl/JointState.idl"
    "idl/Quaternion.idl"
    "idl/Vector3.idl"
    "idl/Imu.idl"
    "idl/Duration.idl"
    "idl/JointTrajectoryPoint.idl"
    "idl/JointTrajectory.idl"
)

idlcxx_generate(
  TARGET msgs_types
  FILES ${IDL_FILES}
)

set(PROTO_FILES 
    "protobuf/motor.proto"
    "protobuf/imu.proto"
) 
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

add_executable(bridge_main 
    src/bridge_main.cpp
    ${PROTO_SRCS}
)
target_include_directories(bridge_main PRIVATE 
    ${CMAKE_CURRENT_BINARY_DIR}
    include/
) 
target_link_libraries(bridge_main
    CycloneDDS-CXX::ddscxx
    msgs_types
    ${ZMQ_LIBRARIES} 
    pthread rt
    protobuf::libprotobuf
)

add_executable(rt_process 
    src/rt_process.cpp
    ${PROTO_SRCS}
)
target_compile_options(rt_process PRIVATE ${XENO_CFLAGS})
target_include_directories(rt_process PRIVATE 
    ${CMAKE_CURRENT_BINARY_DIR}
    include/
) 
target_link_libraries(rt_process
    ${XENO_LDFLAGS}
    pthread rt
    protobuf::libprotobuf
)
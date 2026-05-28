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

set(IDL_FILES 
    "idl/Time.idl" 
    "idl/Header.idl" 
    "idl/JointState.idl"
)

idlcxx_generate(
  TARGET sensor_msgs_types
  FILES ${IDL_FILES}
)

add_executable(dds_process src/dds_process.cpp)
target_link_libraries(dds_process
    CycloneDDS-CXX::ddscxx
    sensor_msgs_types
    pthread rt
)

add_executable(rt_process src/rt_process.cpp)
target_compile_options(rt_process PRIVATE ${XENO_CFLAGS})
target_link_libraries(rt_process
    ${XENO_LDFLAGS}
    pthread rt
)
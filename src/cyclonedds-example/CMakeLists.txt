cmake_minimum_required(VERSION 3.16)
project(cyclone_jointstate LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)

if(NOT TARGET CycloneDDS-CXX::ddscxx)
  find_package(CycloneDDS-CXX REQUIRED)
endif()


set(EC_MASTER_WS "" CACHE PATH "")

string(REPLACE ":" ";" CMAKE_PREFIX_PATH_LIST "$ENV{CMAKE_PREFIX_PATH}")

if(NOT EC_MASTER_WS)
    foreach(prefix ${CMAKE_PREFIX_PATH_LIST})
        get_filename_component(ws_root "${prefix}" DIRECTORY)
        foreach(subdir "build/ecat_master/protobuf" "build/ecat-master-advr/protobuf")
            if(EXISTS "${ws_root}/${subdir}/libecat_master_pb.a")
                set(EC_MASTER_WS "${ws_root}/${subdir}")
                break()
            endif()
        endforeach()
        if(EC_MASTER_WS)
            break()
        endif()
    endforeach()
endif()

if(NOT EC_MASTER_WS OR NOT EXISTS "${EC_MASTER_WS}/libecat_master_pb.a")
    message(FATAL_ERROR "libecat_master_pb.a not found.\n")
endif()

message(STATUS "Protobuf dir: ${EC_MASTER_WS}")

find_package(Protobuf REQUIRED)
find_package(Xenomai QUIET)

set(IDL_FILES
    "idl/Time.idl"
    "idl/Header.idl"
    "idl/JointState.idl"
    "idl/Vector3.idl"
    "idl/Quaternion.idl"
    "idl/Wrench.idl"
    "idl/Motor.idl"
    "idl/Valve.idl"
    "idl/Pump.idl"
    "idl/Imu.idl"
    "idl/ForceTorque.idl"
    "idl/PowerBoard.idl"
)

idlcxx_generate(
    TARGET dds_types
    FILES ${IDL_FILES}
    WARNINGS no-implicit-extensibility
)

add_executable(publisher
    src/publisher.cpp
)

target_include_directories(publisher PRIVATE 
    include                     
    ${CMAKE_CURRENT_SOURCE_DIR}/include 
    ${CMAKE_CURRENT_BINARY_DIR}  
    ${EC_MASTER_WS}
    ${Protobuf_INCLUDE_DIRS} 
)

if(XENOMAI_FOUND)
    message(STATUS "Building with Xenomai support")
    target_compile_options(publisher PRIVATE ${XENOMAI_CFLAGS})
    target_link_libraries(publisher PRIVATE Xenomai::POSIX)
else()
    message(STATUS "Building without Xenomai — using /tmp/nrt_pipes")
endif()

target_link_libraries(publisher
    PRIVATE
    CycloneDDS-CXX::ddscxx
    dds_types
    ${EC_MASTER_WS}/libecat_master_pb.a
    ${Protobuf_LIBRARIES}
    yaml-cpp
    stdc++fs
)
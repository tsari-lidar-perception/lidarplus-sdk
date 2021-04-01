cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(lidarplus_sdk)

set(CMAKE_C_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")

option(BUILD_ZLGCAN "Build ZLG USB CAN-4E-U sample" ON)

add_subdirectory(dependency/network)

# sample 
add_subdirectory(sample/udp_recv_sample/cpp)
add_subdirectory(sample/can_recv_sample)
add_subdirectory(sample/custom_lidar_sample)
add_subdirectory(sample/ins_interface_sample)

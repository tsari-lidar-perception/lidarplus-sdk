cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(ecn_sdk)

set(CMAKE_C_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")

add_subdirectory(network)
add_subdirectory(cpp_sample)

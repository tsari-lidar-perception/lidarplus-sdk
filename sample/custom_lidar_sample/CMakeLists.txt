
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(custom_lidar_sample)

set(CMAKE_C_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")

include_directories (/usr/local/include)
link_directories(/usr/local/lib)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_executable(send_custom_lidar
               send_custom_lidar.cpp
)

target_link_libraries(send_custom_lidar
                      network_driver
)

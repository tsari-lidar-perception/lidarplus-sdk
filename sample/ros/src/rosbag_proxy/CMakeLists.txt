cmake_minimum_required(VERSION 2.8.3)
project(rosbag_proxy)

add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(catkin REQUIRED COMPONENTS roscpp pcl_ros)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
)

include_directories(
   ${catkin_INCLUDE_DIRS}
)

add_subdirectory(network)

add_executable(rosbag_proxy rosbag_proxy.cpp)
add_dependencies(rosbag_proxy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rosbag_proxy ${catkin_LIBRARIES} network_driver)
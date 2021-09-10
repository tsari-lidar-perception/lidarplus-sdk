cmake_minimum_required(VERSION 3.10)

project(rosbag_writter LANGUAGES CXX)
 
add_subdirectory(rosbag_writter)
 
set(CMAKE_CXX_STANDARD 11)

include_directories(/usr/include)
include_directories (/usr/include/pcl-1.8)
include_directories(/usr/include/eigen3)
link_directories(/usr/local/lib)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    /usr/include/python3.6m/
    rosbag_writter/include
    getpkldata/include
    getpkldata/pybind11/include
  )

add_executable (main src/main.cpp getpkldata/src/getpkldata.cpp)
target_link_libraries (main /usr/lib/x86_64-linux-gnu/libpython3.6m.so rosbag_writter ${OpenCV_LIBS})
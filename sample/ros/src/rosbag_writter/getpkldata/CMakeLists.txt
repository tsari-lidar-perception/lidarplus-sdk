cmake_minimum_required(VERSION 3.10)

project(rosbag_writter LANGUAGES CXX)
 
set(CMAKE_CXX_STANDARD 11)

# find_package(pcl_ros)

include_directories(/usr/include)
include_directories (/usr/include/pcl-1.8)
include_directories(/usr/include/eigen3)
link_directories(/usr/local/lib)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/pybind11/include)

include_directories(
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    /usr/include/python3.6m/
    pybind11/include
  )

add_library(getpkldata SHARED
  src/getpkldata.cpp
  )

target_link_libraries(getpkldata
      /usr/lib/x86_64-linux-gnu/libpython3.6m.so
      ${OpenCV_LIBS}
    )

add_executable (main1 src/getpkldata.cpp)
target_link_libraries (main1 /usr/lib/x86_64-linux-gnu/libpython3.6m.so ${OpenCV_LIBS})
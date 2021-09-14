cmake_minimum_required(VERSION 3.10)

project(rosbag_writter LANGUAGES CXX)
 
add_subdirectory(rosbag_writter)
 
set(CMAKE_CXX_STANDARD 11)

include_directories(/usr/include)
link_directories(/usr/local/lib)

find_package(PythonLibs 3 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIR})

find_package(PCL 1.8 REQUIRED COMPONENTS common)

include_directories(SYSTEM ${PCL_INCLUDE_DIRS})

include_directories(
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    rosbag_writter/include
    get_pkldata/include
    get_pkldata/pybind11/include
  )

add_executable (main src/main.cpp get_pkldata/src/getpkldata.cpp)
target_link_libraries (main  rosbag_writter ${PYTHON_LIBRARY})
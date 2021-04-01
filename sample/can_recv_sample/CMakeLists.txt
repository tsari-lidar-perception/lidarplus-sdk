
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(cpp_sample)

set(CMAKE_C_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")

include_directories (/usr/local/include)
link_directories(/usr/local/lib)

find_package(Protobuf REQUIRED)

include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ../../dependency/proto/detection.proto)

add_library(parse_can SHARED
            parse_can.cpp
)

if(${BUILD_ZLGCAN} STREQUAL "ON")
add_executable(recv_detection_zlgcan
               recv_detection_zlgcan.cpp
               ${PROTO_SRCS}
               ${PROTO_HDRS}
)

target_link_libraries(recv_detection_zlgcan
                      parse_can
                      ${PROTOBUF_LIBRARIES}
                      usbcan-4e
)
endif()
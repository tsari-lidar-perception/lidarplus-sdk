
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

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ../proto/detection.proto)

add_executable(recev_detection
               recev_detection.cpp
               ${PROTO_SRCS}
               ${PROTO_HDRS}
)

target_link_libraries(recev_detection
                      network_driver
                      ${PROTOBUF_LIBRARIES}
)

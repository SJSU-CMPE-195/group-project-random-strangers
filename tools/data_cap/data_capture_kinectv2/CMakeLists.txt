cmake_minimum_required(VERSION 3.16)
project(KinectRecorder)

set(CMAKE_CXX_STANDARD 23)

if(NOT DEFINED KINECTSDK20_ROOT)
    message(FATAL_ERROR "Please set KINECTSDK20_ROOT to the Kinect SDK v2.0_1409 directory")
endif()

include_directories(${KINECTSDK20_ROOT}/inc)
link_directories(${KINECTSDK20_ROOT}/Lib/x64)

add_executable(writer src/writer.cpp)

target_link_libraries(writer
    Kinect20
)

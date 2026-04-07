cmake_minimum_required(VERSION 3.16)
project(MocapRecorder)

set(CMAKE_CXX_STANDARD 20)

if(NOT DEFINED NATNET_ROOT)
    message(FATAL_ERROR "Please set NATNET_ROOT to NatNet SDK directory")
endif()

include_directories(${NATNET_ROOT}/include)

link_directories(${NATNET_ROOT}/lib/x64)

add_executable(recorder src/data_capture.cpp)

target_link_libraries(recorder
    NatNetLib
    Ws2_32
)

add_custom_command(TARGET recorder POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${NATNET_ROOT}/lib/x64/NatNetLib.dll"
        $<TARGET_FILE_DIR:recorder>
)
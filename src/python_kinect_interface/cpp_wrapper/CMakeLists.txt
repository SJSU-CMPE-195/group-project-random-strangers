cmake_minimum_required(VERSION 3.16)

project(kinect_wrapper LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_library(KINECT20_LIBRARY
    NAMES Kinect20
    PATHS
        ENV PATH
        "$ENV{KINECTSDK20_ROOT}/Lib/x64"
        "C:/Program Files/Microsoft SDKs/Kinect/v2.0_1409/Lib/x64"
)

if(KINECT20_LIBRARY)
    get_filename_component(_kinect_lib_dir "${KINECT20_LIBRARY}" DIRECTORY)
    get_filename_component(_kinect_sdk_lib_dir "${_kinect_lib_dir}" DIRECTORY)
    get_filename_component(KINECTSDK20_ROOT "${_kinect_sdk_lib_dir}" DIRECTORY)
elseif(NOT DEFINED KINECTSDK20_ROOT)
    message(FATAL_ERROR "Could not find Kinect20.lib on PATH. Set KINECTSDK20_ROOT to the Kinect SDK v2.0_1409 directory.")
else()
    set(KINECT20_LIBRARY "${KINECTSDK20_ROOT}/Lib/x64/Kinect20.lib")
endif()

if(NOT EXISTS "${KINECTSDK20_ROOT}/inc/Kinect.h")
    message(FATAL_ERROR "Could not find Kinect.h under ${KINECTSDK20_ROOT}. Set KINECTSDK20_ROOT to the Kinect SDK v2.0_1409 directory.")
endif()

add_library(kinect_wrapper SHARED
    kinect_wrapper.cpp
)

target_include_directories(kinect_wrapper PRIVATE
    ${KINECTSDK20_ROOT}/inc
)

target_link_directories(kinect_wrapper PRIVATE
    ${KINECTSDK20_ROOT}/Lib/x64
)

target_link_libraries(kinect_wrapper PRIVATE
    ${KINECT20_LIBRARY}
)

set_target_properties(kinect_wrapper PROPERTIES
    OUTPUT_NAME "kinect_wrapper"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    PDB_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
)

foreach(_config Debug Release RelWithDebInfo MinSizeRel)
    string(TOUPPER "${_config}" _config_upper)
    set_target_properties(kinect_wrapper PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY_${_config_upper} "${CMAKE_CURRENT_SOURCE_DIR}"
        LIBRARY_OUTPUT_DIRECTORY_${_config_upper} "${CMAKE_CURRENT_BINARY_DIR}"
        ARCHIVE_OUTPUT_DIRECTORY_${_config_upper} "${CMAKE_CURRENT_BINARY_DIR}"
        PDB_OUTPUT_DIRECTORY_${_config_upper} "${CMAKE_CURRENT_BINARY_DIR}"
    )
endforeach()
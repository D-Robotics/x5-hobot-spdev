cmake_minimum_required(VERSION 2.8)

set(CMAKE_C_COMPILER $ENV{CROSS_COMPILE}gcc)
set(CMAKE_CXX_COMPILER $ENV{CROSS_COMPILE}g++)

project(hobot_spdev)

set(SOC "X5")
add_definitions(-D${SOC}="${SOC}")

option(RELEASE_LIB "build version of release" ON)
message("config types: ${CMAKE_CONFIGURATION_TYPES}")

if (${RELEASE_LIB})
    set(CMAKE_BUILD_TYPE Release)
    message(STATUS "BUILD TYPE:Release")
else ()
    set(CMAKE_BUILD_TYPE Debug)
    set(HOBOT_BUILD_DEBUG "TRUE")
    message(STATUS "BUILD TYPE:Debug")
endif ()

list(APPEND CMAKE_C_FLAGS "-Wall -Wno-unknown-pragmas -fPIC -fexceptions ")
list(APPEND CMAKE_C_FLAGS "--sysroot=$ENV{SYSROOT_DIR} ")
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    list(APPEND CMAKE_C_FLAGS "-march=armv8-a -mcpu=cortex-a53 -O3 ")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC ")
else ()
    list(APPEND CMAKE_C_FLAGS "-march=armv8-a -mcpu=cortex-a53 -g -O0 -fPIC -ggdb ")
endif()

list(APPEND CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -std=c++14 -fno-permissive ")
string(REGEX REPLACE ";" " " CMAKE_C_FLAGS ${CMAKE_C_FLAGS})
string(REGEX REPLACE ";" " " CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
set(SPDEV_OUTPUT_ROOT $ENV{BUILD_OUTPUT_PATH}/output)
set(SPDEV_BUILD_ROOT $ENV{BUILD_OUTPUT_PATH}/build)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vp_sensors
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vp_wrap/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vpp_interface
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/hobot-dnn/x5/usr/include
    $ENV{SYSROOT_DIR}/usr/include/aarch64-linux-gnu
    $ENV{SYSROOT_DIR}/usr/include/drm
)

link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/aarch64-linux-gnu
    ${CMAKE_CURRENT_SOURCE_DIR}/hobot-dnn/x5/usr/lib
    $ENV{SYSROOT_DIR}/usr/lib/aarch64-linux-gnu
    $ENV{SYSROOT_DIR}/usr/lib/aarch64-none-linux-gnu
)

set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/output/${PROJECT_NAME})

# link library
set(BASE_LIBRARIES pthread dl rt)

if ("${SOC}" STREQUAL "X5")
    set(HB_SDK_LIB Nano2D Nano2Dutil cam vpf hbmem multimedia avformat avcodec avutil
    swresample ffmedia gdcbin cjson alog
    dnn cnn_intf hbrt_bayes_aarch64
    m EGL GL X11
    ssl crypto z dl rt pthread python3.10)
endif()

# add spdev include path
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vp_wrap/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/python/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/clang
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cpp_postprocess
)

install(FILES ${SPDEV_BUILD_ROOT}/src/libhbspdev.so
    DESTINATION ${SPDEV_OUTPUT_ROOT}/)

add_subdirectory(src)

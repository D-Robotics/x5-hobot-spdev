cmake_minimum_required(VERSION 2.8)

# projetc name
project(hobot_spdev)

# make libhbspdev.so
set(HBSPDEV_NAME hbspdev)
file(GLOB HBSPDEV_INC
    "utils/*.h"
    "vpp_interface/*.h"
    "vp_sensors/*.h"
    "vp_wrap/include/*.h"
    )

file(GLOB HBSPDEV_SRC
    "utils/*.c"
    "vpp_interface/*.cpp"
    "vp_wrap/src/*.cpp"
    "vp_wrap/src/*.c"
    "vp_sensors/*.c"
    "vp_sensors/*/*.c"
    )

set(SOURCE_FILES ${HBSPDEV_SRC} ${HBSPDEV_INC})
add_library(${HBSPDEV_NAME} SHARED ${SOURCE_FILES})
target_link_libraries(${HBSPDEV_NAME} ${HB_SDK_LIB} ${BASE_LIBRARIES})

include_directories(
    $ENV{SYSROOT_DIR}/usr/include/aarch64-linux-gnu/python3.12
    $ENV{SYSROOT_DIR}/usr/include/python3.12
    $ENV{SYSROOT_DIR}/usr/include
    $ENV{SYSROOT_DIR}/usr/lib/python3/dist-packages/numpy/core/include
)

# make libsrcampy.so
set(SRCAMPY_NAME srcampy)

add_library(${SRCAMPY_NAME} SHARED "python/vpp_python.cpp")
target_link_libraries(${SRCAMPY_NAME} ${HBSPDEV_NAME})

# make libdnnpy.so
set(DNNPY_NAME dnnpy)
add_library(${DNNPY_NAME} SHARED "python/dnn_python.cpp")
target_link_libraries(${DNNPY_NAME} ${HBSPDEV_NAME} dnn)

# make libspcdev.so
set(SPCDEV_NAME spcdev)
file(GLOB SPCDEV_INC
    "clang/*.h"
    )

# 源文件路径信息
file(GLOB SPCDEV_SRC
    "clang/*.cpp"
)

set(SOURCE_FILES ${SPCDEV_SRC} ${SPCDEV_INC})
add_library(${SPCDEV_NAME} SHARED ${SOURCE_FILES})

target_link_libraries(${SPCDEV_NAME} dnn ${HBSPDEV_NAME} )

# make libpostprocess.so
set(POSTPROCESS_NAME postprocess)
file(GLOB POSTPROCESS_INC
    "cpp_postprocess/*.h"
)

# 源文件路径信息
file(GLOB POSTPROCESS_SRC
    "cpp_postprocess/*.cpp"
)

set(SOURCE_FILES ${POSTPROCESS_SRC} ${POSTPROCESS_INC})
add_library(${POSTPROCESS_NAME} SHARED ${SOURCE_FILES})

target_link_libraries(${POSTPROCESS_NAME} dnn ${HBSPDEV_NAME})

install(TARGETS ${HBSPDEV_NAME} DESTINATION ${SPDEV_OUTPUT_ROOT})
install(TARGETS ${SRCAMPY_NAME} DESTINATION ${SPDEV_OUTPUT_ROOT})
install(TARGETS ${DNNPY_NAME} DESTINATION ${SPDEV_OUTPUT_ROOT})
install(TARGETS ${SPCDEV_NAME} DESTINATION ${SPDEV_OUTPUT_ROOT})
install(TARGETS ${POSTPROCESS_NAME} DESTINATION ${SPDEV_OUTPUT_ROOT})

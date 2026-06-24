set(HOBOT_DNN_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../hobot-dnn/x5)

set(HBM_RUNTIME_SOURCES
    src/HB_HBMRuntime.cc
    src/HB_RuntimeUtils.cc
    src/HBMRuntimeBinding.cc
)

add_library(HB_HBMRuntime MODULE ${HBM_RUNTIME_SOURCES})

target_include_directories(HB_HBMRuntime PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    $ENV{SYSROOT_DIR}/usr/include/pybind11
    $ENV{SYSROOT_DIR}/usr/include/python3.12
    $ENV{SYSROOT_DIR}/usr/include/aarch64-linux-gnu/python3.12
    $ENV{SYSROOT_DIR}/usr/lib/python3/dist-packages/numpy/core/include
    ${HOBOT_DNN_ROOT}/usr/include/dnn
)

set_target_properties(HB_HBMRuntime PROPERTIES
    PREFIX ""
    OUTPUT_NAME "HB_HBMRuntime"
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

target_compile_options(HB_HBMRuntime PRIVATE -std=c++17)

link_directories(${HOBOT_DNN_ROOT}/usr/lib)
target_link_libraries(HB_HBMRuntime dnn python3.12)

install(TARGETS HB_HBMRuntime DESTINATION ${SPDEV_OUTPUT_ROOT}/hbm_runtime)

cmake_minimum_required(VERSION 3.10)
project(umd)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(yaml-cpp REQUIRED)

include_directories(common)
include_directories(.)

# As of 3/2024, UMD lacks support for different architectures.
#include_directories(device/grayskull)
#include_directories(src/firmware/riscv/grayskull)
include_directories(device/wormhole)
include_directories(src/firmware/riscv/wormhole)
include_directories(common)

add_subdirectory(third_party/fmt)

# Device library
add_library(device
    device/cpuset_lib.cpp
    device/tt_cluster_descriptor.cpp
    device/tt_device.cpp
    device/tt_silicon_driver.cpp
    device/tt_silicon_driver_common.cpp
    device/tt_soc_descriptor.cpp
    device/util.cpp
    device/wormhole/impl_device.cpp
    # device/blackhole/impl_device.cpp
    # device/grayskull/impl_device.cpp
    # device/tt_versim_device.cpp
)

target_include_directories(device PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/device
)

target_link_libraries(device yaml-cpp hwloc rt fmt::fmt)

# Test executable
add_executable(tests
    tests/unit_test_main.cpp
    # tests/emulation/test_emulation_device.cpp
    # tests/galaxy/test_galaxy_common.cpp
    # tests/galaxy/test_umd_concurrent_threads.cpp
    # tests/galaxy/test_umd_remote_api.cpp
    # tests/galaxy/test_umd_remote_api_stability.cpp
    # tests/grayskull/test_silicon_driver.cpp
    tests/wormhole/test_silicon_driver_wh.cpp
    tests/wormhole/test_umd_remote_api_stability.cpp
)

# Link device library to tests
target_link_libraries(tests device rt pthread gtest gtest_main yaml-cpp fmt::fmt)

# Configure testing
enable_testing()
add_test(NAME AllTests COMMAND tests)

add_executable(umd-thing umd-thing.cpp)
target_link_libraries(umd-thing device rt pthread yaml-cpp fmt::fmt)
cmake_minimum_required(VERSION 3.10)
project(umd)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(yaml-cpp REQUIRED)

include_directories(common)
include_directories(.)

include_directories(device)
include_directories(device/grayskull)
include_directories(src/firmware/riscv/grayskull)
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
    device/tlb.cpp
    device/architecture_implementation.cpp
    device/grayskull_implementation.cpp
    device/blackhole_implementation.cpp
    device/wormhole_implementation.cpp
)

target_include_directories(device PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/device
)

target_link_libraries(device yaml-cpp hwloc rt fmt::fmt)

add_executable(hugepage-setup device/hugepage-setup.cpp device/hugepages.cpp)
target_link_libraries(hugepage-setup numa)

add_executable(iatu-test iatu-test.cpp)
target_link_libraries(iatu-test device rt pthread yaml-cpp fmt::fmt)

cmake_minimum_required(VERSION 3.20)
project(FirmwareTests)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Download GoogleTest automatically
include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
FetchContent_MakeAvailable(googletest)

# Your firmware headers — pure C++ ones only
set(FIRMWARE_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/../Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../Inc/app
    ${CMAKE_CURRENT_SOURCE_DIR}/../Inc/sensors
    ${CMAKE_CURRENT_SOURCE_DIR}/../Inc/comm
    ${CMAKE_CURRENT_SOURCE_DIR}/../drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/../drivers/common
    ${CMAKE_CURRENT_SOURCE_DIR}/../drivers/uart
    ${CMAKE_CURRENT_SOURCE_DIR}/../drivers/i2c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Test executable
add_executable(firmware_tests
    test_ring_buffer.cpp
    test_packet_receiver.cpp
    test_packet_receiver_variant.cpp
)

target_include_directories(firmware_tests PRIVATE
    ${FIRMWARE_INCLUDE_DIRS}
)

target_link_libraries(firmware_tests
    GTest::gtest_main
)

# Auto-discover tests
include(GoogleTest)
gtest_discover_tests(firmware_tests)
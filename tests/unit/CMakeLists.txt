cmake_minimum_required(VERSION 3.14)
project(ArduFliteUnitTests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── GoogleTest via FetchContent ───────────────────────────────────────────────
include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# ── Source paths ──────────────────────────────────────────────────────────────
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# ── Include paths ─────────────────────────────────────────────────────────────
# mocks/ must come first so our Arduino.h stub shadows any real Arduino headers.
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks
    ${ROOT}/src/controller     # pid.h
    ${ROOT}/src/orientation    # FliteQuaternion.h
)

# ── Sources under test ────────────────────────────────────────────────────────
set(SOURCES_UNDER_TEST
    ${ROOT}/src/controller/pid.cpp
    ${ROOT}/src/orientation/FliteQuaternion.cpp
)

# ── Test executable ───────────────────────────────────────────────────────────
add_executable(unit_tests
    ${SOURCES_UNDER_TEST}
    test_pid.cpp
    test_quaternion.cpp
    test_dt.cpp
    test_config_helpers.cpp
    test_servo_math.cpp
    test_motion_signals.cpp
    test_baro_decimation.cpp
    test_production_contracts.cpp
)

target_link_libraries(unit_tests GTest::gtest_main)
target_compile_definitions(unit_tests PRIVATE ARDUFLITE_ROOT="${ROOT}")

# Mirror how the Arduino build system force-includes Arduino.h in every TU.
target_compile_options(unit_tests PRIVATE
    -include ${CMAKE_CURRENT_SOURCE_DIR}/mocks/Arduino.h
)

# ── Register with ctest ──────────────────────────────────────────────────────
# gtest_discover_tests requires the binary to exist at configure time; use
# add_test instead so ctest always works regardless of build order.
add_test(NAME unit_tests COMMAND unit_tests)

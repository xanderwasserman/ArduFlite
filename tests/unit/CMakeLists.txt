cmake_minimum_required(VERSION 3.14)
project(ArduFliteUnitTests CXX)

# Must match the firmware. The ESP32 core's cpp_flags resolves to -std=gnu++2a,
# so host tests compiling as C++17 would let code pass that the firmware rejects,
# and would hide std::span / concepts / constinit. See specs/hal ADR-021.
set(CMAKE_CXX_STANDARD 20)
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
    ${ROOT}                    # "src/hal/..." paths, as the firmware sees them
    ${ROOT}/src/controller     # pid.h
    ${ROOT}/src/orientation    # FliteQuaternion.h
)

# ── Sources under test ────────────────────────────────────────────────────────
set(SOURCES_UNDER_TEST
    ${ROOT}/src/actuators/AirframeMixer.cpp
    ${ROOT}/src/core/LogRotationPolicy.cpp
    ${ROOT}/src/telemetry/PeriodicTelemetryBackend.cpp
    # Real registry and schema — reachable on a host since ADR-048.
    ${ROOT}/src/utils/Logging.cpp
    ${ROOT}/src/utils/ConfigRegistry.cpp
    ${ROOT}/src/utils/ConfigSchema.cpp
    ${ROOT}/src/estimation/AltitudeFilter.cpp
    ${ROOT}/src/estimation/CalibrationService.cpp
    ${ROOT}/src/estimation/InertialSubsystem.cpp
    ${ROOT}/src/estimation/MadgwickEstimator.cpp
    ${ROOT}/src/estimation/MotionDetector.cpp
    ${ROOT}/src/estimation/SensorSelector.cpp
    ${ROOT}/src/hal/drivers/baro/Bmp280.cpp
    ${ROOT}/src/hal/drivers/baro/Bmp581.cpp
    ${ROOT}/src/hal/drivers/imu/Bmi323.cpp
    ${ROOT}/src/hal/drivers/mag/Bmm350.cpp
    ${ROOT}/src/hal/drivers/imu/Mpu6500.cpp
    ${ROOT}/src/hal/drivers/rc/CrsfParser.cpp
    ${ROOT}/src/input/RcMapper.cpp
    ${ROOT}/src/controller/pid.cpp
    ${ROOT}/src/orientation/FliteQuaternion.cpp
)

# The L3 log-replay oracle needs no external library since Phase 9: the
# estimator is our own and depends only on <cmath>. It used to be skipped
# whenever Adafruit_AHRS was not installed, which meant the highest-value test
# in the suite was the one most likely not to run.

# ── Test executable ───────────────────────────────────────────────────────────
add_executable(unit_tests
    ${SOURCES_UNDER_TEST}
    test_pid.cpp
    test_quaternion.cpp
    test_dt.cpp
    test_config_helpers.cpp
    test_airframe_mixer.cpp
    test_motion_signals.cpp
    test_baro_decimation.cpp
    test_production_contracts.cpp
    # ── HAL Phase 0 ──────────────────────────────────────────────────────────
    test_axis_transform.cpp
    test_seqlock.cpp
    test_result.cpp
    test_board_descriptor.cpp
    test_hal_platform.cpp
    test_crsf_parser.cpp
    test_rc_mapper.cpp
    # ── HAL Phase 5 ──────────────────────────────────────────────────────────
    test_mpu6500.cpp
    test_bmp280.cpp
    # ── HAL Phase 6 ──────────────────────────────────────────────────────────
    test_sensor_selector.cpp
    test_calibration_service.cpp
    test_altitude_filter.cpp
    test_inertial_subsystem.cpp
    test_crc32.cpp
    # ── HAL Phase 6B ─────────────────────────────────────────────────────────
    test_control_types.cpp
    # ── HAL Phase 7 ──────────────────────────────────────────────────────────
    test_key_value_store.cpp
    test_log_rotation.cpp
    test_log_store.cpp
    test_config_keys.cpp
    test_bmi323.cpp
    test_bmp581.cpp
    test_bmm350.cpp
    test_magnetic_heading.cpp
    test_periodic_backend.cpp
    # ── HAL Phase 9 ──────────────────────────────────────────────────────────
    test_madgwick_estimator.cpp
    test_log_replay.cpp
)

find_package(Threads REQUIRED)

target_link_libraries(unit_tests GTest::gtest_main Threads::Threads)
target_compile_definitions(unit_tests PRIVATE ARDUFLITE_ROOT="${ROOT}" ARDUFLITE_UNIT_TEST)

# Mirror how the Arduino build system force-includes Arduino.h in every TU.
target_compile_options(unit_tests PRIVATE
    -include ${CMAKE_CURRENT_SOURCE_DIR}/mocks/Arduino.h
)

# ── Register with ctest ──────────────────────────────────────────────────────
# gtest_discover_tests requires the binary to exist at configure time; use
# add_test instead so ctest always works regardless of build order.
add_test(NAME unit_tests COMMAND unit_tests)

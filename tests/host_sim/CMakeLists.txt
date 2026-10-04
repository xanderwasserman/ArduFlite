cmake_minimum_required(VERSION 3.14)
project(arduflite_host_sim CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

get_filename_component(ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

# No external estimator dependency since Phase 9 (ADR-056): the filter is our
# own and needs only <cmath>. host_sim used to refuse to configure without
# Adafruit_AHRS installed.

add_executable(host_sim
    main.cpp
    HostStubs.cpp
    ${ROOT}/src/estimation/InertialSubsystem.cpp
    ${ROOT}/src/estimation/MadgwickEstimator.cpp
    ${ROOT}/src/estimation/CalibrationService.cpp
    ${ROOT}/src/estimation/MotionDetector.cpp
    ${ROOT}/src/estimation/AltitudeFilter.cpp
    ${ROOT}/src/estimation/SensorSelector.cpp
    ${ROOT}/src/actuators/AirframeMixer.cpp
    ${ROOT}/src/controller/ArduFliteAttitudeController.cpp
    ${ROOT}/src/controller/ArduFliteRateController.cpp
    ${ROOT}/src/controller/pid.cpp
    ${ROOT}/src/utils/ConfigRegistry.cpp
    # The real schema — self-registering, and carrying the aircraft's actual
    # default gains. host_sim therefore flies the tuning the aircraft flies,
    # not constants invented for the sim.
    ${ROOT}/src/utils/ConfigSchema.cpp
    ${ROOT}/src/orientation/FliteQuaternion.cpp
)

# tests/unit/mocks supplies <Arduino.h> and the FreeRTOS shims that the flight
# headers still include. The estimation layer needs none of it; the controller
# headers and pid.cpp still do.
target_include_directories(host_sim PRIVATE
    ${ROOT} ${ROOT}/tests/unit ${ROOT}/tests/unit/mocks)

# Same Arduino.h shim the unit tests use. The estimation layer does not need it;
# the controller headers and pid.cpp still do.
target_compile_options(host_sim PRIVATE
    -include ${ROOT}/tests/unit/mocks/Arduino.h)

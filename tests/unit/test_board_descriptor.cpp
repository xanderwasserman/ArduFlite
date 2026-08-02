/**
 * test_board_descriptor.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * The point of these tests is the NEGATIVE cases: a static_assert cannot be
 * unit-tested, so the constexpr predicates behind it are tested directly, using
 * the four real defects that exist in the legacy PinConfiguration.h today
 * (specs/hal/00-current-state.md 2.4).
 */
#include <gtest/gtest.h>

#include "src/hal/board/BoardValidate.h"
#include "src/hal/board/boards/lolin_c3_mini.h"

namespace v = arduflite::board::validate;

using arduflite::board::ActuatorBankDesc;
using arduflite::board::ActuatorOutputDesc;
using arduflite::board::ActuatorTransport;
using arduflite::board::BoardDescriptor;
using arduflite::board::BoardMaturity;
using arduflite::board::BusKind;
using arduflite::board::kNoPin;
using arduflite::board::McuProfile;
using arduflite::board::RcPart;
using arduflite::board::SensorMount;
using arduflite::board::SensorPart;
using arduflite::SignedAxis;

namespace {

/// A minimal but valid ESP32-C3 board, used as the base for each defect.
constexpr McuProfile kC3{
    .name = "ESP32-C3", .minGpio = 0, .maxGpio = 21,
    .inputOnlyMask = 0, .reservedMask = 0x000000000003F000ull,
    .uartCount = 2, .pwmChannelCount = 6,
};

constexpr BoardDescriptor makeGoodBoard()
{
    BoardDescriptor b{};
    b.name        = "test";
    b.maturity    = BoardMaturity::Supported;
    b.mcu         = kC3;
    b.sensorBus   = { .sda = 3, .scl = 5, .clock_hz = 400000 };
    b.rcUart      = { .port = 1, .rx = 6, .tx = 8, .baud = 420000, .invertRx = false };
    b.consoleUart = { .port = 0, .rx = kNoPin, .tx = kNoPin, .baud = 115200, .invertRx = false };

    b.sensors[0] = SensorMount{ SensorPart::Mpu6500, BusKind::I2c, 0x68,
                                { SignedAxis::PlusX, SignedAxis::MinusY, SignedAxis::PlusZ },
                                "imu0" };
    b.sensorCount = 1;
    b.rcLink      = RcPart::Crsf;

    b.actuatorBanks[0].transport   = ActuatorTransport::Pwm;
    b.actuatorBanks[0].outputs[0]  = ActuatorOutputDesc{ .role = "elevator", .pin = 0 };
    b.actuatorBanks[0].outputs[1]  = ActuatorOutputDesc{ .role = "rudder",   .pin = 4 };
    b.actuatorBanks[0].outputCount = 2;
    b.bankCount   = 1;

    b.userButton  = { .pin = 9, .mode = arduflite::hal::PinMode::InputPullUp, .role = "user" };
    b.statusLed   = { .pin = 7, .pixelCount = 1, .brightness = 50 };
    return b;
}

} // namespace

// ── The real board ──────────────────────────────────────────────────────────

TEST(BoardDescriptor, ShippingLolinBoardIsValid)
{
    // Mirrors the static_assert in BoardSelect.h, so a failure here names the
    // specific rule rather than just failing the build.
    const auto& b = arduflite::board::kBoard;
    EXPECT_TRUE(v::allPinsValid(b))            << "a pin is out of range or reserved";
    EXPECT_TRUE(v::allPinsUnique(b))           << "two functions share a GPIO";
    EXPECT_TRUE(v::allOutputPinsCanOutput(b))  << "an input-only GPIO drives an output";
    EXPECT_TRUE(v::outputCountWithinMcu(b))    << "more PWM outputs than LEDC channels";
    EXPECT_TRUE(v::allRolesUnique(b))          << "two outputs claim the same role";
    EXPECT_TRUE(v::allSensorAxesValid(b))      << "a sensor axis map is singular";
    EXPECT_TRUE(v::requiredPeripheralsPresent(b));
    EXPECT_TRUE(v::isValidConstexpr(b));
}

TEST(BoardDescriptor, ShippingLolinBoardKeepsTheFlyingAxisMap)
{
    // If this changes, flight behaviour changes. See specs/hal 00 2.3.
    const auto& imu = arduflite::board::kBoard.sensors[0];
    EXPECT_EQ(imu.part, SensorPart::Mpu6500);
    EXPECT_TRUE(imu.axes == (arduflite::AxisMap{ SignedAxis::PlusX,
                                                 SignedAxis::MinusY,
                                                 SignedAxis::PlusZ }));
    EXPECT_EQ(imu.axes.determinant(), -1) << "the flying map is mirrored; it should stay so";
}

TEST(BoardDescriptor, BaseTestBoardIsValid)
{
    static_assert(v::isValidConstexpr(makeGoodBoard()));
    EXPECT_TRUE(v::isValidConstexpr(makeGoodBoard()));
}

// ── The four defects that exist in PinConfiguration.h today ─────────────────

TEST(BoardValidation, CatchesGpioOutsideTheMcuRange)
{
    // Legacy: PwmInputConfig::PITCH_INPUT_PIN = 32 on an ESP32-C3 (GPIO 0..21).
    auto b = makeGoodBoard();
    b.actuatorBanks[0].outputs[0].pin = 32;

    EXPECT_FALSE(v::allPinsValid(b));
    EXPECT_FALSE(v::isValidConstexpr(b));
}

TEST(BoardValidation, CatchesReservedFlashPins)
{
    auto b = makeGoodBoard();
    b.actuatorBanks[0].outputs[0].pin = 14;   // inside the SPI-flash mask

    EXPECT_FALSE(v::allPinsValid(b));
}

TEST(BoardValidation, CatchesDuplicatePins)
{
    // Legacy: throttle input and throttle output were both GPIO 10;
    // roll input and CRSF RX were both GPIO 6; yaw input and CRSF TX both GPIO 8.
    auto b = makeGoodBoard();
    b.actuatorBanks[0].outputs[1].pin = b.rcUart.rx;   // collide an output with CRSF RX

    EXPECT_TRUE(v::allPinsValid(b))   << "the pin itself is legal — only the clash is wrong";
    EXPECT_FALSE(v::allPinsUnique(b));
    EXPECT_FALSE(v::isValidConstexpr(b));
}

TEST(BoardValidation, CatchesInputOnlyPinDrivingAnOutput)
{
    auto b = makeGoodBoard();
    b.mcu.maxGpio       = 39;
    b.mcu.inputOnlyMask = 0x000000FC00000000ull;    // classic ESP32: GPIO34-39 in only
    b.mcu.reservedMask  = 0;
    b.actuatorBanks[0].outputs[0].pin = 36;

    EXPECT_TRUE(v::allPinsValid(b));
    EXPECT_FALSE(v::allOutputPinsCanOutput(b));
}

// ── Rules added by the Actuator/Bank split ──────────────────────────────────

TEST(BoardValidation, CatchesDuplicateRolesAcrossBanks)
{
    // byRole() is how flight code resolves actuators, so a duplicate is ambiguous.
    auto b = makeGoodBoard();
    b.actuatorBanks[1].transport   = ActuatorTransport::CanOpen;
    b.actuatorBanks[1].outputs[0]  = ActuatorOutputDesc{ .role = "elevator", .pin = kNoPin,
                                                         .nodeId = 12 };
    b.actuatorBanks[1].outputCount = 1;
    b.bankCount = 2;

    EXPECT_FALSE(v::allRolesUnique(b));
    EXPECT_FALSE(v::isValidConstexpr(b));
}

TEST(BoardValidation, AcceptsMixedTransportsWithDistinctRoles)
{
    auto b = makeGoodBoard();
    b.actuatorBanks[1].transport   = ActuatorTransport::CanOpen;
    b.actuatorBanks[1].outputs[0]  = ActuatorOutputDesc{ .role = "throttle", .pin = kNoPin,
                                                         .nodeId = 12 };
    b.actuatorBanks[1].outputCount = 1;
    b.bankCount = 2;

    EXPECT_TRUE(v::allRolesUnique(b));
    EXPECT_TRUE(v::isValidConstexpr(b)) << "PWM surfaces plus a CAN throttle must be legal";
}

TEST(BoardValidation, CanOutputsDoNotConsumePwmChannels)
{
    auto b = makeGoodBoard();
    b.actuatorBanks[1].transport = ActuatorTransport::CanOpen;
    for (int i = 0; i < 8; ++i)
    {
        b.actuatorBanks[1].outputs[i] = ActuatorOutputDesc{ .role = "n", .pin = kNoPin,
                                                            .nodeId = static_cast<std::uint8_t>(i) };
    }
    b.actuatorBanks[1].outputCount = 8;
    b.bankCount = 2;

    // 8 CAN nodes must not exhaust 6 LEDC channels.
    EXPECT_TRUE(v::outputCountWithinMcu(b));
}

TEST(BoardValidation, CatchesTooManyPwmOutputs)
{
    auto b = makeGoodBoard();
    for (int i = 0; i < 7; ++i)
    {
        b.actuatorBanks[0].outputs[i] =
            ActuatorOutputDesc{ .role = "x", .pin = static_cast<arduflite::board::Pin>(i) };
    }
    b.actuatorBanks[0].outputCount = 7;   // mcu has 6 channels

    EXPECT_FALSE(v::outputCountWithinMcu(b));
}

TEST(BoardValidation, CatchesEmptyRole)
{
    auto b = makeGoodBoard();
    b.actuatorBanks[0].outputs[0].role = "";
    EXPECT_FALSE(v::allRolesUnique(b));
}

TEST(BoardValidation, CatchesSingularSensorAxisMap)
{
    auto b = makeGoodBoard();
    b.sensors[0].axes = { SignedAxis::PlusX, SignedAxis::PlusX, SignedAxis::PlusZ };
    EXPECT_FALSE(v::allSensorAxesValid(b));
    EXPECT_FALSE(v::isValidConstexpr(b));
}

// ── Board maturity ──────────────────────────────────────────────────────────

TEST(BoardValidation, SupportedBoardMustHaveItsRcLinkWired)
{
    auto b = makeGoodBoard();
    b.rcUart.rx = kNoPin;
    EXPECT_FALSE(v::requiredPeripheralsPresent(b));
}

TEST(BoardValidation, UntestedBoardMayHaveUnknownPins)
{
    // This is what lets the FireBeetle port compile before anyone meters it,
    // instead of carrying forward the legacy //TODO guess.
    auto b = makeGoodBoard();
    b.maturity  = BoardMaturity::Untested;
    b.rcLink    = RcPart::Unknown;
    b.rcUart.rx = kNoPin;
    b.rcUart.tx = kNoPin;

    EXPECT_TRUE(v::requiredPeripheralsPresent(b));
    EXPECT_TRUE(v::isValidConstexpr(b));
}

TEST(BoardValidation, SupportedBoardMayNotHaveAnUnknownRcLink)
{
    auto b = makeGoodBoard();
    b.maturity = BoardMaturity::Supported;
    b.rcLink   = RcPart::Unknown;
    EXPECT_FALSE(v::requiredPeripheralsPresent(b));
}

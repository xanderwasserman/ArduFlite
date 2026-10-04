/**
 * AirframeMixer.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Airframe geometry mixing — pure function, no state, no hardware.
 *
 * Pure geometry: normalised axis demands in, per-surface demands out. Per-output calibration
 * (inversion, travel limits, trim, slew) is NOT here: that belongs on the
 * Actuator, because it must apply to every writer including manual passthrough
 * and failsafe. See specs/hal ADR-005.
 */
#ifndef ARDUFLITE_ACTUATORS_AIRFRAMEMIXER_H
#define ARDUFLITE_ACTUATORS_AIRFRAMEMIXER_H

#include <cstdint>

namespace arduflite::actuators {

/// Values are persisted in CONFIG_KEY_SERVO_WING_DESIGN, so they are wire
/// (0 = Conventional, 1 = Delta, 2 = V-Tail) — the stored config depends on it.
enum class WingDesign : std::uint8_t
{
    Conventional = 0,
    DeltaWing    = 1,
    VTail        = 2,   ///< STUB — not implemented. See isImplemented().
};

/**
 * @brief Whether mix() actually produces control for this geometry.
 *
 * VTail is a reserved-but-unimplemented stub, because the obvious mix never
 * read rollCmd, so it gave a V-tail airframe no roll authority at all — it was
 * never flown and never tested. Rather than port a latent bug forward or invent
 * untested mixing, the geometry is kept as an enum value (the stored config
 * encoding depends on it) and the path is left explicitly empty.
 *
 * Callers MUST check this and refuse to arm, or fall back, rather than flying a
 * geometry that mixes to nothing.
 */
[[nodiscard]] constexpr bool isImplemented(WingDesign d) noexcept
{
    return d == WingDesign::Conventional || d == WingDesign::DeltaWing;
}

/// Normalised [-1, 1] surface commands, before any per-output calibration.
struct SurfaceCommands
{
    float aileronLeft  = 0.0f;
    float aileronRight = 0.0f;
    float elevator     = 0.0f;
    float rudder       = 0.0f;
};

class AirframeMixer
{
public:
    /**
     * @brief Mix pilot/controller axes into per-surface commands.
     *
     * @param roll,pitch,yaw  Normalised axis demands. Clamped to [-1,1]; a
     *                        non-finite input yields all-zero output, matching
     *                        ServoManager's NaN guard (which returned early and
     *                        held the previous position — the Actuator's
     *                        NaN-hold now provides that half).
     */
    [[nodiscard]] static SurfaceCommands mix(WingDesign design,
                                             float roll, float pitch, float yaw) noexcept;
};

} // namespace arduflite::actuators

#endif // ARDUFLITE_ACTUATORS_AIRFRAMEMIXER_H

# ArduFlite Configuration Reference

This document describes all runtime-tunable parameters available in ArduFlite. Use the CLI to view and modify these values:

```bash
config list              # Show all parameters
config get <key|pattern> # Get current value; pattern accepts a trailing *
config set <key> <value> # Set new value
config save              # Write changed parameters to NVS
config load              # Reload from NVS
config defaults          # Restore schema defaults
```

`config set` is refused while armed or in flight, as are `load` and `defaults`.
JSON export and import are available over the web API (`/api/config/export`,
`/api/config/import`), not from the CLI.

---


> **Schema version 2 — unit suffixes.** Every key carrying a physical quantity now
> names its unit: `att.deadband_rad`, `servo.pitch.min_pulse_us`,
> `mix.max_rate_roll_dps`, `rate.roll.ti_s`. Dimensionless values (`*.kp`, `*.alpha`,
> `*.headroom`, `rate.*.outlimit`, `servo.wing_design`) are unchanged — the absence of
> a suffix is itself informative.
>
> **Wipe NVS after flashing.** Renamed keys hash to new NVS entries, so v1 values are
> not found and the code defaults apply. Those defaults are the current flying values.

## Table of Contents

- [Rate Controller (Inner Loop)](#rate-controller-inner-loop)
- [Attitude Controller (Outer Loop)](#attitude-controller-outer-loop)
- [Control Mixer](#control-mixer)
- [Servo Configuration](#servo-configuration)
- [IMU Configuration](#imu-configuration)
- [Failsafe Configuration](#failsafe-configuration)
- [CRSF Receiver](#crsf-receiver)
- [System](#system)

---

## Rate Controller (Inner Loop)

The rate controller runs at ~500 Hz and converts angular rate errors (deg/s) into servo commands. This is the "inner loop" of the cascade control system.

### Roll Rate PID

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `rate.roll.kp` | 0.09 | 0.0 - 1.0 | Proportional gain |
| `rate.roll.ti_s` | 1.40 | 0.0 - 10.0 | Integral time constant (seconds) |
| `rate.roll.td_s` | 0.30 | 0.0 - 1.0 | Derivative time constant (seconds) |
| `rate.roll.outlimit` | 1.00 | 0.1 - 1.0 | Output limit (normalized) |
| `rate.roll.headroom` | 0.80 | 0.5 - 1.0 | Anti-windup headroom factor |
| `rate.roll.alpha` | 0.10 | 0.01 - 1.0 | Derivative low-pass filter coefficient |

**Tuning Notes:**
- **kp ↑**: Faster response to rate errors, but risks oscillation if too high
- **kp ↓**: Slower, mushier response; aircraft feels "lazy"
- **ti ↑**: Slower integral action (less I gain); reduces overshoot but slower to eliminate steady-state error
- **ti ↓**: Faster integral action (more I gain); quicker trim correction but may cause oscillation
- **td ↑**: More derivative action; helps dampen oscillations but can amplify noise
- **alpha ↓**: Heavier filtering on derivative; reduces noise but adds lag

### Pitch Rate PID

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `rate.pitch.kp` | 0.04 | 0.0 - 1.0 | Proportional gain |
| `rate.pitch.ti_s` | 5.00 | 0.0 - 10.0 | Integral time constant (seconds) |
| `rate.pitch.td_s` | 0.45 | 0.0 - 1.0 | Derivative time constant (seconds) |
| `rate.pitch.outlimit` | 1.00 | 0.1 - 1.0 | Output limit (normalized) |
| `rate.pitch.headroom` | 0.80 | 0.5 - 1.0 | Anti-windup headroom factor |
| `rate.pitch.alpha` | 0.10 | 0.01 - 1.0 | Derivative low-pass filter coefficient |

**Tuning Notes:**
- Pitch typically needs lower P gain than roll due to different inertia
- Longer Ti (slower integral) helps prevent pitch bobbing
- Higher Td helps dampen phugoid oscillations

### Yaw Rate PID

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `rate.yaw.kp` | 0.05 | 0.0 - 1.0 | Proportional gain |
| `rate.yaw.ti_s` | 8.00 | 0.0 - 10.0 | Integral time constant (seconds). Deliberately slower than roll and pitch — the rudder is a weak, laggy control. 0 disables |
| `rate.yaw.td_s` | 0.30 | 0.0 - 1.0 | Derivative time constant (seconds) |
| `rate.yaw.outlimit` | 1.00 | 0.1 - 1.0 | Output limit (normalized) |
| `rate.yaw.headroom` | 0.80 | 0.5 - 1.0 | Anti-windup headroom factor |
| `rate.yaw.alpha` | 0.10 | 0.01 - 1.0 | Derivative low-pass filter coefficient |

**Tuning Notes:**
- Yaw authority is often limited; modest gains work best
- On flying wings without rudder, yaw is controlled via differential thrust or drag devices

### Rate Output Filter

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `rate.out_lp_alpha` | 0.3 | 0.001 - 1.0 | Output low-pass filter coefficient |

**Tuning Notes:**
- **alpha ↓**: Smoother servo movements, but adds control lag
- **alpha ↑**: More responsive but may cause servo jitter
- 0.3 gives τ ≈ 7 ms at 500 Hz inner loop — snappy yet smooth
- Reduce to 0.05–0.1 for smoother flight; increase toward 1.0 for aerobatics

---

## Attitude Controller (Outer Loop)

The attitude controller runs at ~100 Hz and converts attitude errors (degrees) into rate setpoints for the inner loop. Only active in ATTITUDE_MODE.

### Roll Attitude PID

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `att.roll.kp` | 320.0 | 0.0 - 1000.0 | Proportional gain (deg/s per deg error) |
| `att.roll.ti_s` | 0.00 | 0.0 - 10.0 | Integral time constant (seconds) |
| `att.roll.td_s` | 0.00 | 0.0 - 1.0 | Derivative time constant (seconds) |
| `att.roll.outlimit_dps` | 90.0 | 10.0 - 180.0 | Max rate setpoint output (deg/s) |
| `att.roll.headroom` | 0.80 | 0.5 - 1.0 | Anti-windup headroom factor |
| `att.roll.alpha` | 0.10 | 0.01 - 1.0 | Derivative low-pass filter coefficient |

**Tuning Notes:**
- **kp**: Determines how aggressively the aircraft levels. 320 means 1° error → 320°/s rate command
- **outlimit**: Caps the rate setpoint; lower = gentler leveling, higher = snappier
- Attitude I/D terms are typically 0 (P-only is usually sufficient for outer loop)

### Pitch Attitude PID

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `att.pitch.kp` | 150.0 | 0.0 - 1000.0 | Proportional gain (deg/s per deg error) |
| `att.pitch.ti_s` | 0.00 | 0.0 - 10.0 | Integral time constant (seconds) |
| `att.pitch.td_s` | 0.00 | 0.0 - 1.0 | Derivative time constant (seconds) |
| `att.pitch.outlimit_dps` | 45.0 | 10.0 - 180.0 | Max rate setpoint output (deg/s) |
| `att.pitch.headroom` | 0.80 | 0.5 - 1.0 | Anti-windup headroom factor |
| `att.pitch.alpha` | 0.10 | 0.01 - 1.0 | Derivative low-pass filter coefficient |

**Tuning Notes:**
- Lower outlimit (60°/s) than roll keeps pitch corrections gentle
- Pitch has more inertia; lower kp prevents overshooting level flight

### Yaw Attitude PID

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `att.yaw.kp` | 200.0 | 0.0 - 1000.0 | Proportional gain (deg/s per deg error) |
| `att.yaw.ti_s` | 0.00 | 0.0 - 10.0 | Integral time constant (seconds) |
| `att.yaw.td_s` | 0.00 | 0.0 - 1.0 | Derivative time constant (seconds) |
| `att.yaw.outlimit_dps` | 60.0 | 10.0 - 180.0 | Max rate setpoint output (deg/s) |
| `att.yaw.headroom` | 0.80 | 0.5 - 1.0 | Anti-windup headroom factor |
| `att.yaw.alpha` | 0.10 | 0.01 - 1.0 | Derivative low-pass filter coefficient |

### Attitude Deadband

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `att.deadband_rad` | 0.0001 | 0.0 - 0.01 | Error deadband (radians) |

**Tuning Notes:**
- Prevents micro-corrections when nearly level
- 0.0001 rad ≈ 0.006° — essentially disabled
- Increase to 0.001-0.005 if servos chatter at level flight

---

## Control Mixer

The mixer scales pilot stick inputs to setpoints based on the current flight mode.

### Attitude Mode Limits

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `mix.max_att_roll_deg` | 55.0 | 10.0 - 90.0 | Max roll angle (degrees) |
| `mix.max_att_pitch_deg` | 50.0 | 10.0 - 90.0 | Max pitch angle (degrees) |
| `mix.max_att_yaw_deg` | 180.0 | 45.0 - 360.0 | Max yaw heading offset (degrees) |

**Tuning Notes:**
- **max.att.roll/pitch**: Full stick deflection commands this angle
- Lower values = gentler, more beginner-friendly; higher = more aerobatic
- 45° is a good starting point; reduce for trainers, increase for sport flying

### Rate Mode Limits

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `mix.max_rate_roll_dps` | 90.0 | 30.0 - 360.0 | Max roll rate (deg/s) |
| `mix.max_rate_pitch_dps` | 60.0 | 30.0 - 360.0 | Max pitch rate (deg/s) |
| `mix.max_rate_yaw_dps` | 60.0 | 30.0 - 360.0 | Max yaw rate (deg/s) |

**Tuning Notes:**
- Full stick deflection commands this rate
- 360°/s = one full rotation per second (aerobatic)
- Start low and increase as you gain confidence

### Coordinated Turn Mixing (SAFE Mode)

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `mix.roll_from_yaw` | 0.00 | 0.0 - 0.5 | Yaw input adds roll |
| `mix.pitch_from_roll` | 0.08 | 0.0 - 0.5 | Roll input adds pitch (prevents nose drop in turns) |
| `mix.yaw_from_roll` | 0.10 | 0.0 - 0.5 | Roll input adds yaw (coordinated turn) |

**Tuning Notes:**
- **yaw.from.roll**: Adds rudder when banking for coordinated turns. 0.1 = 10% of roll command added to yaw
- **pitch.from.roll**: Adds up-elevator in turns to prevent nose drop
- Set all to 0 for pure independent axis control

---

## Servo Configuration

### Airframe Geometry

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `servo.wing_design` | 0 | 0 - 2 | Wing type: 0=Conventional, 1=Delta Wing, 2=V-Tail (see below) |
| `servo.dual_ailerons` | true | true/false | Use two aileron servos (vs single) |

**Wing Design Values:**
- **0 (CONVENTIONAL)**: Separate ailerons, elevator, rudder
- **1 (DELTA_WING)**: Elevons only (roll+pitch mixed)
- **2 (V_TAIL)**: Reserved, and **not implemented** — the mixer produces no
  deflection for this geometry, so a V-tail airframe has no control surfaces.
  Do not select it.

### Slew Rate Limits

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `servo.max_slew_dps` | 500.0 | 100.0 - 1000.0 | Max servo movement rate (deg/s) |
| `servo.max_thr_slew_per_s` | 1.0 | 0.1 - 5.0 | Max throttle change rate (range/s) |

**Tuning Notes:**
- **max.deg.sec ↓**: Smoother servo movements, protects gears, but limits responsiveness
- **max.thr.sec ↓**: Gentler throttle transitions (good for electric motors)

### Pitch Servo

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `servo.pitch.min_pulse_us` | 500 | 500 - 1000 | Min pulse width (µs) |
| `servo.pitch.max_pulse_us` | 2500 | 2000 - 2500 | Max pulse width (µs) |
| `servo.pitch.neutral_deg` | 90 | 0 - 180 | Neutral position (degrees) |
| `servo.pitch.deflection_deg` | 80 | 10 - 90 | Max deflection from neutral (degrees) |
| `servo.pitch.invert` | true | true/false | Invert servo direction |

### Yaw Servo

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `servo.yaw.min_pulse_us` | 500 | 500 - 1000 | Min pulse width (µs) |
| `servo.yaw.max_pulse_us` | 2500 | 2000 - 2500 | Max pulse width (µs) |
| `servo.yaw.neutral_deg` | 90 | 0 - 180 | Neutral position (degrees) |
| `servo.yaw.deflection_deg` | 80 | 10 - 90 | Max deflection from neutral (degrees) |
| `servo.yaw.invert` | false | true/false | Invert servo direction |

### Left Aileron Servo

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `servo.lail.min_pulse_us` | 500 | 500 - 1000 | Min pulse width (µs) |
| `servo.lail.max_pulse_us` | 2500 | 2000 - 2500 | Max pulse width (µs) |
| `servo.lail.neutral_deg` | 90 | 0 - 180 | Neutral position (degrees) |
| `servo.lail.deflection_deg` | 80 | 10 - 90 | Max deflection from neutral (degrees) |
| `servo.lail.invert` | true | true/false | Invert servo direction |

### Right Aileron Servo

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `servo.rail.min_pulse_us` | 500 | 500 - 1000 | Min pulse width (µs) |
| `servo.rail.max_pulse_us` | 2500 | 2000 - 2500 | Max pulse width (µs) |
| `servo.rail.neutral_deg` | 90 | 0 - 180 | Neutral position (degrees) |
| `servo.rail.deflection_deg` | 80 | 10 - 90 | Max deflection from neutral (degrees) |
| `servo.rail.invert` | false | true/false | Invert servo direction |

### Throttle

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `servo.thr.min_pulse_us` | 1000 | 500 - 1500 | Min throttle pulse (µs) |
| `servo.thr.max_pulse_us` | 2000 | 1500 - 2500 | Max throttle pulse (µs) |

**Tuning Notes:**
- **invert**: If servo moves wrong direction, toggle this instead of rewiring
- **neutral**: Set to where control surface is streamlined (usually 90°)
- **deflection**: Limit travel to prevent binding; start at 80° and reduce if needed

---

## IMU Configuration

### Sensor Filtering

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `imu.accel_alpha` | 0.02 | 0.001 - 1.0 | Accelerometer low-pass filter |
| `imu.gyro_alpha` | 0.4 | 0.001 - 1.0 | Gyroscope low-pass filter |
| `imu.mag_alpha` | 0.04 | 0.001 - 1.0 | Magnetometer low-pass filter |
| `imu.alti_alpha` | 0.10 | 0.001 - 0.50 | Altimeter low-pass filter. At the 50 Hz baro rate, 0.10 gives τ ≈ 200 ms |

**Tuning Notes:**
- **alpha ↓**: Heavier filtering, smoother readings, but more lag
- **alpha ↑**: Less filtering, more responsive, but noisier
- `gyro_alpha` 0.4 gives ~50 Hz cutoff at 500 Hz — balances responsiveness with noise rejection
- Altimeter: Keep very low (0.005) — barometer is read at ~50 Hz

### Madgwick Filter

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `imu.madgwick_beta` | 0.1 | 0.01 - 1.0 | Madgwick filter beta (gyro/accel trust) |
| `imu.fuse_mag` | false | true/false | Fuse a magnetometer into attitude (9-axis) |

**`imu.fuse_mag` defaults to false even when a magnetometer is fitted.** The
part is still probed, sampled and published to telemetry; it simply does not
steer the estimate. Enabling it without hard-iron calibration lets an
uncalibrated field take correction authority away from the accelerometer, which
degrades roll and pitch — the axes the control loops fly on.

**Tuning Notes:**
- **beta ↑**: Trust accelerometer more, faster convergence, but more sensitive to vibration
- **beta ↓**: Trust gyro more, smoother attitude, but slower to correct drift
- Default 0.1 works well for most applications
- Increase to 0.2–0.5 for aggressive aerobatics; decrease to 0.05 for smooth flight

### Sensor Limits

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `imu.max_accel_g` | 16.0 | 4.0 - 16.0 | Max valid acceleration (g) |
| `imu.max_gyro_dps` | 2000.0 | 250.0 - 2000.0 | Max valid angular rate (deg/s) |
| `imu.fail_threshold` | 5 | 1 - 20 | Consecutive failures before unhealthy |

**Tuning Notes:**
- Readings exceeding these limits are rejected as sensor errors
- Increase `fail_threshold` if you get false "IMU unhealthy" warnings

### Calibration Validation

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `imu.gyro_bias_max_dps` | 5.0 | 1.0 - 20.0 | Max acceptable gyro bias (deg/s) |
| `imu.expected_g` | 1.0 | 0.9 - 1.1 | Expected gravity magnitude (g) |
| `imu.gravity_tol_g` | 0.15 | 0.05 - 0.3 | Gravity reading tolerance (g) |

**Tuning Notes:**
- If calibration fails, increase `gravity_tol` or ensure aircraft is perfectly still
- `gyro_bias_max`: Gyros with bias > this after calibration indicate a bad sensor

---

## Failsafe Configuration

These settings control aircraft behavior when RC link is lost.

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `failsafe.bank_deg` | 7.0 | 0.0 - 30.0 | Bank angle during failsafe (degrees) |
| `failsafe.pitch_deg` | -3.0 | -20.0 - 0.0 | Pitch angle during failsafe (degrees) |
| `failsafe.throttle` | 0.0 | 0.0 - 1.0 | Throttle setting during failsafe |
| `failsafe.min_lq_arm_pct` | 50 | 20 - 100 | Minimum link quality % to arm |

**Failsafe Behavior:**
When RC link is lost, the aircraft:
1. Switches to ATTITUDE_MODE
2. Banks to `failsafe.bank_deg` (creates a gentle spiral)
3. Pitches to `failsafe.pitch_deg` (slight nose-down for controlled descent)
4. Cuts throttle to `failsafe.throttle` (default 0 = engine off)

**Tuning Notes:**
- **bank.deg**: 5-10° creates a contained spiral; 0° = straight glide
- **pitch.deg**: -3° to -5° maintains airspeed without steep dive
- **throttle**: Set to 0 to cut engine (safest); set higher if you want powered descent
- **min.lq.arm**: Prevents arming with weak link; 50% is conservative

---

## CRSF Receiver

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `crsf.tri_low` | 0.33 | 0.1 - 0.4 | TriState switch low threshold |
| `crsf.tri_high` | 0.66 | 0.6 - 0.9 | TriState switch high threshold |

**Tuning Notes:**
- These thresholds determine how 3-position switch values are interpreted
- Input < `crsf.tri_low` = position 1 (e.g., ATTITUDE_MODE)
- Input > `crsf.tri_high` = position 3 (e.g., MANUAL_MODE)
- Otherwise = position 2 (e.g., RATE_MODE)

---

## Web Interface

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `web.enabled` | false | true/false | Enable the WiFi AP and web server (reboot to apply) |
| `web.ap_ssid` | ArduFlite | string | AP name. A per-device suffix is appended |
| `web.ap_pass` | arduflite | string | WPA2 password, 8+ characters |

Available only in the full build (`ENABLE_WEB_SERVER`); the lite build has no
WiFi stack. Set `web.ap_pass` before field use — while it is left at the
default, the firmware uses the unique AP SSID as a temporary password and warns
at boot.

## System

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `sys.aircraft_name` | "ArduFlite" | string | Aircraft name for telemetry display |

**Tuning Notes:**
- Displayed on your transmitter's telemetry screen
- Useful if you have multiple aircraft

---

## Tuning Workflow

### First Flight Checklist
1. **Servos**: Verify all control surfaces move correct direction
   - Toggle `servo.*.invert` as needed
   - Adjust `servo.*.neutral` for level surfaces at rest
2. **Failsafe**: Test failsafe behavior on the ground (disarm first!)
3. **Start conservative**: Use default PID values, low mixer limits

### In-Flight Tuning Order
1. **Rate controller first** (inner loop must be stable before outer loop)
   - Start with roll axis
   - Increase `rate.roll.kp` until you see oscillation, then back off 30%
   - Repeat for pitch and yaw
2. **Attitude controller second**
   - Test in ATTITUDE_MODE
   - Adjust `att.*.kp` for desired leveling speed
   - Reduce `att.*.outlimit` if self-leveling feels too aggressive
3. **Mixer limits last**
   - Increase `mix.max.*` values as you gain confidence

### Common Problems

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| Oscillation in level flight | Rate P too high | Reduce `rate.*.kp` |
| Slow to respond | Rate P too low | Increase `rate.*.kp` |
| Servo jitter | D term amplifying noise | Reduce `rate.*.td` or `rate.*.alpha` |
| Drifts off level | Needs integral | Add small `rate.*.ti` (start with 2-3s) |
| Overshoots level | Attitude P too high | Reduce `att.*.kp` |
| Won't hold trim | Needs rate integral | Reduce `rate.*.ti` (faster I action) |

# ArduFlite Configuration Reference

This document describes all runtime-tunable parameters available in ArduFlite. Use the CLI to view and modify these values:

```bash
config list              # Show all parameters
config get <key>         # Get current value
config set <key> <value> # Set new value (persists across reboots)
config export            # Export all config as JSON
config reset             # Reset to defaults
```

---

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
| `rate.roll.ti` | 1.40 | 0.0 - 10.0 | Integral time constant (seconds) |
| `rate.roll.td` | 0.30 | 0.0 - 1.0 | Derivative time constant (seconds) |
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
| `rate.pitch.kp` | 0.06 | 0.0 - 1.0 | Proportional gain |
| `rate.pitch.ti` | 5.00 | 0.0 - 10.0 | Integral time constant (seconds) |
| `rate.pitch.td` | 0.70 | 0.0 - 1.0 | Derivative time constant (seconds) |
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
| `rate.yaw.ti` | 0.00 | 0.0 - 10.0 | Integral time constant (seconds) - 0 disables, prevents drift without magnetometer |
| `rate.yaw.td` | 0.30 | 0.0 - 1.0 | Derivative time constant (seconds) |
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
| `att.roll.ti` | 0.00 | 0.0 - 10.0 | Integral time constant (seconds) |
| `att.roll.td` | 0.00 | 0.0 - 1.0 | Derivative time constant (seconds) |
| `att.roll.outlimit` | 90.0 | 10.0 - 180.0 | Max rate setpoint output (deg/s) |
| `att.roll.headroom` | 0.80 | 0.5 - 1.0 | Anti-windup headroom factor |
| `att.roll.alpha` | 0.10 | 0.01 - 1.0 | Derivative low-pass filter coefficient |

**Tuning Notes:**
- **kp**: Determines how aggressively the aircraft levels. 320 means 1° error → 320°/s rate command
- **outlimit**: Caps the rate setpoint; lower = gentler leveling, higher = snappier
- Attitude I/D terms are typically 0 (P-only is usually sufficient for outer loop)

### Pitch Attitude PID

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `att.pitch.kp` | 200.0 | 0.0 - 1000.0 | Proportional gain (deg/s per deg error) |
| `att.pitch.ti` | 0.00 | 0.0 - 10.0 | Integral time constant (seconds) |
| `att.pitch.td` | 0.00 | 0.0 - 1.0 | Derivative time constant (seconds) |
| `att.pitch.outlimit` | 60.0 | 10.0 - 180.0 | Max rate setpoint output (deg/s) |
| `att.pitch.headroom` | 0.80 | 0.5 - 1.0 | Anti-windup headroom factor |
| `att.pitch.alpha` | 0.10 | 0.01 - 1.0 | Derivative low-pass filter coefficient |

**Tuning Notes:**
- Lower outlimit (60°/s) than roll keeps pitch corrections gentle
- Pitch has more inertia; lower kp prevents overshooting level flight

### Yaw Attitude PID

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `att.yaw.kp` | 200.0 | 0.0 - 1000.0 | Proportional gain (deg/s per deg error) |
| `att.yaw.ti` | 0.00 | 0.0 - 10.0 | Integral time constant (seconds) |
| `att.yaw.td` | 0.00 | 0.0 - 1.0 | Derivative time constant (seconds) |
| `att.yaw.outlimit` | 60.0 | 10.0 - 180.0 | Max rate setpoint output (deg/s) |
| `att.yaw.headroom` | 0.80 | 0.5 - 1.0 | Anti-windup headroom factor |
| `att.yaw.alpha` | 0.10 | 0.01 - 1.0 | Derivative low-pass filter coefficient |

### Attitude Deadband

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `att.deadband` | 0.0001 | 0.0 - 0.01 | Error deadband (radians) |

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
| `mix.max.att.roll` | 45.0 | 10.0 - 90.0 | Max roll angle (degrees) |
| `mix.max.att.pitch` | 45.0 | 10.0 - 90.0 | Max pitch angle (degrees) |
| `mix.max.att.yaw` | 180.0 | 45.0 - 360.0 | Max yaw heading offset (degrees) |

**Tuning Notes:**
- **max.att.roll/pitch**: Full stick deflection commands this angle
- Lower values = gentler, more beginner-friendly; higher = more aerobatic
- 45° is a good starting point; reduce for trainers, increase for sport flying

### Rate Mode Limits

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `mix.max.rate.roll` | 90.0 | 30.0 - 360.0 | Max roll rate (deg/s) |
| `mix.max.rate.pitch` | 60.0 | 30.0 - 360.0 | Max pitch rate (deg/s) |
| `mix.max.rate.yaw` | 60.0 | 30.0 - 360.0 | Max yaw rate (deg/s) |

**Tuning Notes:**
- Full stick deflection commands this rate
- 360°/s = one full rotation per second (aerobatic)
- Start low and increase as you gain confidence

### Coordinated Turn Mixing (SAFE Mode)

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `mix.roll.from.yaw` | 0.00 | 0.0 - 0.5 | Yaw input adds roll |
| `mix.pitch.from.roll` | 0.05 | 0.0 - 0.5 | Roll input adds pitch (prevents nose drop in turns) |
| `mix.yaw.from.roll` | 0.10 | 0.0 - 0.5 | Roll input adds yaw (coordinated turn) |

**Tuning Notes:**
- **yaw.from.roll**: Adds rudder when banking for coordinated turns. 0.1 = 10% of roll command added to yaw
- **pitch.from.roll**: Adds up-elevator in turns to prevent nose drop
- Set all to 0 for pure independent axis control

---

## Servo Configuration

### Airframe Geometry

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `servo.wing_design` | 0 | 0 - 2 | Wing type: 0=Conventional, 1=Delta Wing, 2=V-Tail |
| `servo.dual_ailerons` | true | true/false | Use two aileron servos (vs single) |

**Wing Design Values:**
- **0 (CONVENTIONAL)**: Separate ailerons, elevator, rudder
- **1 (DELTA_WING)**: Elevons only (roll+pitch mixed)
- **2 (V_TAIL)**: Ruddervators (pitch+yaw mixed)

### Slew Rate Limits

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `servo.max.deg.sec` | 500.0 | 100.0 - 1000.0 | Max servo movement rate (deg/s) |
| `servo.max.thr.sec` | 1.0 | 0.1 - 5.0 | Max throttle change rate (range/s) |

**Tuning Notes:**
- **max.deg.sec ↓**: Smoother servo movements, protects gears, but limits responsiveness
- **max.thr.sec ↓**: Gentler throttle transitions (good for electric motors)

### Pitch Servo

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `servo.pitch.min` | 500 | 500 - 1000 | Min pulse width (µs) |
| `servo.pitch.max` | 2500 | 2000 - 2500 | Max pulse width (µs) |
| `servo.pitch.neutral` | 90 | 0 - 180 | Neutral position (degrees) |
| `servo.pitch.deflection` | 80 | 10 - 90 | Max deflection from neutral (degrees) |
| `servo.pitch.invert` | true | true/false | Invert servo direction |

### Yaw Servo

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `servo.yaw.min` | 500 | 500 - 1000 | Min pulse width (µs) |
| `servo.yaw.max` | 2500 | 2000 - 2500 | Max pulse width (µs) |
| `servo.yaw.neutral` | 90 | 0 - 180 | Neutral position (degrees) |
| `servo.yaw.deflection` | 80 | 10 - 90 | Max deflection from neutral (degrees) |
| `servo.yaw.invert` | false | true/false | Invert servo direction |

### Left Aileron Servo

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `servo.lail.min` | 500 | 500 - 1000 | Min pulse width (µs) |
| `servo.lail.max` | 2500 | 2000 - 2500 | Max pulse width (µs) |
| `servo.lail.neutral` | 90 | 0 - 180 | Neutral position (degrees) |
| `servo.lail.deflection` | 80 | 10 - 90 | Max deflection from neutral (degrees) |
| `servo.lail.invert` | true | true/false | Invert servo direction |

### Right Aileron Servo

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `servo.rail.min` | 500 | 500 - 1000 | Min pulse width (µs) |
| `servo.rail.max` | 2500 | 2000 - 2500 | Max pulse width (µs) |
| `servo.rail.neutral` | 90 | 0 - 180 | Neutral position (degrees) |
| `servo.rail.deflection` | 80 | 10 - 90 | Max deflection from neutral (degrees) |
| `servo.rail.invert` | false | true/false | Invert servo direction |

### Throttle

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `servo.thr.min` | 1000 | 500 - 1500 | Min throttle pulse (µs) |
| `servo.thr.max` | 2000 | 1500 - 2500 | Max throttle pulse (µs) |

**Tuning Notes:**
- **invert**: If servo moves wrong direction, toggle this instead of rewiring
- **neutral**: Set to where control surface is streamlined (usually 90°)
- **deflection**: Limit travel to prevent binding; start at 80° and reduce if needed

---

## IMU Configuration

### Sensor Filtering

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `imu.accel_alpha` | 0.02 | 0.01 - 1.0 | Accelerometer low-pass filter |
| `imu.gyro_alpha` | 0.4 | 0.01 - 1.0 | Gyroscope low-pass filter |
| `imu.mag_alpha` | 0.04 | 0.01 - 1.0 | Magnetometer low-pass filter |
| `imu.alti_alpha` | 0.005 | 0.001 - 0.1 | Altimeter low-pass filter |

**Tuning Notes:**
- **alpha ↓**: Heavier filtering, smoother readings, but more lag
- **alpha ↑**: Less filtering, more responsive, but noisier
- `gyro_alpha` 0.4 gives ~50 Hz cutoff at 500 Hz — balances responsiveness with noise rejection
- Altimeter: Keep very low (0.005) — barometer is read at ~50 Hz

### Madgwick Filter

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `imu.madgwick_beta` | 0.1 | 0.01 - 1.0 | Madgwick filter beta (gyro/accel trust) |

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
| `imu.gyro_bias_max` | 5.0 | 1.0 - 20.0 | Max acceptable gyro bias (deg/s) |
| `imu.expected_g` | 1.0 | 0.9 - 1.1 | Expected gravity magnitude (g) |
| `imu.gravity_tol` | 0.15 | 0.05 - 0.3 | Gravity reading tolerance (g) |

**Tuning Notes:**
- If calibration fails, increase `gravity_tol` or ensure aircraft is perfectly still
- `gyro_bias_max`: Gyros with bias > this after calibration indicate a bad sensor

---

## Failsafe Configuration

These settings control aircraft behavior when RC link is lost.

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `fs.bank.deg` | 7.0 | 0.0 - 30.0 | Bank angle during failsafe (degrees) |
| `fs.pitch.deg` | -3.0 | -20.0 - 0.0 | Pitch angle during failsafe (degrees) |
| `fs.throttle` | 0.0 | 0.0 - 1.0 | Throttle setting during failsafe |
| `fs.min.lq.arm` | 50 | 20 - 100 | Minimum link quality % to arm |

**Failsafe Behavior:**
When RC link is lost, the aircraft:
1. Switches to ATTITUDE_MODE
2. Banks to `fs.bank.deg` (creates a gentle spiral)
3. Pitches to `fs.pitch.deg` (slight nose-down for controlled descent)
4. Cuts throttle to `fs.throttle` (default 0 = engine off)

**Tuning Notes:**
- **bank.deg**: 5-10° creates a contained spiral; 0° = straight glide
- **pitch.deg**: -3° to -5° maintains airspeed without steep dive
- **throttle**: Set to 0 to cut engine (safest); set higher if you want powered descent
- **min.lq.arm**: Prevents arming with weak link; 50% is conservative

---

## CRSF Receiver

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `crsf.tri.low` | 0.33 | 0.1 - 0.4 | TriState switch low threshold |
| `crsf.tri.high` | 0.66 | 0.6 - 0.9 | TriState switch high threshold |

**Tuning Notes:**
- These thresholds determine how 3-position switch values are interpreted
- Input < `tri.low` = position 1 (e.g., ATTITUDE_MODE)
- Input > `tri.high` = position 3 (e.g., MANUAL_MODE)
- Otherwise = position 2 (e.g., RATE_MODE)

---

## System

| Key | Default | Range | Description |
|-----|---------|-------|-------------|
| `sys.aircraft.name` | "ArduFlite" | string | Aircraft name for telemetry display |

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

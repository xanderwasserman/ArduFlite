# Flight Log FL001 — First Powered Flight

| Field         | Value                        |
|---------------|------------------------------|
| **Log ID**    | FL001                        |
| **Date**      | 1 March 2026                 |
| **Duration**  | < 5 minutes                  |
| **Outcome**   | Soft crash — broken prop     |
| **Firmware**  | ArduFlite v1.1.0.            |

---

## Conditions

- Hand launch
- Light breeze with 15km/h gusts wind conditions
- Multiple laps at varying throttle settings

---

## Flight Summary

### Launch
- Hand launch successful. Plane was stable immediately on release with no corrective input required.

### Lap 1 — High Power
- Plane was responsive and stable overall.
- Observed small, fast pitch oscillations throughout. Roll and yaw axes were solid.

### Lap 2 — ~50% Throttle
- Responsiveness remained good.
- Limited bank and pitch angle limits made completing turns more difficult — aircraft needed more room to manoeuvre.

### Lap 3 — Reduced Throttle (Stall Investigation)
- Plane struggled to complete turns at low throttle.
- Lost altitude in turns, leading to a soft crash on a turn.
- **Broken prop** — no spare available, flight ended.

---

## Observations

| Axis  | Assessment                                     |
|-------|------------------------------------------------|
| Roll  | ✅ Stable and responsive                        |
| Pitch | ⚠️ Small, fast oscillations observed on all laps |
| Yaw   | ✅ Stable                                       |

- Pitch oscillations were consistent across all throttle settings, pointing to a rate controller tuning issue rather than an aerodynamic one.
- Bank and pitch angle limits felt restrictive during turns — increasing mixer limits would give more authority.
- Stall speed is higher than expected at lower throttle in banked turns — coordinated pitch-from-roll mixing may help prevent nose-drop.

---

## Tuning Actions

### 1. Pitch Rate Oscillations
Small, fast oscillations indicate the pitch rate `kp` is too high. `td` bumped from 0.35 → 0.45 to restore crispness after `kp` reduction.

| Parameter       | Current | Recommended | Rationale                                      |
|-----------------|---------|-------------|------------------------------------------------|
| `rate.pitch.kp` | 0.06    | 0.04        | Reduce P gain ~30% to dampen fast oscillations |
| `rate.pitch.td` | 0.35    | 0.45        | Increased to restore crispness after kp reduction |

```bash
config set rate.pitch.kp 0.04
config set rate.pitch.td 0.45
config save
```

### 2. Attitude Mixer Limits (Roll & Pitch)
Increase max bank and pitch angles to give more turn authority.

| Parameter          | Current | Recommended | Rationale                          |
|--------------------|---------|-------------|------------------------------------|
| `mix.max.att.roll` | 45.0°   | 55.0°       | More roll authority in turns       |
| `mix.max.att.pitch` | 45.0°  | 50.0°       | More pitch authority in climbs     |

```bash
config set mix.max.att.roll 55.0
config set mix.max.att.pitch 50.0
config save
```

### 3. Coordinated Turn Mixing (Optional — address altitude loss in turns)
Increase `mix.pitch.from.roll` slightly to add automatic up-elevator when banking, preventing nose-drop.

| Parameter             | Current | Recommended | Rationale                                      |
|-----------------------|---------|-------------|------------------------------------------------|
| `mix.pitch.from.roll` | 0.05    | 0.08        | More nose-up compensation during banked turns  |

```bash
config set mix.pitch.from.roll 0.08
config save
```

---

## Next Flight Priorities

1. Verify pitch oscillations are resolved after `rate.pitch.kp` reduction.
2. Confirm roll and pitch limits feel comfortable with increased mixer values.
3. Confirm the plane holds altitude through turns with updated pitch-from-roll mixing.
4. Bring a spare prop.

---

## Hardware Notes

- Prop broken in soft crash landing. Replace before next flight.

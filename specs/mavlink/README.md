# ArduFlite MAVLink Telemetry

**Status:** Implemented (M0–M5), bench verification pending · **Date:** 2026-10-04 · **Branch:** `feature/hal-refactor`

One telemetry implementation that serves both the USB port on the bench and a
UART-attached radio in the air, speaking MAVLink 2 so QGroundControl (and other
ground stations) can display, record and tune the aircraft.

**Flying:** MAVLink is merged into the HAL refactor and tested together with
it. Without `mav.uart.enabled` and with no ground station on USB, no MAVLink
task runs, so a flight without a ground station exercises none of it.

---

## Decisions

| # | Decision | Source |
|---|---|---|
| D1 | On USB, the CLI command `mavlink on` hands the port to MAVLink **until reboot**. Logs then travel as `STATUSTEXT`, shown in QGC's message panel | Maintainer |
| D2 | Commands from the ground take the CLI's paths (`ConfigRegistry::set()`, `CommandSystem`) and the CLI's ground-safety rule | Maintainer |
| D3 | Generic QGC support first. A fully featured GCS integration is a later, separately planned phase (M6) | Maintainer |
| D4 | **No MAVLink over ELRS.** CRSF stays the RC link, and CRSF telemetry stays what the radio's screen shows | Maintainer |
| D5 | The air link is a **LoRa radio on a UART**: on the bench prototyping setup first, then on ArduFlite FC v1 (ESP32-S3, UART0 TX 42 / RX 44, per `hardware/arduflite-fc-v1/INTERFACES.md`) | Maintainer |
| D6 | **Vendor the official MAVLink C library** (`mavlink/c_library_v2`, `common` dialect) so the whole standard protocol is available from day one | Maintainer |
| D7 | **Always send MAVLink 2; accept MAVLink 1 inbound.** Some ground stations and radios send MAVLink 1 until they see a MAVLink 2 heartbeat. No custom protocol: the protocol is an encoder behind the transport, so that stays a later option | Maintainer |
| D8 | `mavlink on` is refused while armed or in flight, like every other ground-only command | Implementation default, for review |
| D9 | The CLI also hands the console over when a ground station's MAVLink frame (valid checksum) arrives on it, under the same ground-safety rule. Opening the port from a ground station can reset the C3, depending on how it drives DTR/RTS; after a reset the board is back in the CLI, and the first heartbeat switches it again | Review finding, for review |

ADRs 065–069 in `specs/hal/07-decisions.md` record the design.

## Non-goals

* MAVLink over ELRS (D4).
* Arming, disarming or mode changes from the ground. The RC transmitter stays
  the only authority (ADR-068).
* Mission upload. A mission list request is answered with an empty list.
* Emulating another autopilot's conventions (ArduPilot/PX4). That is M6's
  decision.

---

## Architecture

```
PeriodicTelemetryBackend               task, snapshot, bounded lock
  └── MavlinkTelemetry                 one per port: runs an endpoint in a task
        └── MavlinkEndpoint            the protocol on one stream, host-testable
              ├── TelemetryMessages    TelemetryData -> MAVLink, unit conversions
              ├── MessageScheduler     per-stream intervals, byte budget
              ├── MavlinkParams        parameter protocol <-> ConfigRegistry
              ├── StatusTextQueue      log lines waiting to go out
              └── hal::ByteStream&     USB console or a UART
```

* **Transport** (ADR-065). `hal::ByteStream` is the face `device::Console` and
  `hal::Uart` share. Reads never block. `writable()` reports what a write
  accepts without blocking, and the endpoint skips a message that does not fit.
* **Endpoint.** Each `service()` pass reads and answers what has arrived, then
  sends what is due: periodic streams, queued status text, and the rest of a
  parameter list in progress. It owns all its protocol state: parser, sequence
  numbers, scheduler.
* **Console hand-over** (ADR-067). `mavlink on`, or the first valid MAVLink
  frame the CLI reads (`FrameParser`, D9), makes the CLI stop and the USB
  endpoint start. Both are refused while armed or in flight.
* **Logs** (ADR-067). `MavlinkLogRouter` wraps the console's log handler from
  boot and copies each line to every attached endpoint's queue: Info and above
  on USB, Warn and above on the radio. Lines over 50 characters are chunked.
* **Ground commands** (ADR-068). Parameter writes and reboot need the port to
  accept writes and `groundCommandBlock()` to allow them.

### Files

| Path | Contents |
|---|---|
| `src/hal/platform/ByteStream.h` | The transport interface |
| `src/third_party/mavlink/` | Vendored `c_library_v2` @ `56f6435ee725` (`VERSION`): root headers plus the `common`, `standard` and `minimal` dialects. Never edited |
| `tools/mavlink/update_vendor.sh` | The only way the vendored files change |
| `src/telemetry/mavlink/Mavlink.h` | The only include of the vendored headers (`check_layering.sh` rule 11) |
| `src/telemetry/mavlink/` | Endpoint, backend, frame parser, messages, scheduler, parameters, status text |
| `src/state/GroundSafety.h` | `groundCommandBlock()`, shared by the CLI and MAVLink |

### The vendored library

Header-only, so only the messages used reach the binary. `Mavlink.h` sets:

* `MAVLINK_ALIGNED_FIELDS 0` — fields are packed byte by byte. The default
  stores them through casts to wider types at unaligned offsets.
* `MAVLINK_COMM_NUM_BUFFERS 1` — endpoints use the `*_pack_status` and
  `mavlink_frame_char_buffer` API with their own state, so the library's
  per-channel globals are unused.

It also silences the library's warnings (`-Waddress-of-packed-member` and three
conversion warnings), so the warning checks report on ArduFlite code only. The
house rules in `check_layering.sh` skip `src/third_party/`.

Licence: the generated MAVLink C library is MIT-licensed
(https://mavlink.io/en/#license).

### Ports per board

| Board | USB (after `mavlink on`) | Telemetry UART |
|---|---|---|
| Lolin C3 Mini | USB-Serial-JTAG | UART0, RX GPIO20 / TX GPIO21. Free because the console is USB. The ROM still prints its boot banner on GPIO21, so a radio sees a burst of text at power-up; ground stations ignore it |
| FireBeetle ESP32-E | UART0 through the USB bridge | None declared yet: UART2, pins to be chosen |
| ArduFlite FC v1 (ESP32-S3) | Native USB | UART0, TX 42 / RX 44 (descriptor to be written with the board) |

---

## What is sent

| Message | Content | USB | Radio |
|---|---|---|---|
| `HEARTBEAT` | fixed wing, generic autopilot; `custom_mode` = ArduFliteMode; armed; failsafe as `MAV_STATE_CRITICAL` | 1 Hz | 1 Hz |
| `SYS_STATUS` | gyro/accel/mag/RC presence and health, battery when measured, rejected frames | 1 Hz | 0.5 Hz |
| `ATTITUDE` | roll, pitch, yaw and body rates, radians | 25 Hz | 4 Hz |
| `VFR_HUD` | heading, throttle, altitude, climb rate | 10 Hz | 2 Hz |
| `SCALED_IMU` | accel (mg), gyro (mrad/s), mag (mgauss) | 25 Hz | off |
| `NAMED_VALUE_FLOAT` | `ATT_SP_R/P/Y`, `RATE_SP_R/P/Y`, `MAG_HDG`, `MAG_UT`, `RC_LQ` | 5 Hz | off |
| `STATUSTEXT` | log lines | Info and up | Warn and up |

On request: `PARAM_VALUE`, `COMMAND_ACK`, `AUTOPILOT_VERSION` (MAVLink 2, float
parameters, C-cast encoding), `MISSION_COUNT` (always 0).

Handled inbound: `PARAM_REQUEST_LIST`, `PARAM_REQUEST_READ`, `PARAM_SET`,
`MISSION_REQUEST_LIST`, and `COMMAND_LONG` with `MAV_CMD_REQUEST_MESSAGE`,
`MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES`, `MAV_CMD_SET_MESSAGE_INTERVAL` and
`MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` (reboot autopilot). Other commands are
acknowledged as unsupported.

The radio set covers everything CRSF telemetry sends that exists today. GPS,
pressure, RC channel values and servo outputs are not in `TelemetryData` yet.
`GPS_RAW_INT`, `SCALED_PRESSURE`, `RC_CHANNELS` and `SERVO_OUTPUT_RAW` follow
when they are.

## Configuration

| Key | MAVLink name | Default | Meaning |
|---|---|---|---|
| `mav.sysid` | `MAV_SYSID` | 1 | MAVLink system ID |
| `mav.uart.enabled` | `MAV_UART_ENABLED` | false | MAVLink on the telemetry UART |
| `mav.uart.baud` | `MAV_UART_BAUD` | 57600 | Must match the radio |
| `mav.uart.max_bps` | `MAV_UART_MAX_BPS` | 4800 | Send budget, bits per second |
| `mav.uart.writes` | `MAV_UART_WRITES` | false | Accept parameter writes and reboot over the radio |

All take effect after a reboot. Parameter names for the whole schema are listed
in `docs/CONFIG_REFERENCE.md`.

---

## Bench checklists

### USB (M3)

- [ ] With the board in the CLI, open the port in QGC: it switches to MAVLink
      on its own (D9) and QGC connects; heartbeat steady at 1 Hz.
- [ ] `mavlink on` from a terminal, then QGC, also works.
- [ ] Tilt the aircraft: the HUD follows in the right sense — right wing down
      rolls right, nose up pitches up.
- [ ] Boot and runtime log lines appear in QGC's message panel.
- [ ] Reboot from QGC returns the port to the CLI.
- [ ] A `.tlog` recorded in QGC opens and plots.

### Parameters (M4)

- [ ] QGC loads the full parameter list without retries.
- [ ] Change a PID gain; the controller uses it (`NAMED_VALUE_FLOAT` and the
      behaviour); it survives a reboot.
- [ ] An out-of-range value is rejected and the old value comes back.
- [ ] Every write and the reboot are refused while armed.

### Telemetry UART (M5, prototyping setup)

- [ ] QGC over the LoRa pair shows attitude, altitude, mode and armed state.
- [ ] The measured byte rate stays under `mav.uart.max_bps`.
- [ ] With the radio unplugged or slow, the telemetry task never blocks: `stats`
      before and after enabling the port shows no change in loop overruns.
- [ ] A `PARAM_SET` over the radio is refused while `mav.uart.writes` is off.

---

## M6 — Fully featured GCS integration *(later, decision first)*

Planned once the radio link has been used for a while. The options to weigh:

1. **Stay generic, add the cheap wins:** sensor-health detail, a parameter
   metadata file so QGC shows descriptions, ranges and units.
2. **Adopt ArduPilot Plane conventions** (autopilot type, mode numbers): QGC and
   Mission Planner show plane modes, but you inherit their expectations
   (calibration flows, parameter names). Significant work, and needs the
   `ardupilotmega` dialect (1.7 MB) vendored.
3. **Mission protocol** with the mission planner, if waypoints arrive.

## Cost

Measured on the Lolin full build: +28 KB flash (1.42 MB, 67% of the app
partition), +4.3 KB static RAM. The lite build carries MAVLink too: the radio
link is a flight feature, not a web feature. Each running endpoint adds a task
with a 6 KB stack.

## Open questions

1. **Which LoRa radio, air rate and baud?** Sets `mav.uart.baud`,
   `mav.uart.max_bps` and possibly the radio stream rates.
2. **FireBeetle telemetry UART pins**, if it is the prototyping board for the
   radio.
3. **`tools/visualisation`** still reads MQTT. Point it at MAVLink (`ATTITUDE`
   via pymavlink is a few lines), or retire it now that QGC shows attitude?

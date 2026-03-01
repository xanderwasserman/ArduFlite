# ArduFlite Log Viewer

A single-file browser-based CSV viewer for ArduFlite flight logs. Supports interactive charting, column grouping, preset views, stats, and a collapsible data table — no install required.

---

## Quick Start

### Option A — Open directly in browser (zero setup)
```bash
open tools/csv_viewer/index.html
```
Works in any modern browser. All dependencies load from CDN.

### Option B — Docker (serve over HTTP)
```bash
cd tools/csv_viewer
docker compose up --build
```
Open [http://localhost:8080](http://localhost:8080).

To stop:
```bash
docker compose down
```

---

## Obtaining a Log File

Flight logs are stored in LittleFS on the ESP32. Use the WebUI or the flash dump tool to extract them.

**Via WebUI** (device must be on WiFi AP):
1. Connect to the ArduFlite WiFi AP
2. Navigate to `http://192.168.4.1` → Logs tab
3. Click **Download** next to the log you want

**Via flash dump tool:**
```bash
cd tools/flash_dump
python flash_dump.py --port /dev/tty.usbserial-* --output log.csv
```

The viewer also accepts any CSV with a `timestamp` column (milliseconds) — perfect for replaying recorded sessions.

---

## CSV Format

ArduFlite logs are comma-separated with a header row. Expected columns:

| Column | Unit | Description |
|---|---|---|
| `timestamp` | ms | Milliseconds since boot |
| `accel_x/y/z` | g | Body-frame accelerometer |
| `gyro_x/y/z` | °/s | Body-frame gyroscope |
| `quat_w/x/y/z` | — | Madgwick fusion quaternion |
| `roll` / `pitch` / `yaw` | ° | Euler angles |
| `att_sp_roll/pitch/yaw` | ° | Attitude controller setpoints |
| `rate_sp_roll/pitch/yaw` | °/s | Rate controller setpoints |
| `att_cmd_roll/pitch/yaw` | ° | Attitude controller output |
| `rate_cmd_roll/pitch/yaw` | °/s | Rate controller output |
| `altitude` | m | Barometric altitude |
| `climb_rate` | m/s | Vertical speed |
| `flight_state` | 0–3 | UNKNOWN/PREFLIGHT/INFLIGHT/LANDED |
| `flight_mode` | int | Active flight mode |

Missing or extra columns are handled gracefully — only columns present in the file are shown.

---

## Interface

### Presets
Quick-select groups of related columns:

| Preset | Columns |
|---|---|
| **Attitude** | roll, pitch, yaw + att_sp_* |
| **Inner Loop** | rate_sp_* + rate_cmd_* |
| **IMU** | accel_x/y/z + gyro_x/y/z |
| **Setpt vs Actual** | roll↔att_sp_roll, pitch↔att_sp_pitch |
| **All** | Every column |

### Chart Navigation
| Action | Effect |
|---|---|
| Scroll wheel | Zoom in/out (X axis) |
| Click + drag | Pan left/right |
| Double-click | Reset zoom |
| **Reset Zoom** button | Reset zoom |
| Hover | Tooltip showing all active values at that time |

### Sidebar
Click any column name to toggle it on/off. Group headers are collapsible. Active columns show their assigned colour; inactive columns are grey.

### Stats Bar
Displays **min / max / avg** for up to 8 active columns. Scroll horizontally if more are shown.

### Data Table
Collapsible panel at the bottom. Shows up to 500 rows. Useful for inspecting exact values around an event.

### Export PNG
Saves the current chart view as a `.png` image file.

---

## Performance Notes

- Files with more than **3 000 data points** are automatically downsampled for the chart (every Nth row). The **data table** always shows original rows.
- For long flights (> 10 min at 50 Hz), downsampling is roughly 10:1 — sufficient for trend analysis.
- If you need full-resolution analysis, use the Python tools in `tools/data_analysis/`.

---

## Dependencies (CDN)

| Library | Version |
|---|---|
| [Chart.js](https://www.chartjs.org/) | 4.4.3 |
| [Hammer.js](https://hammerjs.github.io/) | 2.0.8 |
| [chartjs-plugin-zoom](https://www.chartjs.org/chartjs-plugin-zoom/) | 2.0.1 |

No build step, no `npm install`. The HTML file is fully self-contained when served from Docker or a local HTTP server. When opening as `file://` directly, all libraries still load from CDN (requires internet access).

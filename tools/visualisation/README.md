# ArduFlite 3D Visualisation Tool

⚠️ **STATUS: Needs Update**

This tool was originally designed to receive quaternion data via MQTT for real-time 3D attitude visualization. With the removal of MQTT telemetry, it needs to be rewritten to use serial input.

## TODO

- [ ] Replace MQTT client with serial reader
- [ ] Read from `ArduFliteQSerialTelemetry` output (10 Hz quaternion stream)
- [ ] Update `DataStore` to parse serial quaternion format

## Original Functionality

- 3D aircraft model visualization
- Real-time attitude display (roll, pitch, yaw)
- 30-second rolling data window
- PyQt5 GUI with OpenGL rendering

## Dependencies

```bash
pip install -r requirements.txt
```

## Usage (when updated)

1. Enable `ArduFliteQSerialTelemetry` in ArdufliteApp.cpp
2. Connect to the ESP32 serial port
3. Run: `python main.py`

# ArduFlite 3D Visualisation Tool

⚠️ **STATUS: Needs Update**

This tool receives attitude data via MQTT, which the firmware no longer sends. To work again it needs to read MAVLink.

## TODO

- [ ] Replace the MQTT client with a MAVLink reader (pymavlink)
- [ ] Read `ATTITUDE` messages (25 Hz on USB after `mavlink on`)
- [ ] Update `DataStore` to take roll, pitch and yaw from them

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

1. Run `mavlink on` in the CLI
2. Connect to the ESP32 serial port
3. Run: `python main.py`

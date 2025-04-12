#!/usr/bin/env python3
import sys, math, csv, datetime, time
import paho.mqtt.client as mqtt
from PyQt5 import QtCore, QtWidgets, QtGui

# ----- Global Settings -----
BROKER = "localhost"
PORT = 1883

# Define the required topics and the desired CSV column order.
TOPIC_MAPPING = {
    "arduflite/quaternion/w": "quaternion_w",
    "arduflite/quaternion/x": "quaternion_x",
    "arduflite/quaternion/y": "quaternion_y",
    "arduflite/quaternion/z": "quaternion_z",
    "arduflite/accel/x": "accel_x",
    "arduflite/accel/y": "accel_y",
    "arduflite/accel/z": "accel_z",
    "arduflite/gyro/x": "gyro_x",
    "arduflite/gyro/y": "gyro_y",
    "arduflite/gyro/z": "gyro_z",
    "arduflite/orientation/pitch": "orientation_pitch",
    "arduflite/orientation/roll": "orientation_roll",
    "arduflite/orientation/yaw": "orientation_yaw",
    "arduflite/command_rate/rollCmd": "commandsRate_rollCmd",
    "arduflite/command_rate/pitchCmd": "commandsRate_pitchCmd",
    "arduflite/command_rate/yawCmd": "commandsRate_yawCmd",
    "arduflite/command_servo/rollCmd": "commands_rollCmd",
    "arduflite/command_servo/pitchCmd": "commands_pitchCmd",
    "arduflite/command_servo/yawCmd": "commands_yawCmd",
    "arduflite/flight/altitude": "flight_altitude",
    "arduflite/flight/state": "flight_state"
}

CSV_ORDER = ["timestamp", "quaternion_w", "quaternion_x", "quaternion_y", "quaternion_z",
             "accel_x", "accel_y", "accel_z",
             "gyro_x", "gyro_y", "gyro_z",
             "orientation_pitch", "orientation_roll", "orientation_yaw",
             "commandsRate_rollCmd", "commandsRate_pitchCmd", "commandsRate_yawCmd",
             "commands_rollCmd", "commands_pitchCmd", "commands_yawCmd",
             "flight_altitude", "flight_state"]

# Global storage for the latest values.
data_values = { key: 0.0 for key in CSV_ORDER if key != "timestamp" }
data_values["quaternion_w"] = 1.0  # Default for quaternion

# Global flags for connection and data reception.
mqtt_connected = False
last_message_time = 0

# ----- MQTT Client Wrapper -----
class MqttClient(QtCore.QObject):
    connectionChanged = QtCore.pyqtSignal(bool)
    dataReceived = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def connect(self):
        try:
            self.client.connect(BROKER, PORT, 60)
            self.client.loop_start()
        except Exception as e:
            print("MQTT Connection Error:", e)

    def on_connect(self, client, userdata, flags, rc):
        global mqtt_connected
        mqtt_connected = True
        self.connectionChanged.emit(True)
        print("Connected to MQTT Broker with code:", rc)
        for topic in TOPIC_MAPPING.keys():
            client.subscribe(topic)

    def on_message(self, client, userdata, msg):
        global data_values, last_message_time
        try:
            value = float(msg.payload.decode("utf-8").strip())
        except ValueError:
            return
        last_message_time = time.time()
        if msg.topic in TOPIC_MAPPING:
            key = TOPIC_MAPPING[msg.topic]
            data_values[key] = value
            self.dataReceived.emit()

# ----- Data Logger (CSV Recorder) -----
class DataLogger:
    def __init__(self):
        self.recording = False
        self.file = None
        self.writer = None

    def start(self):
        self.recording = True
        self.start_time = datetime.datetime.now()
        fname = "/home/pi/Desktop/" + self.start_time.strftime("flight_data_%Y%m%d_%H%M%S.csv")
        self.file = open(fname, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(CSV_ORDER)
        print("CSV Recording started:", fname)

    def stop(self):
        if self.file:
            self.file.close()
        self.recording = False
        print("CSV Recording stopped.")

    def record(self):
        if not self.recording or not self.writer:
            return
        t = time.time()  # float epoch timestamp
        row = [t]
        for key in CSV_ORDER[1:]:
            row.append(data_values.get(key, 0))
        self.writer.writerow(row)

# ----- Main Window (PyQt5) -----
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Flight Data Logger")
        # For touchscreen devices, starting in full screen can be useful.
        self.showFullScreen()
        # Alternatively, you can use a fixed size:
        # self.resize(800, 600)
        
        # Central widget and layout with padding for touch.
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(20)

        # Status indicators with larger font sizes for improved readability.
        self.brokerLabel = QtWidgets.QLabel("Broker: Not Connected")
        self.dataLabel = QtWidgets.QLabel("Data: No Recent Data")
        label_font = QtGui.QFont()
        label_font.setPointSize(18)
        self.brokerLabel.setFont(label_font)
        self.dataLabel.setFont(label_font)
        ledLayout = QtWidgets.QHBoxLayout()
        ledLayout.addWidget(self.brokerLabel)
        ledLayout.addWidget(self.dataLabel)
        layout.addLayout(ledLayout)

        # Logging button with bigger touch area and appropriate colors.
        self.logButton = QtWidgets.QPushButton("Start Logging")
        self.logButton.setMinimumSize(150, 100)  # Increased button size for touch
        # Initial style: green background for "start logging"
        self.logButton.setStyleSheet("background-color: green; font-size: 20px; padding: 15px;")
        layout.addWidget(self.logButton)
        self.logButton.clicked.connect(self.toggle_logging)

        # Timers for UI update and logging.
        self.uiTimer = QtCore.QTimer(self)
        self.uiTimer.timeout.connect(self.update_ui)
        self.uiTimer.start(500)

        self.logTimer = QtCore.QTimer(self)
        self.logTimer.timeout.connect(self.record_data)
        self.logTimer.start(50)

        self.dataLogger = DataLogger()
        self.mqttClient = MqttClient()
        self.mqttClient.connectionChanged.connect(self.update_connection_status)
        self.mqttClient.dataReceived.connect(self.update_data_status)
        self.mqttClient.connect()

    def toggle_logging(self):
        if not self.dataLogger.recording:
            self.dataLogger.start()
            self.logButton.setText("Stop Logging")
            # Change to red background when logging is active.
            self.logButton.setStyleSheet("background-color: red; font-size: 20px; padding: 15px;")
        else:
            self.dataLogger.stop()
            self.logButton.setText("Start Logging")
            # Change to green background when logging has stopped.
            self.logButton.setStyleSheet("background-color: green; font-size: 20px; padding: 15px;")

    def record_data(self):
        if self.dataLogger.recording:
            self.dataLogger.record()

    def update_ui(self):
        self.update_connection_status(mqtt_connected)
        self.update_data_status()

    def update_connection_status(self, connected):
        if connected:
            self.brokerLabel.setText("Broker: Connected")
            self.brokerLabel.setStyleSheet("color: green;")
        else:
            self.brokerLabel.setText("Broker: Not Connected")
            self.brokerLabel.setStyleSheet("color: red;")

    def update_data_status(self):
        if time.time() - last_message_time < 2:
            self.dataLabel.setText("Data: Received")
            self.dataLabel.setStyleSheet("color: blue;")
        else:
            self.dataLabel.setText("Data: No Recent Data")
            self.dataLabel.setStyleSheet("color: gray;")

# ----- Main -----
def main():
    app = QtWidgets.QApplication(sys.argv)
    # Force a style that respects the stylesheet (Fusion is a common choice).
    app.setStyle("Fusion")
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()

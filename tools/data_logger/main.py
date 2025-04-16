#!/usr/bin/env python3
import sys, math, csv, datetime, time
import paho.mqtt.client as mqtt
from PyQt5 import QtCore, QtWidgets, QtGui

# ----- Global Settings -----
BROKER = "localhost"
PORT = 1883

# Define the required topics and the desired CSV column order.
TOPIC_MAPPING = {
    "arduflite/imu/quaternion/w": "quaternion_w",
    "arduflite/imu/quaternion/x": "quaternion_x",
    "arduflite/imu/quaternion/y": "quaternion_y",
    "arduflite/imu/quaternion/z": "quaternion_z",
    "arduflite/imu/accel/x": "accel_x",
    "arduflite/imu/accel/y": "accel_y",
    "arduflite/imu/accel/z": "accel_z",
    "arduflite/imu/gyro/x": "gyro_x",
    "arduflite/imu/gyro/y": "gyro_y",
    "arduflite/imu/gyro/z": "gyro_z",
    "arduflite/imu/orientation/pitch": "orientation_pitch",
    "arduflite/imu/orientation/roll": "orientation_roll",
    "arduflite/imu/orientation/yaw": "orientation_yaw",
    "arduflite/imu/controller/rate/roll": "attitude_controller_out_roll",
    "arduflite/controller/rate/pitch": "attitude_controller_out_pitch",
    "arduflite/controller/rate/yaw": "attitude_controller_out_yaw",
    "arduflite/controller/attitude/roll": "rate_controller_out_roll",
    "arduflite/ccontroller/attitude/pitch": "rate_controller_out_pitch",
    "arduflite/controller/attitude/yaw": "rate_controller_out_yaw",
    "arduflite/imu/barometer/altitude": "flight_altitude",
    "arduflite/imu/flight/state": "flight_state"
}

CSV_ORDER = ["timestamp", "quaternion_w", "quaternion_x", "quaternion_y", "quaternion_z",
             "accel_x", "accel_y", "accel_z",
             "gyro_x", "gyro_y", "gyro_z",
             "orientation_pitch", "orientation_roll", "orientation_yaw",
             "attitude_controller_out_roll", "attitude_controller_out_pitch", "attitude_controller_out_yaw",
             "rate_controller_out_roll", "rate_controller_out_pitch", "rate_controller_out_yaw",
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
        self.logButton.setMinimumSize(100, 75)
        self.logButton.setStyleSheet("background-color: green; font-size: 20px; padding: 15px;")
        layout.addWidget(self.logButton)
        self.logButton.clicked.connect(self.toggle_logging)

        # Create a horizontal layout for the command buttons.
        commandLayout = QtWidgets.QHBoxLayout()
        
        self.resetButton = QtWidgets.QPushButton("Reset")
        self.resetButton.setMinimumSize(100, 50)
        self.resetButton.setStyleSheet("background-color: lightgray; font-size: 18px; padding: 10px;")
        self.resetButton.clicked.connect(self.publish_reset)
        commandLayout.addWidget(self.resetButton)
        
        self.calibrateButton = QtWidgets.QPushButton("Calibrate")
        self.calibrateButton.setMinimumSize(100, 50)
        self.calibrateButton.setStyleSheet("background-color: lightgray; font-size: 18px; padding: 10px;")
        self.calibrateButton.clicked.connect(self.publish_calibrate)
        commandLayout.addWidget(self.calibrateButton)
        
        layout.addLayout(commandLayout)
        
        # Add a toggle button for switching modes ("Assist" and "Stabilised").
        self.modeToggleButton = QtWidgets.QPushButton("Assist")
        self.modeToggleButton.setMinimumSize(100, 50)
        # Initial style for Assist mode.
        self.modeToggleButton.setStyleSheet("background-color: lightblue; font-size: 18px; padding: 10px;")
        self.modeToggleButton.clicked.connect(self.toggle_mode)
        layout.addWidget(self.modeToggleButton)
        # Current mode state. "Assist" publishes payload 1; "Stabilised" publishes payload 2.
        self.currentMode = "Assist"

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
            self.logButton.setStyleSheet("background-color: red; font-size: 20px; padding: 15px;")
        else:
            self.dataLogger.stop()
            self.logButton.setText("Start Logging")
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
    
    def publish_reset(self):
        self.mqttClient.client.publish("arduflite/command/reset", "1")
        print("Reset command published.")
    
    def publish_calibrate(self):
        self.mqttClient.client.publish("arduflite/command/calibrate", "1")
        print("Calibrate command published.")
    
    def toggle_mode(self):
        # Toggle between "Assist" and "Stabilised"
        if self.currentMode == "Assist":
            # Change to Stabilised mode.
            self.currentMode = "Stabilised"
            self.modeToggleButton.setText("Stabilised")
            self.modeToggleButton.setStyleSheet("background-color: yellow; font-size: 18px; padding: 10px;")
            self.mqttClient.client.publish("arduflite/command/mode", "2")
            print("Mode changed to Stabilised, published payload 2.")
        else:
            # Change to Assist mode.
            self.currentMode = "Assist"
            self.modeToggleButton.setText("Assist")
            self.modeToggleButton.setStyleSheet("background-color: blue; font-size: 18px; padding: 10px;")
            self.mqttClient.client.publish("arduflite/command/mode", "1")
            print("Mode changed to Assist, published payload 1.")

# ----- Main -----
def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()

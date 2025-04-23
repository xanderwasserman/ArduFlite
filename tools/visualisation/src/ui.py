from PyQt5 import QtWidgets, QtCore
from .visualisation import AircraftVisualizer
from .graph_plotter import GraphPlotter, MultiPlotter
from .csv_recorder import CSVRecorder

class MainWindow(QtWidgets.QMainWindow):
    """
    MainWindow integrates:
      - 3D aircraft visualizer
      - Combined & separate plots
      - Text status (flight state & mode)
      - CSV recording and command buttons
    """
    STATE_MAP = {0: "Unknown", 1: "Pre-flight", 2: "In-flight", 10: "Landed"}
    MODE_MAP  = {0: "Assist",   1: "Stabilised"}

    def __init__(self, data_store, mqtt_client):
        super().__init__()
        self.setWindowTitle("Flight Controller Visualization")
        self.data_store   = data_store
        self.mqtt_client  = mqtt_client
        self.csv_recorder = CSVRecorder(data_store)

        self.init_ui()

        # 20 Hz data logging
        self.log_timer = QtCore.QTimer(self)
        self.log_timer.timeout.connect(self.log_data)
        self.log_timer.start(50)

        # 5 Hz label updates
        self.ui_timer = QtCore.QTimer(self)
        self.ui_timer.timeout.connect(self.update_status_labels)
        self.ui_timer.start(200)

    def init_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        grid = QtWidgets.QGridLayout(central)
        grid.setContentsMargins(10, 10, 10, 10)
        grid.setSpacing(8)

        # Set column stretches: center columns get more space
        for col in (2, 3, 4):
            grid.setColumnStretch(col, 2)
        for col in (0, 1, 5):
            grid.setColumnStretch(col, 1)
        grid.setRowStretch(7, 1)

        # --- GroupBox: Sensors ---
        sensor_box = QtWidgets.QGroupBox("Sensors")
        sensor_layout = QtWidgets.QVBoxLayout(sensor_box)
        # Accel & Gyro multi-plots side by side
        accel = MultiPlotter(self.data_store, [
            {"source":"imu","category":"accel","variable":"x","name":"Accel X","pen":"r"},
            {"source":"imu","category":"accel","variable":"y","name":"Accel Y","pen":"g"},
            {"source":"imu","category":"accel","variable":"z","name":"Accel Z","pen":"b"},
        ])
        accel.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        gyro = MultiPlotter(self.data_store, [
            {"source":"imu","category":"gyro","variable":"x","name":"Gyro X","pen":"r"},
            {"source":"imu","category":"gyro","variable":"y","name":"Gyro Y","pen":"g"},
            {"source":"imu","category":"gyro","variable":"z","name":"Gyro Z","pen":"b"},
        ])
        gyro.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        h_split = QtWidgets.QHBoxLayout()
        h_split.addWidget(accel)
        h_split.addWidget(gyro)
        sensor_layout.addLayout(h_split)
        grid.addWidget(sensor_box, 0, 0, 2, 2)

        # --- GroupBox: Attitude Loop ---
        att_box = QtWidgets.QGroupBox("Attitude Loop")
        att_layout = QtWidgets.QGridLayout(att_box)
        for i, axis in enumerate(("roll","pitch","yaw")):
            # orientation vs setpoint
            mp = MultiPlotter(self.data_store, [
                {"source":"imu","category":"orientation","variable":axis,"name":f"Orient {axis}","pen":"b"},
                {"source":"controller-setpoint","category":"attitude","variable":axis,"name":f"SP {axis}","pen":"g"},
            ])
            mp.setTitle(f"Attitude {axis.capitalize()}")
            mp.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            att_layout.addWidget(mp, 0, i)
            # controller output separate
            gp = GraphPlotter(self.data_store, "controller", "attitude", axis)
            gp.setTitle(f"Cmd_{axis}")
            gp.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            att_layout.addWidget(gp, 1, i)
        grid.addWidget(att_box, 0, 2, 2, 3)

        # --- GroupBox: Rate Loop ---
        rate_box = QtWidgets.QGroupBox("Rate Loop")
        rate_layout = QtWidgets.QGridLayout(rate_box)
        for i, axis in enumerate(("roll","pitch","yaw")):
            mp = MultiPlotter(self.data_store, [
                {"source":"imu","category":"gyro","variable":axis,"name":f"Gyro {axis}","pen":"b"},
                {"source":"controller-setpoint","category":"rate","variable":axis,"name":f"SP {axis}","pen":"g"},
            ])
            mp.setTitle(f"Rate {axis.capitalize()}")
            mp.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            rate_layout.addWidget(mp, 0, i)
            gp = GraphPlotter(self.data_store, "controller", "rate", axis)
            gp.setTitle(f"CmdRate_{axis}")
            gp.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            rate_layout.addWidget(gp, 1, i)
        grid.addWidget(rate_box, 2, 2, 2, 3)

        # --- 3D Visualizer ---
        self.visualizer = AircraftVisualizer(self.data_store)
        self.visualizer.set_axis_inversion(-1, 1, -1)
        grid.addWidget(self.visualizer, 4, 2, 4, 3)

        # --- GroupBox: Flight Status ---
        status_box = QtWidgets.QGroupBox("Flight Status")
        status_layout = QtWidgets.QVBoxLayout(status_box)
        alt = GraphPlotter(self.data_store, "imu", "barometer", "altitude")
        alt.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        status_layout.addWidget(alt)
        self.state_label = QtWidgets.QLabel("State: Unknown")
        self.mode_label  = QtWidgets.QLabel("Mode: Unknown")
        font = self.state_label.font()
        font.setPointSize(12)
        self.state_label.setFont(font)
        self.mode_label.setFont(font)
        status_layout.addWidget(self.state_label)
        status_layout.addWidget(self.mode_label)
        grid.addWidget(status_box, 4, 5, 2, 2)

        # --- Control Buttons ---
        btn_box = QtWidgets.QGroupBox("Controls")
        btn_layout = QtWidgets.QHBoxLayout(btn_box)
        self.record_button = QtWidgets.QPushButton("Start Recording")
        self.record_button.setCheckable(True)
        self.record_button.toggled.connect(self.toggle_recording)
        btn_layout.addWidget(self.record_button)
        self.reset_button = QtWidgets.QPushButton("Reset")
        self.reset_button.clicked.connect(self.send_reset_command)
        btn_layout.addWidget(self.reset_button)
        self.calibrate_button = QtWidgets.QPushButton("Calibrate")
        self.calibrate_button.clicked.connect(self.send_calibrate_command)
        btn_layout.addWidget(self.calibrate_button)
        grid.addWidget(btn_box, 7, 2, 1, 4)

    def update_status_labels(self):
        st = self.data_store.get_latest("imu","flight","state") or 0
        md = self.data_store.get_latest("imu","flight","mode")  or 0
        self.state_label.setText(f"State: {self.STATE_MAP.get(st,'?')} ({st})")
        self.mode_label .setText(f"Mode:  {self.MODE_MAP .get(md,'?')} ({md})")

    def toggle_recording(self, checked: bool):
        if checked:
            self.csv_recorder.start_recording()
            self.record_button.setText("Stop Recording")
            self.record_button.setStyleSheet("background-color: red;")
        else:
            self.csv_recorder.stop_recording()
            self.record_button.setText("Start Recording")
            self.record_button.setStyleSheet("background-color: green;")

    def send_reset_command(self):
        self.mqtt_client.publish_command("arduflite/command/reset", "1")

    def send_calibrate_command(self):
        self.mqtt_client.publish_command("arduflite/command/calibrate", "1")

    def log_data(self):
        self.csv_recorder.log_data()

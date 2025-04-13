from PyQt5 import QtWidgets, QtCore
from .visualisation import AircraftVisualizer
from .graph_plotter import GraphPlotter
from .csv_recorder import CSVRecorder

class MainWindow(QtWidgets.QMainWindow):
    """
    MainWindow integrates the 3D aircraft visualizer, the real-time graphs, and CSV recording controls.
    """
    def __init__(self, data_store, mqtt_client):
        super(MainWindow, self).__init__()
        self.setWindowTitle("Flight Controller Visualization")
        self.data_store = data_store
        self.mqtt_client = mqtt_client
        self.csv_recorder = CSVRecorder(data_store)

        self.init_ui()

        # Timer to log data periodically (20Hz)
        self.log_timer = QtCore.QTimer()
        self.log_timer.timeout.connect(self.log_data)
        self.log_timer.start(50)

    def init_ui(self):
        """
        Set up the user interface layout, placing the 3D view in the center
        and the graphs around the edges in a 7×7 grid.
        """
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        
        # Create a 7×7 grid layout.
        layout = QtWidgets.QGridLayout(central_widget)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(5)
        
        # Place the 3D Aircraft Visualizer in the center: rows 2-4 and columns 2-4.
        self.visualizer = AircraftVisualizer(self.data_store)
        self.visualizer.set_axis_inversion(-1, 1, -1)
        # The arguments here mean: start at row 1, column 2; span 3 rows and 3 columns.
        layout.addWidget(self.visualizer, 1, 2, 5, 3)
        
        # # Define positions for graph widgets around the central visualizer.
        gp = GraphPlotter(self.data_store, "orientation", "pitch")
        layout.addWidget(gp, 0, 2)
        gp = GraphPlotter(self.data_store, "orientation", "roll")
        layout.addWidget(gp, 0, 3)
        gp = GraphPlotter(self.data_store, "orientation", "yaw")
        layout.addWidget(gp, 0, 4)
        
        gp = GraphPlotter(self.data_store, "flight", "state")
        layout.addWidget(gp, 6, 2)
        gp = GraphPlotter(self.data_store, "barometer", "altitude")
        layout.addWidget(gp, 6, 3)
        
        gp = GraphPlotter(self.data_store, "accel", "x")
        layout.addWidget(gp, 0, 0)
        gp = GraphPlotter(self.data_store, "accel", "y")
        layout.addWidget(gp, 1, 0)
        gp = GraphPlotter(self.data_store, "accel", "z")
        layout.addWidget(gp, 2, 0)
        
        gp = GraphPlotter(self.data_store, "gyro", "x")
        layout.addWidget(gp, 0, 1)
        gp = GraphPlotter(self.data_store, "gyro", "y")
        layout.addWidget(gp, 1, 1)
        gp = GraphPlotter(self.data_store, "gyro", "z")
        layout.addWidget(gp, 2, 1)
        
        layout.setRowMinimumHeight(3, 100)  # This sets row 3 to be at least 100 pixels tall.
        
        gp = GraphPlotter(self.data_store, "attitude", "roll")
        layout.addWidget(gp, 4, 0)
        gp = GraphPlotter(self.data_store, "attitude", "pitch")
        layout.addWidget(gp, 5, 0)
        gp = GraphPlotter(self.data_store, "attitude", "yaw")
        layout.addWidget(gp, 6, 0)
        
        gp = GraphPlotter(self.data_store, "rate", "roll")
        layout.addWidget(gp, 4, 1)
        gp = GraphPlotter(self.data_store, "rate", "pitch")
        layout.addWidget(gp, 5, 1)
        gp = GraphPlotter(self.data_store, "rate", "yaw")
        layout.addWidget(gp, 6, 1)
        
        # CSV Recording Button. Adjust its location if necessary.
        self.record_button = QtWidgets.QPushButton("Start Recording")
        self.record_button.setCheckable(True)
        self.record_button.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.record_button.setStyleSheet("background-color: green; font-size: 16px;")
        self.record_button.toggled.connect(self.toggle_recording)
        layout.addWidget(self.record_button, 6, 5)
        
        # Create a Reset button.
        self.reset_button = QtWidgets.QPushButton("Reset")
        self.reset_button.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.reset_button.setStyleSheet("font-size: 16px;")
        self.reset_button.clicked.connect(self.send_reset_command)
        layout.addWidget(self.reset_button, 5, 5)  # Position at row 6, column 5
        
        # Create a Calibrate button.
        self.calibrate_button = QtWidgets.QPushButton("Calibrate")
        self.calibrate_button.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.calibrate_button.setStyleSheet("font-size: 16px;")
        self.calibrate_button.clicked.connect(self.send_calibrate_command)
        layout.addWidget(self.calibrate_button, 4, 5)

    def toggle_recording(self, checked):
        """
        Toggle CSV recording on or off.
        """
        if checked:
            self.csv_recorder.start_recording() 
            self.record_button.setStyleSheet("background-color: red; font-size: 16px;")
        else:
            self.csv_recorder.stop_recording()
            self.record_button.setText("Start Recording")
            self.record_button.setStyleSheet("background-color: green; font-size: 16px;")
            
    def send_reset_command(self):
        """
        Publish an MQTT message to reset the flight controller.
        """
        # Adjust the topic string as required by your system.
        command_topic = "arduflite/command/reset"
        self.mqtt_client.publish_command(command_topic, "1")
        print("Reset command sent.")

    def send_calibrate_command(self):
        """
        Publish an MQTT message to calibrate the flight controller.
        """
        # Adjust the topic string as required by your system.
        command_topic = "arduflite/command/calibrate"
        self.mqtt_client.publish_command(command_topic, "1")
        print("Calibrate command sent.")

    def log_data(self):
        """
        Log the current data if recording is enabled.
        """
        self.csv_recorder.log_data()

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
        and the graphs around the edges.
        """
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        
        # Create a 5×5 grid layout
        layout = QtWidgets.QGridLayout(central_widget)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(5)
        
        # 3D Aircraft Visualizer in the center (rows 1..3, cols 1..3)
        self.visualizer = AircraftVisualizer(self.data_store)
        self.visualizer.set_axis_inversion(-1, 1, -1)
        layout.addWidget(self.visualizer, 1, 1, 3, 3)
        
        # Prepare the list of "edge" positions around the 5×5 center:
        edge_positions = [
            # Top row (left to right)
            (0, 1), (0, 2), (0, 3), 
            # Right column (top to bottom, skipping corners)
            (1, 4), (2, 4), (3, 4),
            # Bottom row (right to left)
            (4, 3), (4, 2), (4, 1),
            # Left column (bottom to top)
            (3, 0), (2, 0), (1, 0)
        ]
        
        # Create graphs for accel, gyro, orientation, and commands.
        # We'll place them in order around the edge positions.
        self.graphs = []
        categories = ["accel", "gyro", "orientation", "command_servo"]
        edge_iter = iter(edge_positions)
        
        for category in categories:
            for var in self.data_store.data[category]:
                gp = GraphPlotter(self.data_store, category, var)
                # Get the next available edge cell
                pos = next(edge_iter, None)
                if pos is None:
                    # In case we run out of edge positions (unlikely with 12 graphs)
                    # Just place the graph in the bottom row somewhere.
                    layout.addWidget(gp, 4, 0)
                else:
                    layout.addWidget(gp, pos[0], pos[1])
                self.graphs.append(gp)
        
        # CSV Recording Button in the bottom center (row=4, col=2 for example)
        self.record_button = QtWidgets.QPushButton("Start Recording")
        self.record_button.setCheckable(True)
        self.record_button.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.record_button.setStyleSheet("background-color: green; font-size: 16px;")
        self.record_button.toggled.connect(self.toggle_recording)
        layout.addWidget(self.record_button, 4, 4)

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

    def log_data(self):
        """
        Log the current data if recording is enabled.
        """
        self.csv_recorder.log_data()

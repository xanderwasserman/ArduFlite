from pyqtgraph.Qt import QtCore
import pyqtgraph as pg
import time

class GraphPlotter(pg.PlotWidget):
    """
    GraphPlotter displays a real-time time-series graph for a specific flight variable.
    """
    def __init__(self, data_store, category, variable, parent=None):
        super(GraphPlotter, self).__init__(parent)
        self.data_store = data_store
        self.category = category
        self.variable = variable
        self.setTitle(f"{category} {variable}")
        self.setLabel('left', self.variable)
        self.setLabel('bottom', 'Time (s)')
        self.curve = self.plot(pen='y')
        
        # Set fixed y-axis range depending on the category:
        if self.category in ["accel", "commands"]:
            self.setYRange(-1, 1)
            self.enableAutoRange(axis='y', enable=False)
        elif self.category == "orientation":
            self.setYRange(-90, 90)
            self.enableAutoRange(axis='y', enable=False)
        elif self.category == "gyro":
            self.setYRange(-180, 180)
            self.enableAutoRange(axis='y', enable=False)
        else:
            # For other categories (e.g. gyro) you might allow auto range.
            self.enableAutoRange(axis='y', enable=True)
        
        # Timer for updating the plot every 100 ms
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)

    def update_plot(self):
        """
        Retrieve historical data and update the plot.
        """
        key = f"{self.category}_{self.variable}"
        history = list(self.data_store.history[key])
        if history:
            times, values = zip(*history)
            # Convert timestamps to relative time (seconds from current time)
            current_time = time.time()
            times = [t - current_time for t in times]
            self.curve.setData(times, values)

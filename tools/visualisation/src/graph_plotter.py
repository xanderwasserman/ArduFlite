from pyqtgraph.Qt import QtCore
import pyqtgraph as pg
import time

class GraphPlotter(pg.PlotWidget):
    def __init__(self, data_store, source, category, variable, parent=None):
        super().__init__(parent)
        self.data_store = data_store
        self.source     = source
        self.category   = category
        self.variable   = variable

        self.setTitle(f"{source} / {category} / {variable}")
        self.setLabel('left', variable)
        self.setLabel('bottom', 'Time Ago (s)')
        self.curve = self.plot(pen='y')

        # Example of data-driven Y ranges
        Y_RANGES = {
            ("imu","accel"):      (-2, 2),
            ("imu","gyro"):       (-300, 300),
            ("imu","orientation"):(-90, 90),
            ("controller","rate"):    (-1, 1),
            ("controller","attitude"):(-70, 70),
        }
        rng = Y_RANGES.get((source, category))
        if rng:
            self.setYRange(*rng)
            self.enableAutoRange(axis='y', enable=False)
        else:
            self.enableAutoRange(axis='y', enable=True)

        # Update every 100 ms
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)

    def update_plot(self):
        # 1) Pull history via DataStore API
        history = self.data_store.get_history(
            self.source, self.category, self.variable
        )
        if not history:
            return

        # 2) Split times/values and convert to seconds ago
        times, values = zip(*history)
        now = time.monotonic()
        times = [now - t for t in times]

        # 3) (Optional) downsample if too many points
        if len(times) > 1000:
            factor = len(times) // 1000
            times  = times[::factor]
            values = values[::factor]

        # 4) Update the curve
        self.curve.setData(times, values)

class MultiPlotter(pg.PlotWidget):
    """
    MultiPlotter allows plotting multiple series on one time-series graph.
    Each series is defined by a dict:
      {source, category, variable, name, pen}
    You can optionally pass y_range=(min, max) to fix the Y axis.
    """
    # Default y-ranges by (source, category)
    _Y_RANGES = {
        ("imu","accel"):      (-2, 2),
        ("imu","gyro"):       (-300, 300),
        ("imu","orientation"):(-90, 90),
        ("controller","rate"):    (-1, 1),
        ("controller","attitude"):(-70, 70),
    }

    def __init__(self, data_store, series, y_range=None, parent=None):
        """
        :param data_store: instance of DataStore
        :param series: list of dicts with keys:
                       source, category, variable, name, pen
        :param y_range: optional tuple (ymin, ymax) to fix Y axis
        """
        super().__init__(parent)
        self.data_store = data_store
        self.series     = series

        self.setLabel('left', 'Value')
        self.setLabel('bottom', 'Time Ago (s)')

        # Plot each series and store its curve
        self.curves = []
        self.legend = self.addLegend(offset=(10,10))
        for s in self.series:
            curve = self.plot([], [], pen=s.get('pen', 'w'), name=s.get('name'))
            self.curves.append(curve)

        # Decide Y range: explicit y_range > built-in mapping > auto
        if y_range is not None:
            self.setYRange(*y_range)
            self.enableAutoRange(axis='y', enable=False)
        else:
            # look at first series for default mapping
            src = self.series[0]['source']
            cat = self.series[0]['category']
            rng = MultiPlotter._Y_RANGES.get((src, cat))
            if rng:
                self.setYRange(*rng)
                self.enableAutoRange(axis='y', enable=False)
            else:
                self.enableAutoRange(axis='y', enable=True)

        # Update every 100 ms
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)

    def update_plot(self):
        now = time.monotonic()
        for idx, s in enumerate(self.series):
            history = self.data_store.get_history(
                s['source'], s['category'], s['variable']
            )
            if not history:
                # clear if no data
                self.curves[idx].setData([], [])
                continue

            times, values = zip(*history)
            times = [now - t for t in times]

            # downsample if over 1000 points
            if len(times) > 1000:
                factor = len(times) // 1000
                times  = times[::factor]
                values = values[::factor]

            self.curves[idx].setData(times, values)
import time
from collections import deque

class DataStore:
    """
    DataStore holds the latest values and historical data for all flight variables.
    It maintains a 30-second rolling window for each variable.
    """
    def __init__(self, window_seconds=30):
        self.window_seconds = window_seconds
        # Latest data for each category and variable
        self.data = {
            "quaternion": {"w": None, "x": None, "y": None, "z": None},
            "accel": {"x": None, "y": None, "z": None},
            "gyro": {"x": None, "y": None, "z": None},
            "orientation": {"pitch": None, "roll": None, "yaw": None},
            "attitude": {"roll": None, "pitch": None, "yaw": None},
            "rate": {"roll": None, "pitch": None, "yaw": None},
            "flight": {"state": None},
            "barometer": {"altitude": None},
        }
        # Historical data stored as deque for each variable with (timestamp, value) tuples
        self.history = {}
        for category, variables in self.data.items():
            for var in variables:
                self.history[f"{category}_{var}"] = deque()

    def update(self, category, variable, value):
        """
        Update the current value and append to the history with current timestamp.
        """
        self.data[category][variable] = value
        ts = time.time()
        key = f"{category}_{variable}"
        self.history[key].append((ts, value))
        # Remove old data outside the rolling window
        while self.history[key] and (ts - self.history[key][0][0]) > self.window_seconds:
            self.history[key].popleft()

import time
from collections import deque
from threading import Lock
from typing import Deque, Dict, Tuple, Optional, List, Any

class DataStore:
    """
    Holds the most-recent value and a rolling-window history for each flight variable.
    """

    def __init__(self, window_seconds: float = 30.0, max_rate_hz: float = 50.0):
        self.window_seconds = window_seconds
        # Estimate max samples = window * rate
        max_samples = int(window_seconds * max_rate_hz)

        # Live snapshot of every leaf variable
        self.data: Dict[str, Dict[str, Dict[str, Any]]] = {
            "imu": {
                "quaternion": dict.fromkeys(("w","x","y","z"), None),
                "accel":      dict.fromkeys(("x","y","z"),       None),
                "gyro":       dict.fromkeys(("x","y","z"),       None),
                "orientation":dict.fromkeys(("pitch","roll","yaw"), None),
                "flight":     {"state": None},
                "barometer":  {"altitude": None},
            },
            "controller": {
                "rate":     dict.fromkeys(("roll","pitch","yaw"), None),
                "attitude": dict.fromkeys(("roll","pitch","yaw"), None),
            },
            "controller-setpoint": {
                "rate":     dict.fromkeys(("roll","pitch","yaw"), None),
                "attitude": dict.fromkeys(("roll","pitch","yaw"), None),
            }
        }

        # History deques for each variable, keyed by "source_category_variable"
        self.history: Dict[str, Deque[Tuple[float, Any]]] = {}
        for src, cats in self.data.items():
            for cat, vars in cats.items():
                for var in vars:
                    key = f"{src}_{cat}_{var}"
                    # maxlen caps in-memory size to guard against bursts
                    self.history[key] = deque(maxlen=max_samples)

        # Lock for thread-safe updates & reads
        self._lock = Lock()

    def update(self, source: str, category: str, variable: str, value: Any) -> None:
        """
        Record a new sample:
          - Update the live snapshot.
          - Append (timestamp, value) to the rolling history.
          - Automatically drops anything older than window_seconds.
        """
        ts = time.monotonic()
        key = f"{source}_{category}_{variable}"

        with self._lock:
            # 1) Snapshot
            self.data[source][category][variable] = value

            # 2) History
            dq = self.history[key]
            dq.append((ts, value))

            # 3) Prune anything outside the window
            cutoff = ts - self.window_seconds
            while dq and dq[0][0] < cutoff:
                dq.popleft()

    def get_latest(self, source: str, category: str, variable: str) -> Optional[Any]:
        """Return the most recent value (or None)."""
        return self.data[source][category][variable]

    def get_history(self, source: str, category: str, variable: str
                   ) -> List[Tuple[float, Any]]:
        """Return a copy of the rolling (timestamp, value) list."""
        key = f"{source}_{category}_{variable}"
        with self._lock:
            return list(self.history[key])

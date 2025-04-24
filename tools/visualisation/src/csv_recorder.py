import csv
import os
import time
import datetime
from typing import Optional

class CSVRecorder:
    """
    CSVRecorder writes out the latest DataStore snapshot at a fixed interval
    (e.g. driven by a timer).  It flattens nested DataStore keys into columns,
    and uses DictWriter to keep header/data aligned.
    """

    def __init__(self,
                 data_store,
                 out_dir: str = ".",
                 filename_prefix: str = "flight_data"):
        """
        :param data_store:        Instance of DataStore
        :param out_dir:           Directory to write CSV files into
        :param filename_prefix:   Prefix for the CSV filename (timestamp is appended)
        """
        self.data_store    = data_store
        self.out_dir       = out_dir
        self.filename_prefix = filename_prefix

        self.is_recording  = False
        self.file          = None
        self.writer        = None
        self.fieldnames    = self._make_fieldnames()

    def _make_fieldnames(self):
        """
        Inspect the data_store.data dict and build a flat list of column names:
          ['timestamp',
           'imu_quaternion_w', 'imu_quaternion_x', …,
           'controller_rate_roll', …]
        """
        cols = ["timestamp"]
        for source, cats in self.data_store.data.items():
            for category, vars in cats.items():
                for var in vars:
                    cols.append(f"{source}_{category}_{var}")
        return cols

    def start_recording(self):
        """
        Open a new CSV file and write the header row.
        Filename = {prefix}_YYYYMMDD_HHMMSS.csv in out_dir.
        """
        if self.is_recording:
            return

        # Ensure output directory exists
        os.makedirs(self.out_dir, exist_ok=True)

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = f"{self.filename_prefix}_{timestamp}.csv"
        full_path = os.path.join(self.out_dir, fname)

        try:
            self.file = open(full_path, "w", newline="")
            self.writer = csv.DictWriter(self.file, fieldnames=self.fieldnames)
            self.writer.writeheader()
            self.is_recording = True
            print(f"[CSVRecorder] Recording started: {full_path}")
        except Exception as e:
            print(f"[CSVRecorder] Failed to start recording: {e}")
            self.is_recording = False

    def stop_recording(self):
        """
        Close the CSV file cleanly.
        """
        if not self.is_recording:
            return
        try:
            self.file.close()
            print("[CSVRecorder] Recording stopped.")
        except Exception as e:
            print(f"[CSVRecorder] Error closing file: {e}")
        finally:
            self.is_recording = False
            self.file   = None
            self.writer = None

    def log_data(self):
        """
        Snapshot the latest values from data_store and write one row.
        Call this at whatever frequency you desire.
        """
        if not self.is_recording or self.writer is None:
            return

        # Build a flat dict of values
        row = {}
        row["timestamp"] = time.time()
        for source, cats in self.data_store.data.items():
            for category, vars in cats.items():
                for var, val in vars.items():
                    key = f"{source}_{category}_{var}"
                    row[key] = val

        try:
            self.writer.writerow(row)
            # not flushing every line to improve performance; flush on stop
        except Exception as e:
            print(f"[CSVRecorder] Error writing row: {e}")

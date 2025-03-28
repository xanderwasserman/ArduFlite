import csv
import time
import datetime

class CSVRecorder:
    """
    CSVRecorder handles CSV logging of flight data.
    """
    def __init__(self, data_store):
        self.data_store = data_store
        self.is_recording = False
        self.file = None
        self.writer = None

    def start_recording(self):
        """
        Start recording data to a CSV file. The filename is based on the current date and time.
        """
        if self.is_recording:
            return
        filename = datetime.datetime.now().strftime("flight_data_%Y%m%d_%H%M%S.csv")
        self.file = open(filename, 'w', newline='')
        self.writer = csv.writer(self.file)
        # Create header row: timestamp and all variable names
        header = ["timestamp"]
        for category, vars in self.data_store.data.items():
            for var in vars:
                header.append(f"{category}_{var}")
        self.writer.writerow(header)
        self.is_recording = True
        print(f"Started recording to {filename}")

    def stop_recording(self):
        """
        Stop CSV recording and close the file.
        """
        if not self.is_recording:
            return
        self.file.close()
        self.is_recording = False
        print("Stopped recording.")

    def log_data(self):
        """
        Log the current set of values (with a timestamp) to the CSV file.
        """
        if not self.is_recording:
            return
        row = [time.time()]
        for category, vars in self.data_store.data.items():
            for var in vars:
                row.append(self.data_store.data[category][var])
        self.writer.writerow(row)
        self.file.flush()

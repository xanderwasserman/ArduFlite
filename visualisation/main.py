import sys
from PyQt5 import QtWidgets
from src.data_store import DataStore
from src.mqtt_client import MqttClient
from src.ui import MainWindow

def main():
    # Create the shared DataStore (30-second rolling window)
    data_store = DataStore(window_seconds=30)

    # Initialize the MQTT client with the broker address and port.
    mqtt_client = MqttClient("192.168.100.14", 1883, data_store)
    mqtt_client.connect()

    # Start the Qt application.
    app = QtWidgets.QApplication(sys.argv)
    main_window = MainWindow(data_store, mqtt_client)
    main_window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()

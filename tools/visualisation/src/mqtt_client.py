import paho.mqtt.client as mqtt

class MqttClient:
    """
    MqttClient handles the connection to the MQTT broker and updates the DataStore with incoming data.
    """
    def __init__(self, broker_address: str, broker_port: int, data_store):
        self.broker_address = broker_address
        self.broker_port = broker_port
        self.data_store = data_store
        self.client = mqtt.Client()
        # Set MQTT callbacks
        self.client.on_connect    = self.on_connect
        self.client.on_message    = self.on_message
        self.client.on_disconnect = self.on_disconnect

    def connect(self) -> None:
        """
        Connect to the MQTT broker and start the network loop in a background thread.
        """
        try:
            self.client.connect(self.broker_address, self.broker_port, keepalive=60)
        except Exception as e:
            print(f"[MQTT] Connection failed: {e}")
            return
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc) -> None:
        """
        Callback for when the client connects to the broker.
        Subscribe to all arduflite topics in one shot.
        """
        if rc == 0:
            print("[MQTT] Connected successfully")
            # You can subscribe to # and filter in on_message, or
            # explicitly list out topics if you need fine QoS/per-topic callback.
            client.subscribe("arduflite/#", qos=1)
        else:
            print(f"[MQTT] Bad connection, returned code {rc}")

    def on_disconnect(self, client, userdata, rc) -> None:
        """
        Callback for when the client disconnects.
        """
        print(f"[MQTT] Disconnected with code {rc}. Attempting reconnect…")
        try:
            client.reconnect()
        except Exception as e:
            print(f"[MQTT] Reconnect failed: {e}")

    def on_message(self, client, userdata, msg) -> None:
        """
        Callback for incoming MQTT messages. Decodes the payload and updates the DataStore.
        """
        topic = msg.topic
        payload = msg.payload.decode(errors="ignore").strip()
        try:
            value = float(payload)
        except ValueError:
            print(f"[MQTT] Non-float payload on {topic}: '{payload}'")
            return

        parts = topic.split('/')
        # Expect: ["arduflite", source, category, variable]
        if len(parts) != 4 or parts[0] != "arduflite":
            print(f"[MQTT] Unexpected topic format: {topic}")
            return

        source, category, variable = parts[1], parts[2], parts[3]

        # Validate keys exist before updating
        ds = self.data_store
        if (source   in ds.data 
         and category in ds.data[source] 
         and variable in ds.data[source][category]):
            ds.update(source, category, variable, value)
        else:
            print(f"[DataStore] Unknown channel: {source}/{category}/{variable}")

    def publish_command(self, topic: str, payload: Any) -> None:
        """
        Publish a command to the flight-controller.
        """
        result = self.client.publish(topic, payload)
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            print(f"[MQTT] Failed to publish to {topic}: rc={result.rc}")

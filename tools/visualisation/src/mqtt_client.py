import paho.mqtt.client as mqtt

class MqttClient:
    """
    MqttClient handles the connection to the MQTT broker and updates the DataStore with incoming data.
    """
    def __init__(self, broker_address, broker_port, data_store):
        self.broker_address = broker_address
        self.broker_port = broker_port
        self.data_store = data_store
        self.client = mqtt.Client()
        # Set MQTT callbacks
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def connect(self):
        """
        Connect to the MQTT broker and start the network loop.
        """
        self.client.connect(self.broker_address, self.broker_port, 60)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        """
        Callback for when the client connects to the broker.
        Subscribe to all required topics.
        """
        topics = [
            ("arduflite/quaternion/w", 0),
            ("arduflite/quaternion/x", 0),
            ("arduflite/quaternion/y", 0),
            ("arduflite/quaternion/z", 0),
            ("arduflite/accel/x", 0),
            ("arduflite/accel/y", 0),
            ("arduflite/accel/z", 0),
            ("arduflite/gyro/x", 0),
            ("arduflite/gyro/y", 0),
            ("arduflite/gyro/z", 0),
            ("arduflite/orientation/pitch", 0),
            ("arduflite/orientation/roll", 0),
            ("arduflite/orientation/yaw", 0),
            ("arduflite/command_rate/rollRateCmd", 0),
            ("arduflite/command_rate/pitchRateCmd", 0),
            ("arduflite/command_rate/yawRateCmd", 0),
            ("arduflite/command_servo/rollCmd", 0),
            ("arduflite/command_servo/pitchCmd", 0),
            ("arduflite/command_servo/yawCmd", 0)
        ]
        for topic, qos in topics:
            client.subscribe(topic, qos)
        print("Connected to MQTT broker and subscribed to topics.")

    def on_message(self, client, userdata, msg):
        """
        Callback for incoming MQTT messages. Decodes the payload and updates the DataStore.
        """
        topic = msg.topic
        try:
            value = float(msg.payload.decode())
        except Exception as e:
            print(f"Error decoding message on topic {topic}: {e}")
            return

        # Split topic "arduflite/category/variable" into its parts.
        parts = topic.split('/')
        if len(parts) < 3:
            return

        # parts[0] is "arduflite", parts[1] is the category, parts[2] is the variable.
        category = parts[1]
        variable = parts[2]
        # Update the corresponding category in the data store.
        if category in self.data_store.data:
            self.data_store.update(category, variable, value)

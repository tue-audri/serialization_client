import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import yaml
import json
import importlib
from serialization_client.deserializers import hybrid_deserialize

class DynamicDeserialize(Node):
    def __init__(self, config_file = '/home/saurabh/Workspaces/serialization_client/serialization_client/config_mqtt_to_ros_bridge.yaml'):
        super().__init__('dynamic_json_deserializer_node')

        # Load config
        with open(config_file, 'r') as file:
            self.config = yaml.safe_load(file)

        self.topic_publishers = {}
        self.topic_map = {}

        for entry in self.config['mqtt_to_ros']:
            mqtt_topic = entry['mqtt_topic']
            ros_topic = entry['ros_topic']
            msg_type_str = entry['msg_type']
            module_name, class_name = msg_type_str.rsplit('.', 1)
            module = importlib.import_module(module_name)
            msg_type = getattr(module, class_name)

            self.topic_publishers[mqtt_topic] = self.create_publisher(msg_type, ros_topic, 10)
            self.topic_map[mqtt_topic] = msg_type

        # MQTT client setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect("localhost", 1884, 60) # Change as needed
        self.mqtt_client.loop_start()


    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected to MQTT broker")
        for topic in self.topic_publishers.keys():
            client.subscribe(topic)
            self.get_logger().info(f"Subscribed to MQTT topic: {topic}")

    def on_message(self, client, userdata, msg):
        try:
            json_data = json.loads(msg.payload.decode())
            msg_type = self.topic_map[msg.topic]
            ros_msg = hybrid_deserialize(json_data, msg_type)
            self.topic_publishers[msg.topic].publish(ros_msg)
            self.get_logger().info(f"Published to ROS topic from MQTT: {msg.topic}")
        except Exception as e:
            self.get_logger().error(f"Failed to handle MQTT message: {e}")

def main(args=None):
    rclpy.init(args=args)
    node=DynamicDeserialize()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.mqtt_client.loop_stop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
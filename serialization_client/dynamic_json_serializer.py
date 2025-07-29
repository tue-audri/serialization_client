import sys
import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
import yaml
import importlib
from serialization_client import serializers
import paho.mqtt.client as mqtt

class DynamicSerializerNode(Node):
    def __init__(self,config_path):
        super().__init__('dynamic_json_serializer_node')

        
        # MQTT Setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect("localhost", 1883) # Replace with correct Broker IP and Port

        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)

        for topic in config['topics']:
            topic_name = topic['name']
            msg_type_str = topic['type']
            serializer_fn_name = topic['serializer']
            mqtt_topic = topic.get('mqtt_topic', topic_name.replace("/", "_"))

            # Dynamically import message class
            msg_module, msg_class = msg_type_str.rsplit('.', 1)
            msg_module_imported = importlib.import_module(msg_module)
            msg_type = getattr(msg_module_imported, msg_class)

            # Get serializer function
            serializer_fn = getattr(serializers, serializer_fn_name)

            # Define callback dynamically
            def make_callback(topic_name, serializer_fn, mqtt_topic):
                def callback(msg):
                    json_str = serializer_fn(msg)
                    self.mqtt_client.publish(mqtt_topic,json_str)
                    self.get_logger().info(f"[MQTT:{mqtt_topic}] Published JSON")
                return callback

            callback = make_callback(topic_name, serializer_fn,mqtt_topic)

            # Create subscription
            self.create_subscription(msg_type, topic_name, callback, 10)
            self.get_logger().info(f"Subscribed to {topic_name} with type {msg_type_str}")

def main():
    rclpy.init()

    package_share = get_package_share_directory('serialization_client')
    default_config_file = os.path.join(package_share, 'config', 'config_ros_to_mqtt_bridge.yaml')
    config_file = sys.argv[1] if len(sys.argv) > 1 else default_config_file

    node = DynamicSerializerNode(config_file)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

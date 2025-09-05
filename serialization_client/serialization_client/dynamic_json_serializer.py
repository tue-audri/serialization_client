import sys
import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import yaml
import json
import importlib
from serialization_client import serializers
from tf2_msgs.msg import TFMessage
import paho.mqtt.client as mqtt

class DynamicSerializerNode(Node):
    def __init__(self,config_path):
        super().__init__('dynamic_json_serializer_node')
        # Load config file
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)

        self.transforms = [] # TF cache
        self.current_kinematic_state = None # Store current kinematic state
        self.current_kinematic_state = {
                "pose": {
                    'x': 280,
                    'y': 48.21,
                    'z': 10.6
                },
                "orientation":{
                    'x': 0,
                    'y':0,
                    'z': 0,
                    'w': 0
                }
        }

        qos = QoSProfile(
            depth=10,  # keep last sample
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # subscribe to tf_static
        self.create_subscription(TFMessage, '/tf_static', self.tf_callback, qos)
        self.get_logger().info(f"Subscribed to tf_static with type TFMessage")
        

        mqtt_config = config.get("mqtt", None)

        if mqtt_config:
            hostname = mqtt_config.get("hostname", None)
            port = mqtt_config.get("port", None)
            # MQTT Setup
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.connect(hostname, port)
        else:
            self.get_logger().info("MQTT config details not found")


        agent_id = config.get("agent",{}).get("details",{}).get('agent_id', 'default_agent')
        

        for key, topic in config['agent']['static_topics'].items():
            if isinstance(topic,str):
                config["agent"]["static_topics"][key] = topic.format(agent_id = agent_id)
            
        for topic in config['agent']['ros_topics']:
            topic['mqtt_topic'] = topic['mqtt_topic'].replace('{agent_id}', agent_id)     

        for topic in config['agent']['ros_topics']:
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
            def make_callback(topic_name, serializer_fn, mqtt_topic, serializer_fn_name):
                def callback(msg):
                    json_str = serializer_fn(msg)
                    self.mqtt_client.publish(mqtt_topic,json_str)
                    # Store kinematic state data if this is the kinematic report
                    if serializer_fn_name == 'kinematic_report_to_json':
                        self.store_kinematic_data(msg)
                    # self.get_logger().info(f"[MQTT:{mqtt_topic}] Published JSON")
                return callback

            callback = make_callback(topic_name, serializer_fn,mqtt_topic, serializer_fn_name)

            # Create subscription
            self.create_subscription(msg_type, topic_name, callback, 10)
            self.get_logger().info(f"Subscribed to {topic_name} with type {msg_type_str}")

        # Generate static announcement
        self.announcement_config = {"mqtt_topic": config['agent']['static_topics']['mqtt_topic'],
                               "payload": {
                                   "agent_id": config['agent']['details']['agent_id'],
                                   "make": config['agent']['details']['make'],
                                   "model": config['agent']['details']['model'],
                                   "pose": self.current_kinematic_state,
                                   "urdf": self.transforms,
                                   "publications": []
                               }
                            }
        # Add publications to static announcement
        for topic in config["agent"]["ros_topics"]:
            mqtt_topic = topic.get("mqtt_topic")
            if mqtt_topic:
                self.announcement_config["payload"]["publications"].append(mqtt_topic)
        self.create_timer(5.0,self.publish_announcement)

    def publish_announcement(self):
            if self.announcement_config:
                mqtt_topic = self.announcement_config['mqtt_topic']
                payload = json.dumps(self.announcement_config['payload'])
                self.mqtt_client.publish(mqtt_topic,payload=payload, retain=True)
                print(f"Published announcement to {mqtt_topic}")
                print(f"Published TF {self.transforms}")

    def tf_callback(self, msg: TFMessage):
        # self.transforms.clear()
        print("Hello from tf_callback")
        for transform in msg.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id

            if(parent == "base_link" and child == "sensor_kit_base_link") or (parent == "sensor_kit_base_link"):
                tf_dict = {
                    "parent": parent,
                    "child": child,
                    "translation": {
                        "x": transform.transform.translation.x,
                        "y": transform.transform.translation.y,
                        "z": transform.transform.translation.z,
                    },
                    "rotation": {
                        "x": transform.transform.rotation.x,
                        "y": transform.transform.rotation.y,
                        "z": transform.transform.rotation.z,
                        "w": transform.transform.rotation.w,
                    }
                }
                self.transforms.append(tf_dict)
        self.get_logger().info(f"Cached {len(self.transforms)} static TFs")

    def store_kinematic_data(self, msg):
        try:
            self.current_kinematic_state = {
                "pose": {
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'z': msg.pose.pose.position.z
                },
                "orientation":{
                    'x': msg.pose.pose.orientation.x,
                    'y': msg.pose.pose.orientation.y,
                    'z': msg.pose.pose.orientation.z,
                    'w': msg.pose.pose.orientation.w
                }
            }           
            self.get_logger().debug(f"Updated kinematic_state : {self.current_kinematic_state}")
            self.announcement_config["payload"]["pose"] = self.current_kinematic_state
            
        except AttributeError as e:
            self.get_logger().warn(f"Could not extract pose/orientation from kinematic message: {e}")


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

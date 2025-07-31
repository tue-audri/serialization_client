# serialization_client

A modular ROS 2 package to serialize and deserialize messages to and from JSON, with support for MQTT communication and dynamic topic configuration via YAML. This package consists of 2 nodes - One to dynamically serialize ROS2 messages to JSON format and publish to MQTT, Second to listen to recieve JSON messages from MQTT and deserialize and publish to ROS2 topic.

---

## Pre-Requisites

## Installation
To install the *serialization_client*, clone this repository to *src\* of your ROS workspace. Install by running
``` colcon build --packages-select serialization_client``` 
from the root of the workspace.
## Usage
Source workspace:
```source install/setup.bash```

### Serialize to JSON
Include the serialization logic in *serializers.py* if not already included.
Include the desired topics in *config/config_ros_to_mqtt_bridge.yaml*.
Build and source the workspace 
Run the serialization node with:
```ros2 run serialization_client dynamic_json_serializer```

### Deserialize from JSON
Include details of topics to deserialize in *config/config_mqtt_to_ros_bridge.yaml*
Build and source the workspace
Run Deserailization node with:
```ros2 run serialization_client dynamic_deserializer```
Incase the target ROS2 topic contains multiple nested ROS messages, a custom deserialization logic may need to be included in *deserailizers.py*

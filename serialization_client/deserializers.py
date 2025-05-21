from autoware_perception_msgs.msg import TrafficLightGroupArray, TrafficLightGroup, TrafficLightElement
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry

# Generic Deserializer
def generic_deserialize(json_data, msg_type, type_hints: dict = None):
    msg = msg_type()
    for slot in msg.__slots__:
        field_name = slot.lstrip('_')  # Fix: remove leading underscore
        if field_name not in json_data:
            continue
        value = json_data[field_name]
        field_value = getattr(msg, field_name)

        if isinstance(value, dict):
            nested_type = type(field_value)
            setattr(msg, field_name, generic_deserialize(value, nested_type, type_hints))
        elif isinstance(value, list):
            # Determine list element type using hints
            elem_type = None
            if type_hints and field_name in type_hints:
                elem_type = type_hints[field_name]
            if elem_type:
                setattr(msg, field_name, [generic_deserialize(v, elem_type, type_hints) for v in value])
            else:
                setattr(msg, field_name, value)
        else:
            setattr(msg, field_name, value)

    return msg

# Custom Deserializer from TrafficLightGroupArray
def custom_deserialize_traffic_light_group_array(json_data):
    type_hints = {
        'traffic_light_groups': TrafficLightGroup,
        'elements': TrafficLightElement,
    }
    return generic_deserialize(json_data, TrafficLightGroupArray, type_hints)

# Deserializer Entrypoint
def hybrid_deserialize(json_data, msg_type):
    # Check if message type has custom logic
    if msg_type.__name__ == 'TrafficLightGroupArray':
        return custom_deserialize_traffic_light_group_array(json_data)

    return generic_deserialize(json_data, msg_type)



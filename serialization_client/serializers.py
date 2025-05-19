from autoware_vehicle_msgs.msg import GearReport, VelocityReport
from nav_msgs.msg import Odometry
from autoware_perception_msgs.msg import TrafficLightGroupArray, TrafficLightGroup,TrafficLightElement
import json

# Convert Gear Report Object to json
def gear_report_to_json(msg: GearReport) -> str:
    return json.dumps({
        "stamp": {
            "sec": msg.stamp.sec,
            "nanosec": msg.stamp.nanosec
        },
        "report": msg.report
    }, indent=2)

# Convert Velocity Report Object to json
def velocity_report_to_json(msg: VelocityReport) -> str:
    return json.dumps({
        "header": {
            "stamp": {
                "sec": msg.header.stamp.sec,
                "nanosec": msg.header.stamp.nanosec
            },
            "frame_id": msg.header.frame_id
        },
        "longitudinal_velocity": msg.longitudinal_velocity,
        "lateral_velocity": msg._lateral_velocity,
        "heading_rate": msg.heading_rate
    }, indent=2)

# Convert Kinematic State Object to json
def kinematic_report_to_json(msg: Odometry):
    return json.dumps({
        "header": {
            "stamp": {
                "sec": msg.header.stamp.sec,
                "nanosec": msg.header.stamp.nanosec
            },
            "frame_id": msg.header.frame_id
        },
        "child_frame_id": msg.child_frame_id,
        "pose":{
            "position":{
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "z": msg.pose.pose.position.z
            },
            "orientation": {
                "x": msg.pose.pose.orientation.x,
                "y": msg.pose.pose.orientation.y,
                "z": msg.pose.pose.orientation.z,
                "w": msg.pose.pose.orientation.w
            },
            "covariance": list(msg.pose.covariance)
        },
        "twist":{
            "linear":{
                "x": msg.twist.twist.linear.x,
                "y": msg.twist.twist.linear.y,
                "z": msg.twist.twist.linear.z
            },
             "angular": {
                "x": msg.twist.twist.angular.x,
                "y": msg.twist.twist.angular.y,
                "z": msg.twist.twist.angular.z
            },
            "covariance": list(msg.twist.covariance)
        },

    },indent=3)

# Convert Traffic Light Group Array Object to json
def traffic_light_group_array_to_json(msg: TrafficLightGroupArray)->str:
    return json.dumps({
        "stamp": {
            "sec": msg.stamp.sec,
            "nanosec": msg.stamp.nanosec
        },
        "traffic_light_groups": [
            {
                "traffic_light_group_id": group.traffic_light_group_id,
                "elements": [
                    {
                        "color": element.color,
                        "shape": element.shape,
                        "status": element.status,
                        "confidence": element.confidence
                    }
                    for element in group.elements
                ]
            }
            for group in msg.traffic_light_groups
        ]
    }, indent=3)     
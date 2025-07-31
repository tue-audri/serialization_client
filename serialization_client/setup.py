from setuptools import find_packages, setup

package_name = 'serialization_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/config_mqtt_to_ros_bridge.yaml']),
        ('share/' + package_name + '/config', ['config/config_ros_to_mqtt_bridge.yaml']),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saurabh',
    maintainer_email='152851798+subramanians1@users.noreply.github.com',
    description='A dynamic JSON Serializer Package that serializes and deserializes ROS2 messages for use with MQTT',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamic_json_serializer = serialization_client.dynamic_json_serializer:main',
            'dynamic_json_deserializer = serialization_client.dynamic_deserializer:main',
        ],
    },
)

"""ROS2 port of the original ROS1 speed_query.launch.

Standalone speed-query node for reading motor speeds over the CAN/serial
link.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vehicle_can_bridge',
            executable='speed_query',
            name='speed_query_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 2000000,
                'enable_debug': True,
                'query_frequency': 10.0,
                'motor_pole_pairs': 15,
                'wheel_radius': 0.1275,
            }],
        ),
    ])

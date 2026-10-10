"""ROS2 port of the original ROS1 complete_control.launch.

Brings up the CAN bridge (with speed-query and servo/steering conversion
parameters) plus cmd_vel -> CAN conversion. The commented-out can_monitor
node in the original file is intentionally not ported (out of scope, see
Task 8 brief).
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vehicle_can_bridge',
            executable='can_bridge',
            name='can_bridge',
            output='screen',
            # NOTE: the original launch also set
            # <rosparam param="drive_ids">[1, 2, 3, 4]</rosparam> on this
            # node, but can_bridge_node never declares/reads a "drive_ids"
            # parameter. ROS2 launch would silently ignore an undeclared
            # parameter anyway, so it is intentionally omitted here rather
            # than carried over as a dead/misleading parameter.
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 2000000,
                'enable_debug': True,
                'query_frequency': 10.0,
                'auto_query_enabled': True,
                'servo_center_angle': 90.0,
                'servo_left_limit': 40.0,
                'servo_right_limit': 190.0,
                'steering_ratio': 15.0,
                'max_steering_angle': 30.0,
            }],
        ),
        Node(
            package='vehicle_can_bridge',
            executable='cmd_vel_to_can',
            name='cmd_vel_to_can_node',
            output='screen',
            parameters=[{
                'wheel_base': 0.3,
                'wheel_radius': 0.1275,
                'max_linear_speed': 2.0,
                'max_angular_speed': 1.0,
                'linear_scale': 1.0,
                'angular_scale': 1.0,
            }],
        ),
    ])

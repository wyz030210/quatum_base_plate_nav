"""ROS2 port of the original ROS1 auto_control.launch.

Brings up the cmd_vel -> CAN conversion node and the CAN bridge node.

The keyboard control node is intentionally NOT included here (and never
should be): `ros2 launch` does not attach a real TTY to the processes it
manages, and keyboard_control.py calls termios.tcgetattr(sys.stdin) at
startup, which raises `termios.error: (25, 'Inappropriate ioctl for
device')` and kills the process immediately when run this way (confirmed
on real hardware, 2026-07-13). Run keyboard_control.py directly via
`ros2 run` in its own terminal instead -- see GETTING_STARTED.md
("路径 A：键盘遥控") for the exact command.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    wheel_base = LaunchConfiguration('wheel_base')
    wheel_radius = LaunchConfiguration('wheel_radius')
    max_forward_speed = LaunchConfiguration('max_forward_speed')
    max_reverse_speed = LaunchConfiguration('max_reverse_speed')
    max_yaw_rate = LaunchConfiguration('max_yaw_rate')
    max_steering_angle_deg = LaunchConfiguration('max_steering_angle_deg')
    min_steering_speed = LaunchConfiguration('min_steering_speed')
    serial_port = LaunchConfiguration('serial_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'command_safety_params', default_value=PathJoinSubstitution([
                FindPackageShare('vehicle_can_bridge'), 'config', 'command_safety.yaml'])),
        DeclareLaunchArgument('task_control_required', default_value='false'),
        DeclareLaunchArgument('wheel_base', default_value='0.3'),
        DeclareLaunchArgument('wheel_radius', default_value='0.1275'),
        DeclareLaunchArgument('max_forward_speed', default_value='0.25'),
        DeclareLaunchArgument('max_reverse_speed', default_value='0.10'),
        DeclareLaunchArgument('max_yaw_rate', default_value='0.50'),
        # 实际最大前轮转角。实测转弯半径 0.95 m（车体中心画圆）→
        # tan(delta) = 轴距/0.95 = 0.30/0.95 → delta ≈ 17.5°。
        # 该参数既是自行车模型的钳位、又是 servoAngleFromSteering 的满量程
        # （见 ackermann_steering.hpp:42-51 的线性映射），配 30° 会导致
        # 命令 30° 只得到 17.5° 实际转角（增益 0.583），使控制器系统性转向不足。
        DeclareLaunchArgument('max_steering_angle_deg', default_value='17.5'),
        DeclareLaunchArgument('min_steering_speed', default_value='0.02'),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'),
        Node(
            package='vehicle_can_bridge',
            executable='cmd_vel_to_can',
            name='cmd_vel_to_can_node',
            output='screen',
            parameters=[LaunchConfiguration('command_safety_params'), {
                'task_control_required': ParameterValue(
                    LaunchConfiguration('task_control_required'), value_type=bool),
                'wheel_base': ParameterValue(wheel_base, value_type=float),
                'wheel_radius': ParameterValue(wheel_radius, value_type=float),
                'max_forward_speed': ParameterValue(max_forward_speed, value_type=float),
                'max_reverse_speed': ParameterValue(max_reverse_speed, value_type=float),
                'max_angular_speed': ParameterValue(max_yaw_rate, value_type=float),
                'max_steering_angle_deg': ParameterValue(
                    max_steering_angle_deg, value_type=float),
                'min_steering_speed': ParameterValue(
                    min_steering_speed, value_type=float),
                'linear_scale': 1.0,
                'angular_scale': 1.0,
            }],
        ),
        Node(
            package='vehicle_can_bridge',
            executable='can_bridge',
            name='can_bridge',
            output='screen',
            parameters=[{
                'port': serial_port,
                'baudrate': 2000000,
                'enable_debug': True,
            }],
        ),
    ])

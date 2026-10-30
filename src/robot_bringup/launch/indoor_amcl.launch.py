from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include_launch(package_name, relative_path, launch_arguments=None):
    launch_path = PathJoinSubstitution([FindPackageShare(package_name), *relative_path])
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=(launch_arguments or {}).items(),
    )


def delayed_stage(delay, label, action):
    actions = [
        LogInfo(msg=f"==================== INDOOR STAGE: {label} ===================="),
        action,
    ]
    return actions if delay <= 0.0 else [TimerAction(period=delay, actions=actions)]


def validate_arguments(context):
    map_path = Path(LaunchConfiguration("map").perform(context))
    if not map_path.is_absolute():
        raise RuntimeError("map must be an absolute path to a map YAML file")
    if not map_path.is_file():
        raise RuntimeError(f"map YAML does not exist: {map_path}")

    for name in ("use_rviz", "use_sim_time", "use_task_control"):
        value = LaunchConfiguration(name).perform(context).strip().lower()
        if value not in {"true", "1", "false", "0"}:
            raise RuntimeError(
                f"{name} must be one of true, false, 1, or 0; got {value!r}"
            )

    if LaunchConfiguration("use_task_control").perform(context).lower() in ("true", "1"):
        digest = LaunchConfiguration("task_map_id").perform(context)
        profile = LaunchConfiguration("task_localization_profile").perform(context)
        if len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest):
            raise RuntimeError("task_map_id must be the explicitly provisioned lowercase SHA256")
        if not profile or profile != profile.strip():
            raise RuntimeError("task_localization_profile must be explicitly provisioned")

    positive_arguments = (
        "max_forward_speed",
        "max_reverse_speed",
        "max_yaw_rate",
        "max_linear_accel",
        "max_yaw_accel",
        "max_steering_angle_deg",
        "min_steering_speed",
    )
    for name in positive_arguments:
        value = float(LaunchConfiguration(name).perform(context))
        if value <= 0.0:
            raise RuntimeError(f"{name} must be greater than zero")

    scan_min_height = float(LaunchConfiguration("scan_min_height").perform(context))
    scan_max_height = float(LaunchConfiguration("scan_max_height").perform(context))
    if scan_min_height < 0.0:
        raise RuntimeError("scan_min_height must be zero or greater")
    if scan_max_height <= scan_min_height:
        raise RuntimeError("scan_max_height must be greater than scan_min_height")
    return []


def generate_launch_description():
    map_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    use_coverage = LaunchConfiguration("use_coverage")
    use_composition = LaunchConfiguration("use_composition")
    can_serial_port = LaunchConfiguration("can_serial_port")
    max_forward_speed = LaunchConfiguration("max_forward_speed")
    max_reverse_speed = LaunchConfiguration("max_reverse_speed")
    max_yaw_rate = LaunchConfiguration("max_yaw_rate")
    max_linear_accel = LaunchConfiguration("max_linear_accel")
    max_yaw_accel = LaunchConfiguration("max_yaw_accel")
    max_steering_angle_deg = LaunchConfiguration("max_steering_angle_deg")
    min_steering_speed = LaunchConfiguration("min_steering_speed")
    lidar_x = LaunchConfiguration("lidar_x")
    lidar_y = LaunchConfiguration("lidar_y")
    lidar_z = LaunchConfiguration("lidar_z")
    lidar_yaw = LaunchConfiguration("lidar_yaw")
    scan_min_height = LaunchConfiguration("scan_min_height")
    scan_max_height = LaunchConfiguration("scan_max_height")

    base_driver = include_launch(
        "vehicle_can_bridge",
        ["launch", "auto_control.launch.py"],
        {
            "serial_port": can_serial_port,
            "use_sim_time": use_sim_time,
            "task_control_required": LaunchConfiguration("use_task_control"),
            "wheel_base": "0.3",
            "wheel_radius": "0.1275",
            "max_forward_speed": max_forward_speed,
            "max_reverse_speed": max_reverse_speed,
            "max_yaw_rate": max_yaw_rate,
            "max_steering_angle_deg": max_steering_angle_deg,
            "min_steering_speed": min_steering_speed,
        },
    )

    livox_driver = include_launch(
        "livox_ros_driver2",
        ["launch_ROS2", "msg_MID360_launch.py"],
    )

    fast_lio = include_launch(
        "fast_lio",
        ["launch", "mapping.launch.py"],
        {
            "use_sim_time": use_sim_time,
            "config_file": "mid360_indoor.yaml",
            "rviz": "false",
        },
    )

    local_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_odom_node",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("wheeltec_nav2"),
                    "param",
                    "wheeltec_params",
                    "ekf_odom.yaml",
                ]
            ),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("/odometry/filtered", "/odometry/filtered_odom")],
    )

    navigation = include_launch(
        "wheeltec_nav2",
        ["launch", "nav2_indoor.launch.py"],
        {
            "map": map_file,
            "use_sim_time": use_sim_time,
            "use_rviz": use_rviz,
            "use_coverage": PythonExpression([
                "'", use_coverage, "'.lower() in ('true', '1') or '",
                LaunchConfiguration("use_task_control"), "'.lower() in ('true', '1')"]),
            "use_task_control": LaunchConfiguration("use_task_control"),
            "use_composition": use_composition,
            "max_forward_speed": max_forward_speed,
            "max_reverse_speed": max_reverse_speed,
            "max_yaw_rate": max_yaw_rate,
            "max_linear_accel": max_linear_accel,
            "max_yaw_accel": max_yaw_accel,
            "lidar_x": lidar_x,
            "lidar_y": lidar_y,
            "lidar_z": lidar_z,
            "lidar_yaw": lidar_yaw,
            "scan_min_height": scan_min_height,
            "scan_max_height": scan_max_height,
        },
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("wheeltec_nav2"), "map", "indoor_map.yaml"]
                ),
                description="Absolute path to the indoor map YAML",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_task_control", default_value="false"),
            DeclareLaunchArgument("task_map_id", default_value=""),
            DeclareLaunchArgument("task_localization_profile", default_value=""),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "use_composition",
                default_value="False",
                description=(
                    "Use a shared Nav2 component container. Separate processes avoid "
                    "Humble lifecycle shutdown deadlocks while waiting for initial TF."
                ),
            ),
            DeclareLaunchArgument(
                "use_coverage",
                default_value="false",
                description=(
                    "Launch the on-demand coverage trigger path: the coverage action "
                    "bridge (/execute_coverage_navpath) and the RViz region selector "
                    "(/clicked_point). Default false for ordinary indoor navigation; "
                    "set true to run full-coverage missions."
                ),
            ),
            DeclareLaunchArgument(
                "can_serial_port",
                default_value="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
            ),
            DeclareLaunchArgument("max_forward_speed", default_value="0.25"),
            DeclareLaunchArgument("max_reverse_speed", default_value="0.10"),
            DeclareLaunchArgument("max_yaw_rate", default_value="0.50"),
            DeclareLaunchArgument("max_linear_accel", default_value="0.40"),
            DeclareLaunchArgument("max_yaw_accel", default_value="0.50"),
            # 实测转弯半径 0.95 m → 实际最大前轮转角约 17.5°（见 auto_control.launch.py 的说明）
            DeclareLaunchArgument("max_steering_angle_deg", default_value="17.5"),
            DeclareLaunchArgument("min_steering_speed", default_value="0.02"),
            DeclareLaunchArgument("lidar_x", default_value="0.11"),
            DeclareLaunchArgument("lidar_y", default_value="0.0"),
            DeclareLaunchArgument("lidar_z", default_value="0.615"),
            DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
            DeclareLaunchArgument("scan_min_height", default_value="0.05"),
            DeclareLaunchArgument("scan_max_height", default_value="0.80"),
            OpaqueFunction(function=validate_arguments),
            Node(
                package="nav2_mission", executable="task_control_node", name="task_control_node",
                condition=IfCondition(LaunchConfiguration("use_task_control")),
                parameters=[{
                    "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                    "map_id": ParameterValue(LaunchConfiguration("task_map_id"), value_type=str),
                    "localization_profile": ParameterValue(
                        LaunchConfiguration("task_localization_profile"), value_type=str),
                    "require_localization_schema": True,
                }]),
            Node(
                package="nav2_mission", executable="localization_provider", name="localization_provider",
                condition=IfCondition(LaunchConfiguration("use_task_control")),
                parameters=[{
                    "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                    "source": "amcl",
                    "map_id": ParameterValue(LaunchConfiguration("task_map_id"), value_type=str),
                    "localization_profile": ParameterValue(
                        LaunchConfiguration("task_localization_profile"), value_type=str),
                }]),
            *delayed_stage(0.0, "vehicle_can_bridge", base_driver),
            *delayed_stage(2.0, "livox_mid360", livox_driver),
            *delayed_stage(4.0, "fast_lio", fast_lio),
            *delayed_stage(7.0, "ekf_odom_node", local_ekf),
            *delayed_stage(10.0, "amcl_nav2", navigation),
        ]
    )

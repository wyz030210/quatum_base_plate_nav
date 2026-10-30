"""Outdoor RTK localization with local and global EKFs; no AMCL."""

import math
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def include_launch(package_name, relative_path, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), *relative_path])
        ),
        launch_arguments=(launch_arguments or {}).items(),
    )


def delayed_stage(delay, label, action):
    actions = [LogInfo(msg=f"Outdoor stage: {label}"), action]
    return actions if delay <= 0.0 else [TimerAction(period=delay, actions=actions)]


def validate_arguments(context):
    map_path = Path(LaunchConfiguration("map").perform(context)).expanduser().resolve()
    if not map_path.is_file():
        raise RuntimeError(f"map YAML does not exist: {map_path}")

    for name in ("use_sim_time", "use_rviz", "use_coverage", "autostart",
                 "use_composition", "use_respawn", "use_task_control"):
        value = LaunchConfiguration(name).perform(context).strip().lower()
        if value not in {"true", "false", "1", "0"}:
            raise RuntimeError(f"{name} must be true or false; got {value!r}")

    if LaunchConfiguration("use_task_control").perform(context).lower() in ("true", "1"):
        digest = LaunchConfiguration("task_map_id").perform(context).strip()
        profile = LaunchConfiguration("task_localization_profile").perform(context).strip()
        if len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest):
            raise RuntimeError("task_map_id must be the explicitly provisioned lowercase SHA256")
        if not profile:
            raise RuntimeError("task_localization_profile must be explicitly provisioned")

    values = {}
    for name in ("wheel_base", "wheel_radius", "max_forward_speed", "max_reverse_speed",
                 "max_yaw_rate", "max_linear_accel", "max_yaw_accel",
                 "max_steering_angle_deg", "min_steering_speed"):
        try:
            value = float(LaunchConfiguration(name).perform(context))
        except ValueError as error:
            raise RuntimeError(f"{name} must be a finite positive number") from error
        if not math.isfinite(value) or value <= 0.0:
            raise RuntimeError(f"{name} must be a finite positive number")
        values[name] = value
    if values["max_steering_angle_deg"] >= 90.0:
        raise RuntimeError("max_steering_angle_deg must be less than 90 degrees")

    can_port = Path(LaunchConfiguration("can_serial_port").perform(context)).expanduser()
    rtk_port = Path(LaunchConfiguration("rtk_serial_port").perform(context)).expanduser()
    if can_port.resolve() == rtk_port.resolve():
        raise RuntimeError("CAN and RTK serial ports must be different; set rtk_serial_port")

    # Keep the outdoor planner and coverage curvature within the same vehicle limit.
    radius = values["wheel_base"] / math.tan(math.radians(values["max_steering_angle_deg"]))
    return [
        SetLaunchConfiguration("map", str(map_path)),
        SetLaunchConfiguration("minimum_turning_radius", str(radius)),
    ]


def generate_launch_description():
    config = LaunchConfiguration
    speed_names = ("max_forward_speed", "max_reverse_speed", "max_yaw_rate",
                   "max_linear_accel", "max_yaw_accel")
    base_driver = include_launch(
        "vehicle_can_bridge", ["launch", "auto_control.launch.py"],
        {"serial_port": config("can_serial_port"),
         "use_sim_time": config("use_sim_time"),
         "task_control_required": config("use_task_control"),
         **{name: config(name) for name in (
             "wheel_base", "wheel_radius", "max_forward_speed", "max_reverse_speed",
             "max_yaw_rate", "max_steering_angle_deg", "min_steering_speed")}},
    )
    livox_driver = include_launch(
        "livox_ros_driver2", ["launch_ROS2", "msg_MID360_launch.py"]
    )
    fast_lio = include_launch(
        "fast_lio", ["launch", "mapping.launch.py"],
        {"use_sim_time": config("use_sim_time"),
         "config_file": config("fast_lio_config_file"), "rviz": "false"},
    )
    rtk = include_launch(
        "location", ["launch", "location_launch.py"],
        {"use_sim_time": config("use_sim_time"),
         "serial_port": config("rtk_serial_port"),
         "baud_rate": config("rtk_baud_rate"),
         "datum_lat": config("datum_lat"), "datum_lon": config("datum_lon")},
    )
    local_ekf = Node(
        package="robot_localization", executable="ekf_node", name="ekf_odom_node",
        output="screen",
        parameters=[PathJoinSubstitution([
            FindPackageShare("wheeltec_nav2"), "param", "wheeltec_params", "ekf_odom.yaml"
        ]), {"use_sim_time": ParameterValue(config("use_sim_time"), value_type=bool)}],
        remappings=[("/odometry/filtered", "/odometry/filtered_odom")],
    )
    global_ekf = Node(
        package="robot_localization", executable="ekf_node", name="ekf_map_node",
        output="screen",
        parameters=[PathJoinSubstitution([
            FindPackageShare("wheeltec_nav2"), "param", "wheeltec_params", "ekf_map.yaml"
        ]), {"use_sim_time": ParameterValue(config("use_sim_time"), value_type=bool)}],
        remappings=[("/odometry/filtered", "/odometry/filtered_map")],
    )
    navigation = include_launch(
        "wheeltec_nav2", ["launch", "nav2.launch.py"],
        {"map": config("map"), "params": config("params_file"),
         "use_task_control": config("use_task_control"),
         "use_coverage": PythonExpression(["'", config("use_coverage"), "'.lower() in ('true', '1') or '", config("use_task_control"), "'.lower() in ('true', '1')"]),
         **{name: config(name) for name in (
             "coverage_params_file", "task_file", "use_sim_time",
             "use_rviz", "autostart", "use_composition", "use_respawn", "log_level",
             "minimum_turning_radius", *speed_names)}},
    )

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=PathJoinSubstitution([
            FindPackageShare("wheeltec_nav2"), "map", "outdoor_map.yaml"
        ]), description="Map YAML registered to the RTK datum and ENU axes"),
        DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution([
            FindPackageShare("wheeltec_nav2"), "param", "wheeltec_params", "param_mini_diff.yaml"
        ])),
        DeclareLaunchArgument("coverage_params_file", default_value=PathJoinSubstitution([
            FindPackageShare("wheeltec_nav2"), "param", "coverage_params.yaml"
        ])),
        DeclareLaunchArgument("task_file", default_value=PathJoinSubstitution([
            FindPackageShare("nav2_mission"), "config", "coverage_navpath_task.yaml"
        ])),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_task_control", default_value="false"),
        DeclareLaunchArgument("task_map_id", default_value=""),
        DeclareLaunchArgument("task_localization_profile", default_value=""),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_coverage", default_value="false",
                              description="Enable coverage nodes, without sending a mission"),
        DeclareLaunchArgument("autostart", default_value="true"),
        # Separate contexts avoid Humble lifecycle shutdown deadlocks during the initial TF wait.
        DeclareLaunchArgument("use_composition", default_value="False"),
        DeclareLaunchArgument("use_respawn", default_value="False"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("can_serial_port", default_value=
                              "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"),
        DeclareLaunchArgument("rtk_serial_port", default_value=(
            "/dev/serial/by-id/"
            "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
        )),
        DeclareLaunchArgument("rtk_baud_rate", default_value="115200"),
        DeclareLaunchArgument("datum_lat", default_value="29.4898045"),
        DeclareLaunchArgument("datum_lon", default_value="106.5676103"),
        DeclareLaunchArgument("fast_lio_config_file", default_value="mid360.yaml"),
        DeclareLaunchArgument("wheel_base", default_value="0.30"),
        DeclareLaunchArgument("wheel_radius", default_value="0.1275"),
        DeclareLaunchArgument("max_forward_speed", default_value="0.25"),
        DeclareLaunchArgument("max_reverse_speed", default_value="0.10"),
        DeclareLaunchArgument("max_yaw_rate", default_value="0.50"),
        DeclareLaunchArgument("max_linear_accel", default_value="0.40"),
        DeclareLaunchArgument("max_yaw_accel", default_value="0.50"),
        DeclareLaunchArgument("max_steering_angle_deg", default_value="17.5"),
        DeclareLaunchArgument("min_steering_speed", default_value="0.02"),
        OpaqueFunction(function=validate_arguments),
        Node(package="nav2_mission", executable="task_control_node", name="task_control_node",
             condition=IfCondition(LaunchConfiguration("use_task_control")),
             parameters=[{"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool), "map_id": LaunchConfiguration("task_map_id"),
                          "localization_profile": LaunchConfiguration("task_localization_profile"),
                          "require_localization_schema": True}]),
        Node(package="nav2_mission", executable="localization_provider", name="localization_provider",
             condition=IfCondition(LaunchConfiguration("use_task_control")),
             parameters=[{"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool), "source": "rtk_ekf",
                          "map_id": LaunchConfiguration("task_map_id"),
                          "localization_profile": LaunchConfiguration("task_localization_profile"), "rtk_datum_lat": ParameterValue(LaunchConfiguration("datum_lat"), value_type=float), "rtk_datum_lon": ParameterValue(LaunchConfiguration("datum_lon"), value_type=float)}]),
        *delayed_stage(0.0, "vehicle_can_bridge", base_driver),
        *delayed_stage(2.0, "livox_mid360", livox_driver),
        *delayed_stage(4.0, "fast_lio", fast_lio),
        *delayed_stage(5.0, "rtk", rtk),
        *delayed_stage(7.0, "ekf_odom_node", local_ekf),
        *delayed_stage(9.0, "ekf_map_node", global_ekf),
        *delayed_stage(11.0, "nav2", navigation),
    ])

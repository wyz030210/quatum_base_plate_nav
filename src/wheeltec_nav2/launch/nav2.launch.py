"""Outdoor Nav2 adapter: RTK EKF supplies map -> odom instead of AMCL."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_dir = get_package_share_directory("wheeltec_nav2")
    config = LaunchConfiguration
    use_sim_time = config("use_sim_time")
    use_coverage = config("use_coverage")

    # Only the outdoor adapter rewrites curvature and odometry; indoor stays unchanged.
    params_file = RewrittenYaml(
        source_file=config("params"),
        param_rewrites={
            "odom_topic": "/odometry/filtered_odom",
            "min_turning_r": config("minimum_turning_radius"),
            "minimum_turning_radius": config("minimum_turning_radius"),
        }, convert_types=True,
    )
    coverage_params_file = RewrittenYaml(
        source_file=config("coverage_params_file"),
        param_rewrites={"min_turning_radius": config("minimum_turning_radius")},
        convert_types=True,
    )
    navigation_arguments = {
        "map": config("map"), "params_file": params_file,
        "coverage_params_file": coverage_params_file,
        "use_amcl": "false", "slam": "False",
        "use_coverage": use_coverage, "use_coverage_server": use_coverage,
        **{name: config(name) for name in (
            "task_file", "use_sim_time", "autostart", "use_composition", "use_respawn",
            "log_level", "max_forward_speed", "max_reverse_speed", "max_yaw_rate",
            "max_linear_accel", "max_yaw_accel")},
    }

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=os.path.join(package_dir, "map", "map.yaml")),
        DeclareLaunchArgument("params", default_value=os.path.join(
            package_dir, "param", "wheeltec_params", "param_mini_diff.yaml"
        )),
        DeclareLaunchArgument("coverage_params_file", default_value=os.path.join(
            package_dir, "param", "coverage_params.yaml"
        )),
        DeclareLaunchArgument("task_file", default_value=PathJoinSubstitution([
            FindPackageShare("nav2_mission"), "config", "coverage_navpath_task.yaml"
        ])),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_coverage", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        # Keep the direct outdoor entry consistent with outdoor_ekf.launch.py.
        DeclareLaunchArgument("use_composition", default_value="False"),
        DeclareLaunchArgument("use_respawn", default_value="False"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("max_forward_speed", default_value="0.25"),
        DeclareLaunchArgument("max_reverse_speed", default_value="0.10"),
        DeclareLaunchArgument("max_yaw_rate", default_value="0.50"),
        DeclareLaunchArgument("max_linear_accel", default_value="0.40"),
        DeclareLaunchArgument("max_yaw_accel", default_value="0.50"),
        DeclareLaunchArgument("minimum_turning_radius", default_value="0.95"),
        Node(
            package="rviz2", executable="rviz2", name="rviz2",
            arguments=["-d", os.path.join(package_dir, "rviz", "map.rviz")],
            condition=IfCondition(config("use_rviz")), output="screen",
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="footprint_to_link",
            arguments=["0.0", "0.0", "0.0", "0", "0", "0", "base_link", "base_footprint"],
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher", name="livox_to_base",
            arguments=["0.11", "0.0", "0.615", "0", "0", "0", "base_footprint", "livox_frame"],
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher", name="camera_to_base",
            arguments=["0.1", "0.0", "0.28", "0", "0", "0", "base_footprint", "camera_link"],
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher", name="rtk_to_base",
            arguments=["0.319", "0.0", "0.11", "0", "0", "0", "base_footprint", "gps_link"],
        ),
        Node(
            package="wheeltec_nav2", executable="livox_to_pointcloud2.py",
            name="livox_custom_to_pointcloud2",
        ),
        Node(
            package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan", output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "target_frame": "livox_frame", "transform_tolerance": 0.01,
                "min_height": -0.5, "max_height": 0.5,
                "angle_min": -3.14159265359, "angle_max": 3.14159265359,
                "angle_increment": 0.00872664626, "scan_time": 0.1,
                "range_min": 0.1, "range_max": 30.0, "use_inf": True, "inf_epsilon": 1.0,
            }],
            remappings=[("cloud_in", "/livox/pointcloud2"), ("scan", "/scan")],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package_dir, "launch", "bringup_launch.py")),
            launch_arguments=navigation_arguments.items(),
        ),
    ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_dir = get_package_share_directory("wheeltec_nav2")
    launch_dir = os.path.join(package_dir, "launch")

    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    coverage_params_file = LaunchConfiguration("coverage_params_file")
    use_coverage = LaunchConfiguration("use_coverage")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    max_forward_speed = LaunchConfiguration("max_forward_speed")
    max_reverse_speed = LaunchConfiguration("max_reverse_speed")
    max_yaw_rate = LaunchConfiguration("max_yaw_rate")
    max_linear_accel = LaunchConfiguration("max_linear_accel")
    max_yaw_accel = LaunchConfiguration("max_yaw_accel")
    lidar_x = LaunchConfiguration("lidar_x")
    lidar_y = LaunchConfiguration("lidar_y")
    lidar_z = LaunchConfiguration("lidar_z")
    lidar_yaw = LaunchConfiguration("lidar_yaw")
    scan_min_height = LaunchConfiguration("scan_min_height")
    scan_max_height = LaunchConfiguration("scan_max_height")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                description="Absolute path to the Cartographer-generated map YAML",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    package_dir,
                    "param",
                    "wheeltec_params",
                    "param_mini_diff_indoor.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "coverage_params_file",
                default_value=os.path.join(
                    package_dir, "param", "coverage_params.yaml"
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
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_task_control", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            # Humble's shared-context lifecycle shutdown can deadlock in the initial TF wait.
            DeclareLaunchArgument("use_composition", default_value="False"),
            DeclareLaunchArgument("use_respawn", default_value="False"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("max_forward_speed", default_value="0.25"),
            DeclareLaunchArgument("max_reverse_speed", default_value="0.10"),
            DeclareLaunchArgument("max_yaw_rate", default_value="0.50"),
            DeclareLaunchArgument("max_linear_accel", default_value="0.40"),
            DeclareLaunchArgument("max_yaw_accel", default_value="0.50"),
            DeclareLaunchArgument("lidar_x", default_value="0.11"),
            DeclareLaunchArgument("lidar_y", default_value="0.0"),
            DeclareLaunchArgument("lidar_z", default_value="0.615"),
            DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
            DeclareLaunchArgument("scan_min_height", default_value="0.05"),
            DeclareLaunchArgument("scan_max_height", default_value="0.80"),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", PythonExpression([
                    "'", os.path.join(package_dir, "rviz", "map_task.rviz"), "' if '",
                    LaunchConfiguration("use_task_control"), "'.lower() in ('true', '1') else '",
                    os.path.join(package_dir, "rviz", "map.rviz"), "'"])],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_footprint",
                arguments=[
                    "--x", "0.0",
                    "--y", "0.0",
                    "--z", "0.0",
                    "--yaw", "0.0",
                    "--pitch", "0.0",
                    "--roll", "0.0",
                    "--frame-id", "base_link",
                    "--child-frame-id", "base_footprint",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_footprint_to_livox",
                arguments=[
                    "--x", lidar_x,
                    "--y", lidar_y,
                    "--z", lidar_z,
                    "--yaw", lidar_yaw,
                    "--pitch", "0.0",
                    "--roll", "0.0",
                    "--frame-id", "base_footprint",
                    "--child-frame-id", "livox_frame",
                ],
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                parameters=[
                    {
                        # Project in the ground-aligned vehicle frame so height
                        # limits reject floor returns instead of depending on
                        # the LiDAR mounting height.
                        "use_sim_time": use_sim_time,
                        "target_frame": "base_footprint",
                        "transform_tolerance": 0.01,
                        "min_height": ParameterValue(
                            scan_min_height, value_type=float
                        ),
                        "max_height": ParameterValue(
                            scan_max_height, value_type=float
                        ),
                        "angle_min": -3.14159265359,
                        "angle_max": 3.14159265359,
                        "angle_increment": 0.00872664626,
                        "scan_time": 0.1,
                        "range_min": 0.1,
                        "range_max": 30.0,
                        "use_inf": True,
                        "inf_epsilon": 1.0,
                    }
                ],
                remappings=[
                    ("cloud_in", "/livox/pointcloud2_undistorted"),
                    ("scan", "/scan"),
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "bringup_launch.py")
                ),
                launch_arguments={
                    "map": map_file,
                    "params_file": params_file,
                    "coverage_params_file": coverage_params_file,
                    "use_coverage": use_coverage,
                    "use_task_control": LaunchConfiguration("use_task_control"),
                    "use_sim_time": use_sim_time,
                    "slam": "False",
                    "autostart": autostart,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "log_level": log_level,
                    "max_forward_speed": max_forward_speed,
                    "max_reverse_speed": max_reverse_speed,
                    "max_yaw_rate": max_yaw_rate,
                    "max_linear_accel": max_linear_accel,
                    "max_yaw_accel": max_yaw_accel,
                }.items(),
            ),
        ]
    )

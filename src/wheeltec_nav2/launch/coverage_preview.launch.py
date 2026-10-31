"""Map, shared region editor and GUI preview without vehicle nodes."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = get_package_share_directory("wheeltec_nav2")
    params_file = LaunchConfiguration("params_file")
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([share, "param", "coverage_params.yaml"]),
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([share, "rviz", "coverage_preview.rviz"]),
        ),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_coverage_gui", default_value="true"),
        DeclareLaunchArgument("map", default_value=PathJoinSubstitution([
            share, "map", "indoor_map.yaml"])),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="coverage_preview_frame", output="screen",
            arguments=["--frame-id", "map", "--child-frame-id", "coverage_preview_origin"],
        ),
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             output="screen", parameters=[{'yaml_filename': LaunchConfiguration('map'),
                                           'use_sim_time': False}]),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_coverage_preview", output="screen",
            parameters=[{
                "use_sim_time": False, "autostart": True,
                "node_names": ["map_server"],
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('nav2_mission'), 'launch', 'coverage_control.launch.py'])),
            launch_arguments={'coverage_params_file': params_file,
                              'start_editor': 'true', 'enable_execution': 'false',
                              'use_gui': LaunchConfiguration('use_coverage_gui'),
                              'use_sim_time': 'false'}.items(),
        ),
        Node(
            package="rviz2", executable="rviz2", name="coverage_preview_rviz",
            output="screen", arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])

"""Standalone coverage planning and RViz polygon selection."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


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
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="coverage_preview_frame", output="screen",
            arguments=["--frame-id", "map", "--child-frame-id", "coverage_preview_origin"],
        ),
        Node(
            package="opennav_coverage", executable="opennav_coverage",
            name="coverage_server", output="screen",
            parameters=[params_file, {"use_sim_time": False}],
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_coverage_preview", output="screen",
            parameters=[{
                "use_sim_time": False, "autostart": True,
                "node_names": ["coverage_server"],
            }],
        ),
        Node(
            package="nav2_mission", executable="coverage_region_selector_node",
            name="coverage_region_selector", output="screen",
            parameters=[{
                "use_sim_time": False, "plan_only": True,
                "auto_plan_on_close": True, "default_frame_id": "map",
            }],
            remappings=[
                (name, "/coverage_region_selector/" + name)
                for name in [
                    "clear", "undo_last_point", "close_polygon",
                    "execute_polygon", "close_and_execute",
                ]
            ],
        ),
        Node(
            package="rviz2", executable="rviz2", name="coverage_preview_rviz",
            output="screen", arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])

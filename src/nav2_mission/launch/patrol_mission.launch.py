import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("nav2_mission")
    default_mission = os.path.join(pkg_share, "config", "patrol_points.yaml")

    mission_file = LaunchConfiguration("mission_file")
    navigate_action_name = LaunchConfiguration("navigate_action_name")
    detection_service_name = LaunchConfiguration("detection_service_name")
    model_path = LaunchConfiguration("model_path")
    save_dir = LaunchConfiguration("save_dir")
    conf_threshold = LaunchConfiguration("conf_threshold")

    return LaunchDescription([
        DeclareLaunchArgument(
            "mission_file",
            default_value=default_mission,
            description="Path to patrol yaml"
        ),
        DeclareLaunchArgument(
            "navigate_action_name",
            default_value="/navigate_to_pose",
            description="Nav2 action server name"
        ),
        DeclareLaunchArgument(
            "detection_service_name",
            default_value="/detect_facility",
            description="Facility detection service name"
        ),
        DeclareLaunchArgument(
            "model_path",
            default_value="/path/to/ludeng.pt",
            description="Path to YOLO model file"
        ),
        DeclareLaunchArgument(
            "save_dir",
            default_value="~/inspection_results",
            description="Directory to save detection results"
        ),
        DeclareLaunchArgument(
            "conf_threshold",
            default_value="0.5",
            description="Detection confidence threshold"
        ),
        Node(
            package="nav2_mission",
            executable="patrol_mission_node",
            name="patrol_mission_node",
            output="screen",
            parameters=[{
                "mission_file": mission_file,
                "navigate_action_name": navigate_action_name,
                "detection_service_name": detection_service_name,
            }]
        ),
        Node(
            package="nav2_mission",
            executable="facility_detection_node",
            name="facility_detection_node",
            output="screen",
            parameters=[{
                "model_path": model_path,
                "save_dir": save_dir,
                "conf_threshold": conf_threshold,
            }]
        )
    ])
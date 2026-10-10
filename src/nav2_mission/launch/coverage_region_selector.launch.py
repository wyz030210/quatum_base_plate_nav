from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    clicked_point_topic = LaunchConfiguration("clicked_point_topic")
    marker_topic = LaunchConfiguration("marker_topic")
    polygon_topic = LaunchConfiguration("polygon_topic")
    default_frame_id = LaunchConfiguration("default_frame_id")
    close_distance = LaunchConfiguration("close_distance")
    min_points = LaunchConfiguration("min_points")
    execute_action_name = LaunchConfiguration("execute_action_name")
    server_wait_timeout_sec = LaunchConfiguration("server_wait_timeout_sec")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "clicked_point_topic",
                default_value="/clicked_point",
                description="Point topic published by RViz Publish Point tool",
            ),
            DeclareLaunchArgument(
                "marker_topic",
                default_value="/coverage_ui/markers",
                description="MarkerArray topic for selected polygon visualization",
            ),
            DeclareLaunchArgument(
                "polygon_topic",
                default_value="/coverage_ui/polygon",
                description="PolygonStamped topic for selected polygon visualization",
            ),
            DeclareLaunchArgument(
                "default_frame_id",
                default_value="map",
                description="Frame id used if clicked points do not provide one",
            ),
            DeclareLaunchArgument(
                "close_distance",
                default_value="0.35",
                description="Distance threshold to first vertex to close polygon",
            ),
            DeclareLaunchArgument(
                "min_points",
                default_value="3",
                description="Minimum points required to form a polygon",
            ),
            DeclareLaunchArgument(
                "execute_action_name",
                default_value="/execute_coverage_navpath",
                description="Action name for manual polygon execution",
            ),
            DeclareLaunchArgument(
                "server_wait_timeout_sec",
                default_value="5.0",
                description="Timeout when waiting execute action server",
            ),
            Node(
                package="nav2_mission",
                executable="coverage_region_selector_node",
                name="coverage_region_selector",
                output="screen",
                parameters=[
                    {
                        "clicked_point_topic": clicked_point_topic,
                        "marker_topic": marker_topic,
                        "polygon_topic": polygon_topic,
                        "default_frame_id": default_frame_id,
                        "close_distance": close_distance,
                        "min_points": min_points,
                        "execute_action_name": execute_action_name,
                        "server_wait_timeout_sec": server_wait_timeout_sec,
                    }
                ],
            ),
        ]
    )

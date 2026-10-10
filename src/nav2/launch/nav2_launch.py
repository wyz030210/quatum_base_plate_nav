from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_wheeltec_nav2 = get_package_share_directory("wheeltec_nav2")
    pkg_nav2_mission = get_package_share_directory("nav2_mission")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    coverage_params_file = LaunchConfiguration("coverage_params_file")
    task_file = LaunchConfiguration("task_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    log_level = LaunchConfiguration("log_level")
    start_coverage_bridge = LaunchConfiguration("start_coverage_bridge")
    start_coverage_ui = LaunchConfiguration("start_coverage_ui")

    static_transforms = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_footprint_to_base_link",
            arguments=["0.0", "0.0", "0.0", "0", "0.0", "0.0", "base_footprint", "base_link"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_camera_tf",
            arguments=["0.0", "0.0", "0.0", "0", "0.0", "0.0", "base_link", "camera_link"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_lidar_tf",
            arguments=["0.0", "0.0", "0.15", "0", "0.0", "0.0", "base_link", "laser_link"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_rtk_tf",
            arguments=["0.0", "0.0", "0.0", "0", "0.0", "0.0", "base_link", "gps_link"],
        ),
    ]

    wheeltec_robot_node = Node(
        package="turn_on_wheeltec_robot",
        executable="wheeltec_robot_node",
        name="wheeltec_robot_node",
        output="screen",
        parameters=[
            {
                "usart_port_name": "/dev/wheeltec_controller",
                "serial_baud_rate": 115200,
                "robot_frame_id": "base_footprint",
                "odom_frame_id": "odom",
                "cmd_vel": "cmd_vel",
                "akm_cmd_vel": "none",
                "product_number": 0,
                "odom_x_scale": 1.0,
                "odom_y_scale": 1.0,
                "odom_z_scale_positive": 1.0,
                "odom_z_scale_negative": 1.0,
            }
        ],
    )

    wheeltec_nav_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_wheeltec_nav2, "/launch/bringup_launch.py"]
        ),
        launch_arguments={
            "namespace": namespace,
            "use_namespace": use_namespace,
            "map": map_yaml_file,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "coverage_params_file": coverage_params_file,
            "autostart": autostart,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
            "log_level": log_level,
        }.items(),
    )

    coverage_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_nav2_mission, "/launch/coverage_navpath_bridge.launch.py"]
        ),
        condition=IfCondition(start_coverage_bridge),
        launch_arguments={
            "task_file": task_file,
            "default_frame_id": "map",
            "base_frame_id": "base_link",
        }.items(),
    )

    coverage_ui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_nav2_mission, "/launch/coverage_region_selector.launch.py"]
        ),
        condition=IfCondition(start_coverage_ui),
        launch_arguments={
            "default_frame_id": "map",
            "execute_action_name": "/execute_coverage_navpath",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Top-level namespace",
            ),
            DeclareLaunchArgument(
                "use_namespace",
                default_value="false",
                description="Whether to apply a namespace to the navigation stack",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=f"{pkg_wheeltec_nav2}/map/fastlio_map.yaml",
                description="Full path to the map yaml file to load",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=(
                    f"{pkg_wheeltec_nav2}/param/wheeltec_params/param_mini_diff.yaml"
                ),
                description="Full path to the navigation params file",
            ),
            DeclareLaunchArgument(
                "coverage_params_file",
                default_value=f"{pkg_wheeltec_nav2}/param/coverage_params.yaml",
                description="Full path to the coverage params file",
            ),
            DeclareLaunchArgument(
                "task_file",
                default_value=f"{pkg_nav2_mission}/config/coverage_navpath_task.yaml",
                description="Coverage task YAML file",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically startup the nav2 stack",
            ),
            DeclareLaunchArgument(
                "use_composition",
                default_value="True",
                description="Whether to use composed bringup",
            ),
            DeclareLaunchArgument(
                "use_respawn",
                default_value="False",
                description="Whether to respawn if a node crashes",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Log level",
            ),
            DeclareLaunchArgument(
                "start_coverage_bridge",
                default_value="true",
                description="Whether to launch the nav_path coverage bridge",
            ),
            DeclareLaunchArgument(
                "start_coverage_ui",
                default_value="true",
                description="Whether to launch RViz clicked-point coverage region selector",
            ),
            wheeltec_robot_node,
            wheeltec_nav_bringup,
            coverage_bridge,
            coverage_ui,
            *static_transforms,
        ]
    )

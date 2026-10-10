from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include_launch(package_name, relative_path, launch_arguments=None):
    launch_path = PathJoinSubstitution([FindPackageShare(package_name), *relative_path])
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=(launch_arguments or {}).items(),
    )


def stage_actions(label, action):
    separator = "==================== LAUNCH STAGE: " + label + " ===================="
    return [LogInfo(msg=separator), action]


def delayed_stage(period, label, action):
    actions = stage_actions(label, action)
    if period <= 0.0:
        return actions
    return [TimerAction(period=period, actions=actions)]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    coverage_params_file = LaunchConfiguration("coverage_params_file")
    task_file = LaunchConfiguration("task_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    base_serial = include_launch(
        "turn_on_wheeltec_robot",
        ["launch", "base_serial.launch.py"],
    )

    livox_mid360 = include_launch(
        "livox_ros_driver2",
        ["launch_ROS2", "msg_MID360_launch.py"],
    )

    location = include_launch(
        "location",
        ["launch", "location_launch.py"],
        {"use_sim_time": use_sim_time},
    )

    ekf_odom = Node(
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
            )
        ],
        remappings=[
            ("/odometry/filtered", "/odometry/filtered_odom"),
        ],
    )

    ekf_map = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_map_node",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("wheeltec_nav2"),
                    "param",
                    "wheeltec_params",
                    "ekf_map.yaml",
                ]
            )
        ],
        remappings=[
            ("/odometry/filtered", "/odometry/filtered_map"),
        ],
    )

    nav2 = include_launch(
        "wheeltec_nav2",
        ["launch", "nav2.launch.py"],
        {
            "map": map_file,
            "params": params_file,
            "coverage_params_file": coverage_params_file,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
            "log_level": log_level,
        },
    )

    coverage_bridge = include_launch(
        "nav2_mission",
        ["launch", "coverage_navpath_bridge.launch.py"],
        {
            "task_file": task_file,
            "default_frame_id": "map",
            "base_frame_id": "base_link",
        },
    )

    coverage_region_selector = include_launch(
        "nav2_mission",
        ["launch", "coverage_region_selector.launch.py"],
        {
            "default_frame_id": "map",
            "execute_action_name": "/execute_coverage_navpath",
        },
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("wheeltec_nav2"), "map", "fastlio_map.yaml"]
                ),
                description="Package-relative path to the map yaml file to load",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("wheeltec_nav2"),
                        "param",
                        "wheeltec_params",
                        "param_mini_diff.yaml",
                    ]
                ),
                description="Package-relative path to the navigation params file",
            ),
            DeclareLaunchArgument(
                "coverage_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("wheeltec_nav2"), "param", "coverage_params.yaml"]
                ),
                description="Package-relative path to the coverage params file",
            ),
            DeclareLaunchArgument(
                "task_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("nav2_mission"),
                        "config",
                        "coverage_navpath_task.yaml",
                    ]
                ),
                description="Package-relative path to the coverage task YAML file",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Kept for launch compatibility with Nav2 bringup arguments",
            ),
            DeclareLaunchArgument(
                "use_composition",
                default_value="True",
                description="Kept for launch compatibility with Nav2 bringup arguments",
            ),
            DeclareLaunchArgument(
                "use_respawn",
                default_value="False",
                description="Kept for launch compatibility with Nav2 bringup arguments",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Kept for launch compatibility with Nav2 bringup arguments",
            ),
            *delayed_stage(0.0, "base_serial.launch.py", base_serial),
            *delayed_stage(2.0, "msg_MID360_launch.py", livox_mid360),
            *delayed_stage(4.0, "location_launch.py", location),
            *delayed_stage(6.0, "ekf_odom_node", ekf_odom),
            *delayed_stage(8.0, "ekf_map_node", ekf_map),
            *delayed_stage(10.0, "nav2.launch.py", nav2),
            *delayed_stage(14.0, "coverage_navpath_bridge.launch.py", coverage_bridge),
            *delayed_stage(
                16.0,
                "coverage_region_selector.launch.py",
                coverage_region_selector,
            ),
        ]
    )

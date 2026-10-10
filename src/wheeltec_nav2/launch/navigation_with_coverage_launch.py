import os
from typing import List

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def float_list_parameter(*values):
    yaml_list = ["["]
    for index, value in enumerate(values):
        if index:
            yaml_list.append(", ")
        yaml_list.append(value)
    yaml_list.append("]")
    return ParameterValue(yaml_list, value_type=List[float])


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    coverage_params_file = LaunchConfiguration("coverage_params_file")
    use_coverage = LaunchConfiguration("use_coverage")
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    max_forward_speed = LaunchConfiguration("max_forward_speed")
    max_reverse_speed = LaunchConfiguration("max_reverse_speed")
    max_yaw_rate = LaunchConfiguration("max_yaw_rate")
    max_linear_accel = LaunchConfiguration("max_linear_accel")
    max_yaw_accel = LaunchConfiguration("max_yaw_accel")

    min_linear_speed = PythonExpression(["0.0 - ", max_reverse_speed])
    min_yaw_rate = PythonExpression(["0.0 - ", max_yaw_rate])
    min_linear_accel = PythonExpression(["0.0 - ", max_linear_accel])
    min_yaw_accel = PythonExpression(["0.0 - ", max_yaw_accel])

    container_name_full = (namespace, "/", container_name)

    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
    ]

    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
        "coverage_server",
    ]

    param_substitutions = {
        "use_sim_time": use_sim_time,
        "autostart": autostart,
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    configured_coverage_params = ParameterFile(
        RewrittenYaml(
            source_file=coverage_params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM",
        "1",
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        description="Full path to the ROS2 params file for navigation nodes",
    )
    declare_coverage_params_file_cmd = DeclareLaunchArgument(
        "coverage_params_file",
        description="Full path to the ROS2 params file for coverage_server",
    )
    declare_use_coverage_cmd = DeclareLaunchArgument(
        "use_coverage",
        default_value="false",
        description=(
            "Launch the on-demand coverage trigger path: the coverage action bridge "
            "(/execute_coverage_navpath) and the RViz region selector (/clicked_point). "
            "Default false: ordinary indoor navigation does not start them. "
            "Set true to run full-coverage missions. "
            "Note: coverage_server itself is part of the Nav2 lifecycle manager and "
            "starts regardless of this flag; see doc/Coverage_Porting_Plan.md 4.3.1."
        ),
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )
    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Whether to use composed bringup",
    )
    declare_container_name_cmd = DeclareLaunchArgument(
        "container_name",
        default_value="nav2_container",
        description="Name of the nav2 component container",
    )
    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes when composition is disabled",
    )
    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log level",
    )

    controller_overrides = {
        "cmd_vel_topic": "cmd_vel_nav",
        "FollowPath.vx_max": ParameterValue(max_forward_speed, value_type=float),
        "FollowPath.vx_min": ParameterValue(min_linear_speed, value_type=float),
        "FollowPath.wz_max": ParameterValue(max_yaw_rate, value_type=float),
        "FollowPath.ax_max": ParameterValue(max_linear_accel, value_type=float),
        "FollowPath.ax_min": ParameterValue(min_linear_accel, value_type=float),
        "FollowPath.az_max": ParameterValue(max_yaw_accel, value_type=float),
    }

    velocity_smoother_overrides = {
        "max_velocity": float_list_parameter(
            max_forward_speed, "0.0", max_yaw_rate
        ),
        "min_velocity": float_list_parameter(
            min_linear_speed, "0.0", min_yaw_rate
        ),
        "max_accel": float_list_parameter(
            max_linear_accel, "0.0", max_yaw_accel
        ),
        "max_decel": float_list_parameter(
            min_linear_accel, "0.0", min_yaw_accel
        ),
    }

    behavior_overrides = {
        "max_rotational_vel": ParameterValue(max_yaw_rate, value_type=float),
        "rotational_acc_lim": ParameterValue(max_yaw_accel, value_type=float),
    }

    bt_overrides = {
        "default_nav_to_pose_bt_xml": PathJoinSubstitution(
            [
                FindPackageShare("wheeltec_nav2"),
                "behavior_trees",
                "navigate_to_pose_ackermann.xml",
            ]
        ),
        "default_nav_through_poses_bt_xml": PathJoinSubstitution(
            [
                FindPackageShare("wheeltec_nav2"),
                "behavior_trees",
                "navigate_through_poses_ackermann.xml",
            ]
        ),
    }

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(["not ", use_composition])),
        actions=[
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, controller_overrides],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, behavior_overrides],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, bt_overrides],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, velocity_smoother_overrides],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
            ),
            Node(
                package="opennav_coverage",
                executable="opennav_coverage",
                name="coverage_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, configured_coverage_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                ],
            ),
        ],
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package="nav2_controller",
                plugin="nav2_controller::ControllerServer",
                name="controller_server",
                parameters=[configured_params, controller_overrides],
                remappings=remappings,
            ),
            ComposableNode(
                package="nav2_planner",
                plugin="nav2_planner::PlannerServer",
                name="planner_server",
                parameters=[configured_params],
                remappings=remappings,
            ),
            ComposableNode(
                package="nav2_behaviors",
                plugin="behavior_server::BehaviorServer",
                name="behavior_server",
                parameters=[configured_params, behavior_overrides],
                remappings=remappings,
            ),
            ComposableNode(
                package="nav2_bt_navigator",
                plugin="nav2_bt_navigator::BtNavigator",
                name="bt_navigator",
                parameters=[configured_params, bt_overrides],
                remappings=remappings,
            ),
            ComposableNode(
                package="nav2_waypoint_follower",
                plugin="nav2_waypoint_follower::WaypointFollower",
                name="waypoint_follower",
                parameters=[configured_params],
                remappings=remappings,
            ),
            ComposableNode(
                package="nav2_velocity_smoother",
                plugin="nav2_velocity_smoother::VelocitySmoother",
                name="velocity_smoother",
                parameters=[configured_params, velocity_smoother_overrides],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
            ),
            ComposableNode(
                package="opennav_coverage",
                plugin="opennav_coverage::CoverageServer",
                name="coverage_server",
                parameters=[configured_params, configured_coverage_params],
                remappings=remappings,
            ),
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_navigation",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": lifecycle_nodes,
                    }
                ],
            ),
        ],
    )

    # ── 按需启动的覆盖触发链（仅 use_coverage:=true 时启动）──────────────
    # 这两个节点此前【只】出现在 robot_bringup/system.launch.py 与
    # nav2/launch/nav2_launch.py 里，而这两条 launch 都因引用不存在的
    # turn_on_wheeltec_robot 而失效 —— 结果是走室内入口时 coverage_server 起来了，
    # 但 /execute_coverage_navpath action 不存在、RViz 的 /clicked_point 无人监听，
    # 覆盖功能在整条链上是断的。详见 doc/Coverage_Porting_Plan.md §4.3.1。
    coverage_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nav2_mission"),
                    "launch",
                    "coverage_navpath_bridge.launch.py",
                ]
            )
        ),
        condition=IfCondition(use_coverage),
    )

    coverage_selector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nav2_mission"),
                    "launch",
                    "coverage_region_selector.launch.py",
                ]
            )
        ),
        condition=IfCondition(use_coverage),
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_coverage_params_file_cmd)
    ld.add_action(declare_use_coverage_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(
        DeclareLaunchArgument("max_forward_speed", default_value="0.25")
    )
    ld.add_action(
        DeclareLaunchArgument("max_reverse_speed", default_value="0.10")
    )
    ld.add_action(DeclareLaunchArgument("max_yaw_rate", default_value="0.50"))
    ld.add_action(
        DeclareLaunchArgument("max_linear_accel", default_value="0.40")
    )
    ld.add_action(DeclareLaunchArgument("max_yaw_accel", default_value="0.50"))
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)
    ld.add_action(coverage_bridge)
    ld.add_action(coverage_selector)
    return ld

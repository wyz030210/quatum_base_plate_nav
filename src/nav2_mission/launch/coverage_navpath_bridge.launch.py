import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("nav2_mission")
    default_task_file = os.path.join(pkg_share, "config", "coverage_navpath_task.yaml")

    task_file = LaunchConfiguration("task_file")
    default_frame_id = LaunchConfiguration("default_frame_id")
    execute_action_name = LaunchConfiguration("execute_action_name")
    compute_action_name = LaunchConfiguration("compute_action_name")
    follow_path_action_name = LaunchConfiguration("follow_path_action_name")
    server_wait_timeout_sec = LaunchConfiguration("server_wait_timeout_sec")
    compute_timeout_sec = LaunchConfiguration("compute_timeout_sec")
    follow_timeout_sec = LaunchConfiguration("follow_timeout_sec")
    enable_lifecycle_checks = LaunchConfiguration("enable_lifecycle_checks")
    trim_path_to_robot = LaunchConfiguration("trim_path_to_robot")
    base_frame_id = LaunchConfiguration("base_frame_id")
    generate_headland = LaunchConfiguration("generate_headland")
    generate_route = LaunchConfiguration("generate_route")
    generate_path = LaunchConfiguration("generate_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "task_file",
                default_value=default_task_file,
                description="YAML task file path for default coverage polygon",
            ),
            DeclareLaunchArgument(
                "default_frame_id",
                default_value="map",
                description="Default frame id for coverage goals",
            ),
            DeclareLaunchArgument(
                "execute_action_name",
                default_value="/execute_coverage_navpath",
                description="Action name exposed by the bridge node",
            ),
            DeclareLaunchArgument(
                "compute_action_name",
                default_value="/compute_coverage_path",
                description="ComputeCoveragePath action name",
            ),
            DeclareLaunchArgument(
                "follow_path_action_name",
                default_value="/follow_path",
                description="FollowPath action name",
            ),
            DeclareLaunchArgument(
                "server_wait_timeout_sec",
                default_value="10.0",
                description="Timeout for waiting dependent action servers",
            ),
            DeclareLaunchArgument(
                "compute_timeout_sec",
                default_value="120.0",
                description=(
                    "Timeout for ComputeCoveragePath result (F2C planning). "
                    "Raised 60->120 to match coverage_server's planning budget."
                ),
            ),
            DeclareLaunchArgument(
                "follow_timeout_sec",
                # ⚠️ 这是【覆盖任务的总执行超时】，不是"等待服务上线"的超时。
                # 它用在 coverage_navpath_bridge_node.py:318-323 的
                #   self._wait_future(follow_result_future, self._follow_timeout_sec, ...)
                # 而 FollowPath 要跑完整条覆盖路径才返回结果。
                # 超时分支带 cancel_fn，会【主动取消 FollowPath】→ 车中途停下。
                #
                # 200-300 m^2 场地按 swath 0.8 m 估算：纯行驶约 1000-1500 s，
                # 加掉头约 1300-2000 s（22-33 分钟）。
                # 因此原来的 300.0（5 分钟）必然在覆盖跑到一半时中止任务。
                default_value="3600.0",
                description="Timeout for FollowPath result (total coverage run time)",
            ),
            DeclareLaunchArgument(
                "enable_lifecycle_checks",
                default_value="true",
                description="Whether to check lifecycle state before dispatching actions",
            ),
            DeclareLaunchArgument(
                "trim_path_to_robot",
                default_value="true",
                description="Trim nav_path to robot-nearest point before FollowPath",
            ),
            DeclareLaunchArgument(
                "base_frame_id",
                default_value="base_link",
                description="Base frame used for path trimming lookup",
            ),
            DeclareLaunchArgument(
                "generate_headland",
                # 阿克曼车掉头需要地头空间（见 coverage_params.yaml 的
                # default_headland_width = min_turning_radius + 0.10）。
                # 不开地头，车会在地块边缘转不过来。
                default_value="true",
                description="Whether to generate headland before swath generation",
            ),
            DeclareLaunchArgument(
                "generate_route",
                default_value="true",
                description="Whether to generate route ordering for swaths",
            ),
            DeclareLaunchArgument(
                "generate_path",
                default_value="true",
                description="Whether to generate nav_path for FollowPath",
            ),
            Node(
                package="nav2_mission",
                executable="coverage_navpath_bridge_node",
                name="coverage_navpath_bridge",
                output="screen",
                parameters=[
                    {
                        "task_file": task_file,
                        "default_frame_id": default_frame_id,
                        "execute_action_name": execute_action_name,
                        "compute_action_name": compute_action_name,
                        "follow_path_action_name": follow_path_action_name,
                        "server_wait_timeout_sec": server_wait_timeout_sec,
                        "compute_timeout_sec": compute_timeout_sec,
                        "follow_timeout_sec": follow_timeout_sec,
                        "enable_lifecycle_checks": enable_lifecycle_checks,
                        "trim_path_to_robot": trim_path_to_robot,
                        "base_frame_id": base_frame_id,
                        "generate_headland": generate_headland,
                        "generate_route": generate_route,
                        "generate_path": generate_path,
                    }
                ],
            ),
        ]
    )

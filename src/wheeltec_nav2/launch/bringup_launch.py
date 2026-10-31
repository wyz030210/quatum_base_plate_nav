import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,                           
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString


def generate_launch_description():
    # 获取nav2_bringup包的路径
    bringup_dir = get_package_share_directory('nav2_bringup')
    local_dir = get_package_share_directory('wheeltec_nav2')
    launch_dir = os.path.join(bringup_dir, 'launch')
    local_launch_dir = os.path.join(local_dir, 'launch')

    # 获取launch参数
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    coverage_params_file = LaunchConfiguration('coverage_params_file')
    use_coverage = LaunchConfiguration('use_coverage')
    use_task_control = LaunchConfiguration('use_task_control')
    use_coverage_server = LaunchConfiguration('use_coverage_server')
    use_amcl = LaunchConfiguration('use_amcl')
    task_file = LaunchConfiguration('task_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    max_forward_speed = LaunchConfiguration('max_forward_speed')
    max_reverse_speed = LaunchConfiguration('max_reverse_speed')
    max_yaw_rate = LaunchConfiguration('max_yaw_rate')
    max_linear_accel = LaunchConfiguration('max_linear_accel')
    max_yaw_accel = LaunchConfiguration('max_yaw_accel')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # 获取参数文件并进行替换
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    #只有在`use_namespace`为True时才适用。
    #在配置文件'nav2_multirobot_params.yaml'中，'<robot_namespace
    #'关键字应被替换为'namespace'启动参数，作为默认值和示例。
    #用户定义的配置文件应包含用于替换的'<robot_namespace>'关键字

    params_file = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': ('/', namespace)},
        condition=IfCondition(use_namespace))

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_coverage_params_file_cmd = DeclareLaunchArgument(
        'coverage_params_file',
        default_value=os.path.join(local_dir, 'param', 'coverage_params.yaml'),
        description='Full path to the ROS2 parameters file to use for coverage server')

    declare_use_coverage_cmd = DeclareLaunchArgument(
        'use_coverage',
        default_value='false',
        description='Launch the on-demand coverage trigger path '
                    '(coverage action bridge + RViz region selector). '
                    'Default false for ordinary indoor navigation.')

    declare_use_amcl_cmd = DeclareLaunchArgument(
        'use_amcl', default_value='true',
        description='Disable when an external global EKF supplies map -> odom')
    declare_use_coverage_server_cmd = DeclareLaunchArgument(
        'use_coverage_server', default_value='true',
        description='Keep legacy indoor behavior; outdoor follows use_coverage')
    declare_task_file_cmd = DeclareLaunchArgument(
        'task_file', default_value=os.path.join(
            get_package_share_directory('nav2_mission'), 'config', 'coverage_navpath_task.yaml'))

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    declare_max_forward_speed_cmd = DeclareLaunchArgument(
        'max_forward_speed', default_value='0.25')
    declare_max_reverse_speed_cmd = DeclareLaunchArgument(
        'max_reverse_speed', default_value='0.10')
    declare_max_yaw_rate_cmd = DeclareLaunchArgument(
        'max_yaw_rate', default_value='0.50')
    declare_max_linear_accel_cmd = DeclareLaunchArgument(
        'max_linear_accel', default_value='0.40')
    declare_max_yaw_accel_cmd = DeclareLaunchArgument(
        'max_yaw_accel', default_value='0.50')

    # 启动导航功能节点组
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam_launch.py')),
            condition=IfCondition(slam),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'use_respawn': use_respawn,
                              'params_file': params_file}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir,
                                                       'localization_launch.py')),
            condition=IfCondition(PythonExpression([
                "'", slam, "'.lower() not in ('true', '1') and '",
                use_amcl, "'.lower() in ('true', '1')"])),
            launch_arguments={'namespace': namespace,
                              'map': map_yaml_file,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),

        # The RTK EKF owns map -> odom. Load only the map, not a second localizer.
        GroupAction(
            condition=IfCondition(PythonExpression([
                "'", slam, "'.lower() not in ('true', '1') and '",
                use_amcl, "'.lower() not in ('true', '1')"])),
            actions=[
                Node(
                    condition=UnlessCondition(use_composition),
                    package='nav2_map_server', executable='map_server', name='map_server',
                    output='screen', parameters=[configured_params],
                    respawn=use_respawn, respawn_delay=2.0, remappings=remappings,
                    arguments=['--ros-args', '--log-level', log_level]),
                Node(
                    condition=UnlessCondition(use_composition),
                    package='nav2_lifecycle_manager', executable='lifecycle_manager',
                    name='lifecycle_manager_localization', output='screen',
                    parameters=[{'use_sim_time': use_sim_time, 'autostart': autostart,
                                 'node_names': ['map_server']}]),
                LoadComposableNodes(
                    condition=IfCondition(use_composition),
                    target_container=(namespace, '/nav2_container'),
                    composable_node_descriptions=[
                        ComposableNode(
                            package='nav2_map_server', plugin='nav2_map_server::MapServer',
                            name='map_server', parameters=[configured_params],
                            remappings=remappings),
                        ComposableNode(
                            package='nav2_lifecycle_manager',
                            plugin='nav2_lifecycle_manager::LifecycleManager',
                            name='lifecycle_manager_localization',
                            parameters=[{'use_sim_time': use_sim_time, 'autostart': autostart,
                                         'node_names': ['map_server']}]),
                    ]),
            ]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(local_launch_dir, 'navigation_with_coverage_launch.py')
            ),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'coverage_params_file': coverage_params_file,
                              'use_coverage': use_coverage,
                              'use_task_control': use_task_control,
                              'use_coverage_server': use_coverage_server,
                              'task_file': task_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container',
                              'log_level': log_level,
                              'max_forward_speed': max_forward_speed,
                              'max_reverse_speed': max_reverse_speed,
                              'max_yaw_rate': max_yaw_rate,
                              'max_linear_accel': max_linear_accel,
                              'max_yaw_accel': max_yaw_accel}.items()),
    ])

    #创建 LaunchDescription 并进行填充
    ld = LaunchDescription()

    # 设置环境变量
    ld.add_action(stdout_linebuf_envvar)

    #声明启动选项
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_coverage_params_file_cmd)
    ld.add_action(declare_use_coverage_cmd)
    ld.add_action(DeclareLaunchArgument('use_task_control', default_value='false'))
    ld.add_action(declare_use_amcl_cmd)
    ld.add_action(declare_use_coverage_server_cmd)
    ld.add_action(declare_task_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_max_forward_speed_cmd)
    ld.add_action(declare_max_reverse_speed_cmd)
    ld.add_action(declare_max_yaw_rate_cmd)
    ld.add_action(declare_max_linear_accel_cmd)
    ld.add_action(declare_max_yaw_accel_cmd)

    # 添加导航功能节点组
    ld.add_action(bringup_cmd_group)

    return ld

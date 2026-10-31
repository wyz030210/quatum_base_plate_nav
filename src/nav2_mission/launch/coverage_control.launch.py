"""Coverage controls and their dedicated compute-only preview planner."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile, ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    clock = LaunchConfiguration('use_sim_time')
    params = ParameterFile(RewrittenYaml(
        source_file=LaunchConfiguration('coverage_params_file'), root_key='coverage_preview',
        param_rewrites={'use_sim_time': clock}, convert_types=True), allow_substs=True)
    return LaunchDescription([
        DeclareLaunchArgument('coverage_params_file'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_execution', default_value='false'),
        DeclareLaunchArgument('start_editor', default_value='false'),
        DeclareLaunchArgument('use_gui', default_value='true'),
        Node(package='nav2_mission', executable='coverage_editor_node',
             name='coverage_editor', condition=IfCondition(LaunchConfiguration('start_editor')),
             parameters=[{'use_sim_time': ParameterValue(clock, value_type=bool)}],
             output='screen'),
        Node(package='opennav_coverage', executable='opennav_coverage',
             namespace='coverage_preview', name='coverage_server',
             parameters=[params], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             namespace='coverage_preview', name='lifecycle_manager', parameters=[{
                 'use_sim_time': ParameterValue(clock, value_type=bool), 'autostart': True,
                 'node_names': ['coverage_server']}], output='screen'),
        Node(package='nav2_mission', executable='coverage_control_gui',
             name='coverage_controls', condition=IfCondition(LaunchConfiguration('use_gui')),
             parameters=[{'use_sim_time': ParameterValue(clock, value_type=bool),
                          'enable_execution': ParameterValue(
                              LaunchConfiguration('enable_execution'), value_type=bool)}],
             output='screen'),
    ])

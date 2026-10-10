import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('quantum_ark_driver')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='quantum_ark_driver',
            executable='quantum_ark_node',
            name='quantum_ark_node',
            output='screen',
            parameters=[params_file]
        ),
    ])

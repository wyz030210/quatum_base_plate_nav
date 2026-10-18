import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    #获取包路径
    pkg_location = get_package_share_directory('location')
    location_launch_dir = os.path.join(pkg_location, 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('serial_port', default_value=(
            '/dev/serial/by-id/'
            'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
        )),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        DeclareLaunchArgument('datum_lat', default_value='29.4898045'),
        DeclareLaunchArgument('datum_lon', default_value='106.5676103'),

        Node(
            package='location',
            executable='rtk_localization_node',
            name='rtk_localization',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': ParameterValue(LaunchConfiguration('baud_rate'), value_type=int),
                'datum_lat': ParameterValue(LaunchConfiguration('datum_lat'), value_type=float),
                'datum_lon': ParameterValue(LaunchConfiguration('datum_lon'), value_type=float),
                'rtk_frame_id': 'gps_link',  
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                'auto_set_origin': False,
                'publish_tf': False,
                'publish_odom': True,
                'publish_navsat': True,
                'publish_raw': True,
            }]
       ),
    ])

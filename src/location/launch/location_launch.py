import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

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

        Node(
            package='location',
            executable='rtk_localization_node',
            name='rtk_localization',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'baud_rate': 115200,
                'datum_lat': 29.4898045, 
                'datum_lon': 106.5676103,
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

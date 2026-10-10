import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    #获取wheeltec_robot_nav2包的路径
    wheeltec_nav_dir = get_package_share_directory('wheeltec_nav2')
    wheeltec_nav_launch_dir = os.path.join(wheeltec_nav_dir, 'launch')
    #获取map路径
    map_dir = os.path.join(wheeltec_nav_dir, 'map')
    map_file = LaunchConfiguration('map', default=os.path.join(
        map_dir, 'fastlio_map.yaml'))
    
    map_server_config_path = LaunchConfiguration('map')

    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[{'use_sim_time':use_sim_time,
                    'yaml_filename':map_server_config_path}]
    )

    lifecycle_nodes = ['map_server']
    
    autostart = True

    start_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True, 
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    #获取参数文件路径
    param_dir = os.path.join(wheeltec_nav_dir, 'param','wheeltec_params')
    param_file = LaunchConfiguration('params', default=os.path.join(
        param_dir, 'param_mini_diff.yaml'))
    #获取rviz文件路径
    rviz_config_dir = os.path.join(wheeltec_nav_dir,'rviz')
    rviz_config_file = os.path.join(rviz_config_dir,'map.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params',
            default_value=param_file,
            description='Full path to param file to load'),

        #启动rviz节点
        Node(
            name='rviz2',
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file]
        ),
        
        # TF变换
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='footprint_to_link',
            arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'base_link', 'base_footprint']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='livox_to_base',
            arguments=['0.11', '0.0', '0.615', '0', '0', '0', 'base_footprint', 'livox_frame']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_base',
            arguments=['0.1', '0.0', '0.28', '0', '0', '0', 'base_footprint', 'camera_link']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rtk_to_base',
            arguments=['0.319', '0.0', '0.11', '0', '0', '0', 'base_footprint', 'gps_link']
        ),
        
        # ============================================================
        # 第一步：CustomMsg → PointCloud2 (用于导航，不影响FastLIO2)
        # ============================================================
        Node(
            package='wheeltec_nav2',
            executable='livox_to_pointcloud2.py',
            name='livox_custom_to_pointcloud2'
        ),
        
        # ============================================================
        # 第二步：PointCloud2 → /scan (使用转换后的点云生成激光雷达数据)
        # ============================================================
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': 'livox_frame',
                'transform_tolerance': 0.01,
                'min_height': -0.5,
                'max_height': 0.5,
                'angle_min': -3.14159265359,
                'angle_max': 3.14159265359,
                'angle_increment': 0.00872664626,
                'scan_time': 0.1,
                'range_min': 0.1,
                'range_max': 30.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }],
            remappings=[
                ('cloud_in', '/livox/pointcloud2'),  # 使用转换后的点云
                ('scan', '/scan')                    # 输出 /scan
            ]
        ),

        #启动导航核心       
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [wheeltec_nav_launch_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': param_file}.items(),
        ),
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='LiDAR serial port device'
        ),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # LiDAR Driver
        Node(
            package='lslidar_driver',
            executable='lslidar_driver_node',
            name='lslidar_driver_node',
            output='screen',
            parameters=[{
                'interface_selection': 'serial',
                'serial_port_': LaunchConfiguration('serial_port'),
                'lidar_name': 'N10_P',
                'baud_rate_': 460800,
                'frame_id': 'laser_link',
                'scan_topic': '/scan',
                'pointcloud_topic': '/lslidar_point_cloud',
                'min_range': 0.0,
                'max_range': 0.8,
                'pubScan': True,
                'pubPointCloud2': True,
                'use_gps_ts': False,
                'compensation': False,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('/lslidar_point_cloud', '/lslidar_point_cloud'),
            ]
        ),
        
        # Pole Detection Node (Production Mode - Debug OFF)
        Node(
            package='pole_detection',
            executable='pole_detection_node',
            name='pole_detection',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('pole_detection'),
                    'config',
                    'production_params.yaml'
                ]),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                ('/lslidar_point_cloud', '/lslidar_point_cloud'),
                ('/detected_objects', '/detected_objects'),
                ('/detected_poles', '/detected_poles'),
            ]
        ),
        
        # Gripper Control Action Server
        Node(
            package='pole_detection',
            executable='action_server',
            name='gripper_control_action_server',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('/detected_objects', '/detected_objects'),
                ('/task/gripper_control', '/task/gripper_control'),
            ]
        ),
        
        # Static Transform Publisher: base_link -> laser_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_link'],
            output='screen'
        ),
        
        LogInfo(msg=[
            '🚀 Pole Detection System launched in PRODUCTION mode ',
            '(debug publishing disabled)'
        ]),
    ])
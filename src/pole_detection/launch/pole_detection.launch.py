from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz with debug visualization'
        ),
        
        DeclareLaunchArgument(
            'lidar_model',
            default_value='n10p',
            description='LiDAR model: n10p, n10, m10, m10p (default: n10p)'
        ),
        
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='LiDAR serial port device (used if not specified in YAML)'
        ),
        
        # LiDAR Driver - Always launch for real hardware operation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('lslidar_driver'),
                    'launch',
                    PythonExpression([
                        "'lsn10p_launch.py' if '", LaunchConfiguration('lidar_model'), "' == 'n10p' else ",
                        "'lsn10_launch.py' if '", LaunchConfiguration('lidar_model'), "' == 'n10' else ",
                        "'lsm10p_uart_launch.py' if '", LaunchConfiguration('lidar_model'), "' == 'm10p' else ",
                        "'lsm10_uart_launch.py'"
                    ])
                ])
            ])
        ),
        
        # Pole Detection Node (UNIFIED Configuration)
        Node(
            package='pole_detection',
            executable='pole_detection_node',
            name='pole_detection',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('pole_detection'),
                    'config',
                    'params.yaml'
                ]),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                ('/lslidar_point_cloud', '/lslidar_point_cloud'),
            ]
        ),
        
        # TrackPoles Action Server (Behavior Tree Compatible)
        Node(
            package='pole_detection',
            executable='action_server',
            name='track_poles_action_server',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('/detected_objects', '/detected_objects'),
            ]
        ),
        
        # Static Transform Publisher - Use laser_link to match LiDAR driver
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_link'],
            output='screen'
        ),
        
        # RViz with Pre-configured Debug Displays
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('pole_detection'),
                'rviz',
                'debug.rviz'
            ])],
            condition=IfCondition(LaunchConfiguration('start_rviz')),
            output='screen',
            additional_env={'QT_QPA_PLATFORM': 'xcb'}  # Force X11 to avoid Wayland/Snap conflicts
        ),
        
        LogInfo(msg=[
            '🔍 Pole Detection System launched with UNIFIED configuration ',
            '(Combines production stability with debug visibility)\n',
            '📡 LiDAR Model: ', LaunchConfiguration('lidar_model'), '\n',
            '🔌 Serial Port: ', LaunchConfiguration('serial_port')
        ]),
    ])
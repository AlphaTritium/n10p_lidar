from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, LifecycleNode
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
        
        # LiDAR Driver - using n10p_driver approach with YAML configuration
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('lslidar_driver'),
                    'launch',
                    'lsn10p_launch.py'
                ])
            ])
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
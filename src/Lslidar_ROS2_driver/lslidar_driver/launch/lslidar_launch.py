#!/usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        Node(
            package='lslidar_driver',
            executable='lslidar_driver_node',
            name='lslidar_driver_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'interface_selection': 'serial',
                'serial_port_': '/dev/ttyACM0',
                'lidar_name': 'N10_P',
                'baud_rate_': 460800,
                'frame_id': 'laser_link',
                'scan_topic': '/scan',
                'pointcloud_topic': '/lslidar_point_cloud',
                'min_range': 0.0,
                'max_range': 10.0,
                'pubScan': True,
                'pubPointCloud2': True,
                'use_gps_ts': False,
                'compensation': False,
            }]
        ),
    ])
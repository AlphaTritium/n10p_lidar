from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='LiDAR serial port'
        ),
        DeclareLaunchArgument(
            'lidar_name',
            default_value='N10_P',
            description='LiDAR model name'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='460800',
            description='Serial baudrate'
        ),

        Node(
            package='lslidar_driver',
            executable='lslidar_driver_node',
            name='lslidar_driver_node',
            output='screen',
            parameters=[{
                'interface_selection': 'serial',
                'serial_port_': LaunchConfiguration('serial_port'),
                'lidar_name': LaunchConfiguration('lidar_name'),
                'baud_rate_': LaunchConfiguration('baudrate'),
                'frame_id': 'laser_link',
                'scan_topic': '/scan',
                'pointcloud_topic': '/lslidar_point_cloud',
                'min_range': 0.15,
                'max_range': 12.0,
                'pubScan': True,
                'pubPointCloud2': True,
                'use_gps_ts': False,
                'compensation': False,
            }]
        ),

        Node(
            package='sim',
            executable='lidar_pointcloud_processor',
            name='lidar_pointcloud_processor',
            output='screen',
            parameters=[{
                'input_topic': '/lslidar_point_cloud',
                'output_scan_topic': '/scan_processed',
                'output_cloud_topic': '/cloud_processed',
                'range_min': 0.15,
                'range_max': 12.0,
                'z_min': -1.0,
                'z_max': 2.0,
                'voxel_leaf_size': 0.05,
                'cluster_min_size': 10,
                'cluster_max_size': 500,
                'cluster_tolerance': 0.2,
                'publish_filtered_cloud': True,
                'detect_objects': True,
                'classify_surfaces': False,
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('sim'),
                'config',
                'pcl_detection.rviz'
            ])],
            output='screen'
        ),
    ])
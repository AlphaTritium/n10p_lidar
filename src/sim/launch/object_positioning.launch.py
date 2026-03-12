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
            }]
        ),

        # Point Cloud Processor with Circle Detection
        Node(
            package='sim',
            executable='lidar_pointcloud_processor',
            name='lidar_pointcloud_processor',
            output='screen',
            parameters=[{
                'input_topic': '/lslidar_point_cloud',
                'output_scan_topic': '/scan_processed',
                'range_min': 0.0,
                'range_max': 0.8,
                'z_min': -0.3,
                'z_max': 0.3,
                'voxel_leaf_size': 0.01,
                'cluster_min_size': 6,
                'cluster_max_size': 100,
                'cluster_tolerance': 0.02,
                'publish_filtered_cloud': True,
                'detect_objects': True,
                'classify_surfaces': False,
                'enable_circle_detection': True,
                'expected_object_diameter': 0.025,
                'detection_tolerance': 0.008,
                'accumulated_scans': 5,
                'enable_tracking': False,
            }]
        ),

        # Object Positioning Action Server
        Node(
            package='sim',
            executable='object_positioning_server',
            name='object_positioning_server',
            output='screen',
            parameters=[{
                'stability_threshold': 0.005,
                'required_stable_frames': 10,
            }]
        ),

        # RViz for Visualization
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
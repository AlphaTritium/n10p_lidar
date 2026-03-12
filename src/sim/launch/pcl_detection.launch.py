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
                'min_range': 0.0,
                'max_range': 10.0,
                'pubScan': True,
                'pubPointCloud2': True,
                'use_gps_ts': False,
                'compensation': False,
                'angle_disable_min': 0.0,
                'angle_disable_max': 0.0,
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
                'range_min': 0.25,
                'range_max': 1.0,
                'z_min': -0.5,
                'z_max': 0.5,
                'voxel_leaf_size': 0.005,
                'cluster_min_size': 3,
                'cluster_max_size': 100,
                'cluster_tolerance': 0.03,
                'publish_filtered_cloud': True,
                'detect_objects': True,
                'classify_surfaces': False,
                'enable_circle_detection': True,
                'expected_object_diameter': 0.025,
                'detection_tolerance': 0.005,
                'accumulated_scans': 5,
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
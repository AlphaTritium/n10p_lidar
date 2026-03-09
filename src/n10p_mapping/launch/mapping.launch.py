from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Lidar driver node with N10_P model to force serial mode
        Node(
            package='lslidar_driver',
            executable='lslidar_driver_node',
            name='lslidar_driver_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyACM0',
                'baudrate': 230400,
                'frame_id': 'laser',
                'lidar_name': 'N10_P',        # This forces serial mode
                # If you still get UDP timeouts, try adding:
                # 'use_serial': True,
            }]
        ),
        # SLAM Toolbox for online asynchronous mapping
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                # You can also specify a config file, e.g.:
                # 'slam_config_file': '/path/to/mapper_params_online_async.yaml'
            }],
            remappings=[('/scan', '/scan')]
        )
    ])
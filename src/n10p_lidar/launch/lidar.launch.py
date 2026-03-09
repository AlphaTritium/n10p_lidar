# my_n10p_mapping/launch/n10p_mapping_launch.py

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # -------------------- Launch Arguments --------------------
    # Lidar driver parameters
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for the N10P lidar'
    )
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='230400',
        description='Baudrate for the lidar'
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser_frame',
        description='Frame ID for the lidar scans'
    )
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Topic to publish laser scans'
    )
    angle_min_arg = DeclareLaunchArgument(
        'angle_min',
        default_value='-3.14159',
        description='Minimum scan angle (radians)'
    )
    angle_max_arg = DeclareLaunchArgument(
        'angle_max',
        default_value='3.14159',
        description='Maximum scan angle (radians)'
    )
    range_min_arg = DeclareLaunchArgument(
        'range_min',
        default_value='0.15',
        description='Minimum range (m)'
    )
    range_max_arg = DeclareLaunchArgument(
        'range_max',
        default_value='12.0',
        description='Maximum range (m)'
    )

    # Mapping parameters (slam_toolbox)
    slam_config_file_arg = DeclareLaunchArgument(
        'slam_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('slam_toolbox'),
            'config', 'mapper_params_online_async.yaml'
        ]),
        description='Full path to the slam_toolbox configuration YAML file'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    # -------------------- Nodes --------------------
    # 1. N10P Lidar Driver Node (from lslidar_driver package)
    lidar_driver_node = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',  # adjust if your driver uses a different name
        name='lslidar_driver_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'frame_id': LaunchConfiguration('frame_id'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'angle_min': LaunchConfiguration('angle_min'),
            'angle_max': LaunchConfiguration('angle_max'),
            'range_min': LaunchConfiguration('range_min'),
            'range_max': LaunchConfiguration('range_max'),
            'lidar_name': 'N10_P',  
            'use_serial': True,
        }],
        # If the driver is a lifecycle node and you want to start it inactive,
        # add: arguments=['--ros-args', '--log-level', 'info'], and manage state later.
        # For automatic activation, you can use a LifecycleNodeManager.
        # See commented block below.
    )

    # 2. SLAM Toolbox Node (online asynchronous mapping)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[('/scan', LaunchConfiguration('scan_topic'))]
    )

    # -------------------- Lifecycle Management (Optional) --------------------
    # If your lidar driver is a lifecycle node and you want to start it in the 'unconfigured' state,
    # you can use a LifecycleNodeManager to handle transitions. Uncomment and adapt if needed.
    #
    # from launch_ros.actions import LifecycleNode
    # from launch_ros.actions import Node
    #
    # lidar_driver_node = LifecycleNode(
    #     package='lslidar_driver',
    #     executable='lslidar_driver_node',
    #     name='lslidar_driver_node',
    #     namespace='',
    #     output='screen',
    #     parameters=[{...}],
    # )
    #
    # # Then you can use a separate node or a command to activate it later:
    # # ros2 lifecycle set /lslidar_driver_node configure
    # # ros2 lifecycle set /lslidar_driver_node activate
    # # Or use the lifecycle_node_manager:
    # lifecycle_manager = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_lidar',
    #     output='screen',
    #     parameters=[{'node_names': ['lslidar_driver_node']}]
    # )

    return LaunchDescription([
        # Declare arguments
        serial_port_arg,
        baudrate_arg,
        frame_id_arg,
        scan_topic_arg,
        angle_min_arg,
        angle_max_arg,
        range_min_arg,
        range_max_arg,
        slam_config_file_arg,
        use_sim_time_arg,

        # Nodes
        lidar_driver_node,
        slam_toolbox_node,
        # lifecycle_manager,  # uncomment if using lifecycle
    ])
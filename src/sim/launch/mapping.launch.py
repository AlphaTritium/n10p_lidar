from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time'
    )
    use_simulation_arg = DeclareLaunchArgument(
        'use_simulation', default_value='false',
        description='Launch in simulation mode (Gazebo)'
    )
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz', default_value='true',
        description='Start RViz automatically'
    )
    
    interface_selection_arg = DeclareLaunchArgument(
        'interface_selection',
        default_value='serial',
        description='Interface: net or serial'
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port_',
        default_value='/dev/ttyUSB0',
        description='Serial port device'
    )
    lidar_name_arg = DeclareLaunchArgument(
        'lidar_name',
        default_value='N10_P',
        description='LiDAR model name'
    )
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='460800',
        description='Serial baudrate'
    )

    # Real driver (official launch) – forward the arguments
    real_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lslidar_driver'),
                'launch',
                'lslidar_launch.py'
            ])
        ]),
        condition=UnlessCondition(LaunchConfiguration('use_simulation')),
        launch_arguments={
            'interface_selection': LaunchConfiguration('interface_selection'),
            'serial_port_': LaunchConfiguration('serial_port_'),
            'lidar_name': LaunchConfiguration('lidar_name'),
            'baudrate': LaunchConfiguration('baudrate'),
        }.items()
    )

    # Simulation (Gazebo)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sim'),
                'launch',
                'gazebo_sim.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('use_simulation')),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # Processor node
    pointcloud_processor_node = Node(
        package='sim',
        executable='lidar_pointcloud_processor',
        name='lidar_pointcloud_processor',
        output='screen',
        parameters=[{
            'input_topic': '/lslidar_point_cloud',  # From driver
            'output_scan_topic': '/scan_processed',
            'output_cloud_topic': '/cloud_processed',
            'range_min': 0.15,
            'range_max': 12.0,
            'z_min': -0.5,
            'z_max': 2.0,
            'voxel_leaf_size': 0.05,
            'cluster_min_size': 5,
            'cluster_max_size': 1000,
            'cluster_tolerance': 0.2,
            'publish_filtered_cloud': True,
            'detect_objects': True,
            'classify_surfaces': True,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Detector node (uses processed scan)
    detector_node = Node(
        package='sim',
        executable='lidar_app_node',
        name='lidar_detector',
        output='screen',
        parameters=[{
            'input_scan_topic': '/scan_processed',  # Changed from /scan_filtered
            'cluster_tolerance': 0.1,
            'min_cluster_size': 5,
            'max_cluster_size': 100,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('sim'),
                'config',
                'slam_toolbox_params.yaml'
            ]),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[('/scan', '/scan_processed')],  # Changed from /scan_filtered
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('sim'),
            'config',
            'rviz_config.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('start_rviz'))
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_simulation_arg,
        start_rviz_arg,
        interface_selection_arg,
        serial_port_arg,
        lidar_name_arg,
        baudrate_arg,
        real_driver_launch,
        sim_launch,
        pointcloud_processor_node,
        detector_node,
        slam_node,
        rviz_node,
    ])
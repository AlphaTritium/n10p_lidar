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
    map_yaml_file_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            FindPackageShare('sim'),
            'maps',
            'my_map.yaml'
        ]),
        description='Full path to map yaml file'
    )

    # Real driver
    real_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lslidar_driver'),
                'launch',
                'lslidar_launch.py'
            ])
        ]),
        condition=UnlessCondition(LaunchConfiguration('use_simulation'))
    )

    # Simulation
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
    processor_node = Node(
        package='sim',
        executable='lidar_processor',
        name='lidar_processor',
        output='screen',
        parameters=[{
            'input_scan_topic': '/scan',
            'output_scan_topic': '/scan_filtered',
            'range_min': 0.15,
            'range_max': 12.0,
            'publish_cloud': False,
            'target_frame': 'laser',
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # AMCL – use filtered scan via remapping
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # ... (all your existing AMCL parameters) ...
            # Do NOT set 'scan_topic' here; let remapping handle it.
        }],
        remappings=[('/scan', '/scan_filtered')]   # <-- key line
    )

    # Nav2 bringup (includes controller, planner, recoveries, etc.)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': PathJoinSubstitution([
                FindPackageShare('sim'),
                'config',
                'nav2_params.yaml'
            ])
        }.items()
    )

    # Lifecycle manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['map_server', 'amcl', 'controller_server',
                           'planner_server', 'recoveries_server', 'bt_navigator']
        }]
    )

    '''
    # Application node
    app_node = Node(
        package='sim',
        executable='lidar_app_node',
        name='lidar_app_node',
        output='screen',
        parameters=[{
            'scan_topic': '/scan',                  # adjust if needed
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    '''

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
        map_yaml_file_arg,
        real_driver_launch,
        sim_launch,
        processor_node,          # <-- added
        map_server,
        amcl,
        nav2_bringup,
        lifecycle_manager,
        app_node,
        rviz_node,
    ])
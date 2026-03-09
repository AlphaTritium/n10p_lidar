from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition   # <-- import IfCondition
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

    # Real driver (official)
    real_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lslidar_driver'),
                'launch',
                'lslidar_launch.py'   # adjust filename if needed
            ])
        ]),
        condition=UnlessCondition(LaunchConfiguration('use_simulation'))
    )

    # Simulation (Gazebo)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sim'),
                'launch',
                'gazebo_sim.launch.py'   # or gazebo_lidar.launch.py
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('use_simulation')),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
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

    # AMCL
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'alpha1': 0.2,
            'alpha2': 0.2,
            'alpha3': 0.2,
            'alpha4': 0.2,
            'alpha5': 0.2,
            'base_frame_id': 'base_link',
            'beam_skip_distance': 0.5,
            'beam_skip_error_threshold': 0.9,
            'beam_skip_threshold': 0.3,
            'do_beamskip': False,
            'global_frame_id': 'map',
            'lambda_short': 0.1,
            'laser_likelihood_max_dist': 2.0,
            'laser_max_range': 100.0,
            'laser_min_range': -1.0,
            'laser_model_type': 'likelihood_field',
            'max_beams': 60,
            'max_particles': 2000,
            'min_particles': 500,
            'odom_frame_id': 'odom',
            'pf_err': 0.05,
            'pf_z': 0.99,
            'recovery_alpha_fast': 0.0,
            'recovery_alpha_slow': 0.0,
            'resample_interval': 1,
            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
            'save_pose_rate': 0.5,
            'sigma_hit': 0.2,
            'tf_broadcast': True,
            'transform_tolerance': 1.0,
            'update_min_a': 0.2,
            'update_min_d': 0.25,
            'z_hit': 0.5,
            'z_max': 0.05,
            'z_rand': 0.5,
            'z_short': 0.05,
            'scan_topic': '/scan'
        }]
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

    # Lifecycle manager to activate all nav2 nodes
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

    # Application node
    app_node = Node(
        package='sim',
        executable='lidar_app_node',
        name='lidar_app_node',
        output='screen',
        parameters=[{
            'scan_topic': '/scan',
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
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
        map_yaml_file_arg,
        real_driver_launch,
        sim_launch,
        map_server,
        amcl,
        nav2_bringup,
        lifecycle_manager,
        app_node,
        rviz_node,
    ])
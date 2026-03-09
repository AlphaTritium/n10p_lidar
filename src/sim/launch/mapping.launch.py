from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
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

    # Include real driver (official) or simulation
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lslidar_driver'),   # official driver package
                'launch',
                'lslidar_launch.py'                    # official single LiDAR launch
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('use_simulation')),
        # If the official launch accepts arguments, you can pass them here
        # launch_arguments={'frame_id': 'laser', ...}.items()
    )

    # Simulation launch (from our package)
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

    # Combine: use either real or sim, but we can't use both conditions in one Include?
    # Better: use two separate includes with opposite conditions.
    # We'll create a conditional group using OpaqueFunction, but for simplicity:
    # We'll include both but they are mutually exclusive due to condition.
    # So the launch will include either the real driver or the sim.

    # SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('sim'),
                'config',
                'slam_toolbox_params.yaml'
            ]),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[('/scan', '/scan')]
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
        # Include either real or sim based on use_simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('lslidar_driver'),
                    'launch',
                    'lslidar_launch.py'
                ])
            ]),
            condition=IfCondition(LaunchConfiguration('use_simulation'))  # false -> include real
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('sim'),
                    'launch',
                    'gazebo_sim.launch.py'
                ])
            ]),
            condition=IfCondition(LaunchConfiguration('use_simulation')),  # true -> include sim
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
        ),
        slam_node,
        app_node,
        rviz_node,
    ])
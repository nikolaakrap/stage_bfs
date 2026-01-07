from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # args
    map_yaml = LaunchConfiguration('map_yaml')
    world = LaunchConfiguration('world')
    base_frame = LaunchConfiguration('base_frame')
    scan_topic = LaunchConfiguration('scan_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_args = [
        DeclareLaunchArgument(
            'map_yaml',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'),
                'stage_bfs_ws', 'maps', 'stage_map.yaml'
            ])
        ),
        DeclareLaunchArgument('world', default_value='cave'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('scan_topic', default_value='/base_scan'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]

    # Stage launch
    stage_pkg = get_package_share_directory('stage_ros2')
    stage_launch = os.path.join(stage_pkg, 'launch', 'demo.launch.py')

    stage = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(stage_launch),
        launch_arguments={
            'world': world,
            'use_stamped_velocity': 'false',
        }.items()
    )

    # map_server (loads full static map immediately)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_yaml},
        ],
    )

    # lifecycle manager for map_server
    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server']},
        ],
    )

    # BFS + follower + recovery (unchanged)
    bfs_nav = Node(
        package='stage_bfs_navigation',
        executable='bfs_nav',
        name='bfs_nav',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'map_yaml': map_yaml},
            {'base_frame': base_frame},
            {'scan_topic': scan_topic},
            {'cmd_vel_topic': cmd_vel_topic},
            {'stop_dist': 0.45},
            {'block_ahead_cells': 10},
            {'block_width_cells': 3},
            {'recovery_backup_time': 1.0},
        ],

        
    )

    static_map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(
        declare_args + [
            stage,
            map_server,
            lifecycle,
            static_map_odom_tf,
            bfs_nav
        ]
    )

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetRemap
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    config_path = PathJoinSubstitution([FindPackageShare('41068_ignition_bringup'), 'config'])

    # Additional command line arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )

    # Start Simultaneous Localisation and Mapping (SLaM)
    slam = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('slam_toolbox'),
                             'launch', 'online_async_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': PathJoinSubstitution([config_path, 'slam_params.yaml'])
        }.items()
    )

    # Start Navigation Stack
    navigation = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([config_path, 'nav2_params.yaml']),
            "slam": 'True'
        }.items()
    )

    remapped_nav_stack = GroupAction([
        # Send TF to the global topics
        SetRemap(src='tf', dst='/tf'),
        SetRemap(src='tf_static', dst='/tf_static'),
        # Make sure laser resolves to the root scan topic
        SetRemap(src='scan', dst='/scan'),
        # (optional) map topic match (usually not needed, slam publishes /map already)
        # SetRemap(src='map', dst='/map'),

        slam,
        
        navigation,
    ])

    ld.add_action(use_sim_time_launch_arg)
    ld.add_action(remapped_nav_stack)

    return ld

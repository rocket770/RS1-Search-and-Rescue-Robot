from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    config_path = PathJoinSubstitution([FindPackageShare('41068_ignition_bringup'), 'config'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    # --- Cartographer 3D instead of slam_toolbox ---
    cartographer3d = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('41068_ignition_bringup'),
                              'launch', '41068_cartographer3d.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Nav2 stack (controllers/planners). Keep as-is.
    navigation = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('nav2_bringup'),
                              'launch', 'navigation_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([config_path, 'nav2_params.yaml'])
        }.items()
    )

    ld.add_action(use_sim_time_launch_arg)
    ld.add_action(cartographer3d)
    ld.add_action(navigation)
    return ld

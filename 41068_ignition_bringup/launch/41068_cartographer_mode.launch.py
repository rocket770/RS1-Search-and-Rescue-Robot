from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    mode = LaunchConfiguration('mode')  # '2d' or '3d'

    pkg = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg, 'config'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('mode', default_value='2d',
                              description='Choose "2d" (with Nav2) or "3d" (no Nav2)'),

        # 2D mapping + Nav2
        IncludeLaunchDescription(
            PathJoinSubstitution([pkg, 'launch', '41068_cartographer2d.launch.py']),
            condition=IfCondition(PythonExpression(["'", mode, "'", " == '2d'"])),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('nav2_bringup'),
                                  'launch', 'navigation_launch.py']),
            condition=IfCondition(PythonExpression(["'", mode, "'", " == '2d'"])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': PathJoinSubstitution([config_path, 'nav2_params.yaml'])
            }.items()
        ),

        # 3D mapping only (NO Nav2)
        IncludeLaunchDescription(
            PathJoinSubstitution([pkg, 'launch', '41068_cartographer3d.launch.py']),
            condition=IfCondition(PythonExpression(["'", mode, "'", " == '3d'"])),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
    ])

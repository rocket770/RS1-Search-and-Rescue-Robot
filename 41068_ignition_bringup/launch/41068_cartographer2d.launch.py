from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='True')

    cfg_dir = PathJoinSubstitution([FindPackageShare('41068_ignition_bringup'), 'config'])

    carto = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cfg_dir,
            '-configuration_basename', 'cartographer_2d.lua',
        ],
        remappings=[
            ('scan', '/scan'),           # 2D uses LaserScan
            ('odom', '/odometry'),
            ('imu',  '/imu'),
        ],
    )

    occ = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'resolution': 0.05}],
        remappings=[
            ('submap_list', '/submap_list'),
            ('map', '/map'),
            ('map_updates', '/map_updates'),
        ],
    )

    return LaunchDescription([use_sim_time_arg, carto, occ])

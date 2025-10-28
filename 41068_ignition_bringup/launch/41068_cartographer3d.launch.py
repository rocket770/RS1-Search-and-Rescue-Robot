from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use Gazebo clock'
    )

    cfg_dir = PathJoinSubstitution([
        FindPackageShare('41068_ignition_bringup'), 'config'
    ])

    carto = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', cfg_dir,
            '-configuration_basename', 'cartographer_3d.lua',
        ],
        remappings=[
            ('points2', '/camera/depth/points'),
            ('imu', '/imu'),
            ('odom', '/odometry'),
        ],
    )

    # 2D occupancy grid publisher (/map and /map_updates)
    occ = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'resolution': 0.05,         # OK to match your costmaps
            # 'publish_period_sec': 0.5  # optional
        }],
        remappings=[
            ('submap_list', '/submap_list'),
            ('map', '/map'),
            ('map_updates', '/map_updates'),
        ],
    )

    return LaunchDescription([use_sim_time, carto, occ])

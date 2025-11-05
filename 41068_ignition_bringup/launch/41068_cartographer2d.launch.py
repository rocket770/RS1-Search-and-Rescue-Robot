from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='True') #Expose use_sim_time.

    cfg_dir = PathJoinSubstitution([FindPackageShare('41068_ignition_bringup'), 'config']) #Config directory where Lua files live.

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
    )    #start cartographer ROS node; pass lua file path; remap i)nput topics to robots topics; publish map-odom/submap-list

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
    )        #Converts Cartographer submaps into a rolling 2D occupancy grid for RViz/Nav2 (/map, /map_updates), at 5 cm resolution.

    return LaunchDescription([use_sim_time_arg, carto, occ])

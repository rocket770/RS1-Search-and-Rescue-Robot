from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

 # i used this to make the bt config portable btw guys - nick
from nav2_common.launch import RewrittenYaml

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


    params_file = PathJoinSubstitution([FindPackageShare('41068_ignition_bringup'), 'config', 'nav2_params.yaml'])

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'default_bt_xml_filename': PathJoinSubstitution([
                FindPackageShare('41068_ignition_bringup'),
                'bt', 'custom_navigate_to_pose.xml' 
            ])
        },
        convert_types=True
    )

    navigation = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': configured_params
        }.items()
        
    )

    ld.add_action(use_sim_time_launch_arg)
    ld.add_action(slam)
    ld.add_action(navigation)

    return ld

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# from launch_ros.substitutions import FindPackageShare


# def generate_launch_description():

#     ld = LaunchDescription()

#     config_path = PathJoinSubstitution([FindPackageShare('41068_ignition_bringup'), 'config'])

#     # Additional command line arguments
#     use_sim_time = LaunchConfiguration('use_sim_time')
#     use_sim_time_launch_arg = DeclareLaunchArgument(
#         'use_sim_time',
#         default_value='True',
#         description='Flag to enable use_sim_time'
#     )

#     # Start Simultaneous Localisation and Mapping (SLaM)
#     slam = IncludeLaunchDescription(
#         PathJoinSubstitution([FindPackageShare('slam_toolbox'),
#                              'launch', 'online_async_launch.py']),
#         launch_arguments={
#             'use_sim_time': use_sim_time,
#             'slam_params_file': PathJoinSubstitution([config_path, 'slam_params.yaml'])
#         }.items()
#     )

#     # Start Navigation Stack
#     navigation = IncludeLaunchDescription(
#         PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']),
#         launch_arguments={
#             'use_sim_time': use_sim_time,
#             'params_file': PathJoinSubstitution([config_path, 'nav2_params.yaml'])
#         }.items()
#     )

#     ld.add_action(use_sim_time_launch_arg)
#     ld.add_action(slam)
#     ld.add_action(navigation)

#     return ld


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    config_path = PathJoinSubstitution([FindPackageShare('41068_ignition_bringup'), 'config'])

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )
    ld.add_action(use_sim_time_launch_arg)

    # BT XML file argument
    default_nav_to_pose_bt_xml = LaunchConfiguration('default_nav_to_pose_bt_xml')
    default_bt_xml_arg = DeclareLaunchArgument(
        'default_nav_to_pose_bt_xml',
        default_value='',
        description='Full path to the behavior tree xml file'
    )
    ld.add_action(default_bt_xml_arg)

    # Start SLAM
    slam = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('slam_toolbox'),
                             'launch', 'online_async_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': PathJoinSubstitution([config_path, 'slam_params.yaml'])
        }.items()
    )

    # Build navigation launch arguments
    nav2_launch_args = {
        'use_sim_time': use_sim_time,
        'params_file': PathJoinSubstitution([config_path, 'nav2_params.yaml'])
    }
    
    # Add BT XML if provided
    nav2_launch_args['default_nav_to_pose_bt_xml'] = default_nav_to_pose_bt_xml

    # Start Navigation Stack
    navigation = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']),
        launch_arguments=nav2_launch_args.items()
    )

    ld.add_action(slam)
    ld.add_action(navigation)

    return ld
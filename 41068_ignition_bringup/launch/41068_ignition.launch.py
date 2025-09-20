from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap


def generate_launch_description():
    ld = LaunchDescription()

    use_sim_time_launch_arg = DeclareLaunchArgument('use_sim_time', default_value='True')
    ld.add_action(use_sim_time_launch_arg)
    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(DeclareLaunchArgument('rviz', default_value='False'))
    ld.add_action(DeclareLaunchArgument('nav2', default_value='False'))  
    ld.add_action(DeclareLaunchArgument('yolo', default_value='false'))
    ld.add_action(DeclareLaunchArgument('goal_sender', default_value='false'))

    pkg_path = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path, 'config'])

    ld.add_action(DeclareLaunchArgument('world', default_value='simple_trees', choices=['simple_trees','large_demo']))
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py']),
        launch_arguments={
            'ign_args': [PathJoinSubstitution([pkg_path, 'worlds', [LaunchConfiguration('world'), '.sdf']]), ' -r']
        }.items()
    )
    ld.add_action(gazebo)


    robot_description_husky = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg_path, 'urdf', 'husky.urdf.xacro'])]),  
        value_type=str
    )

    husky_cfg = PathJoinSubstitution([config_path, 'husky'])
    husky_group = GroupAction([
        PushRosNamespace('husky'),

        # Make TF global for everything in this group
        # (if SetRemap isn't available in your env, skip these and keep per-node remaps)
        SetRemap(src='tf', dst='/tf'),
        SetRemap(src='tf_static', dst='/tf_static'),

        # robot_state_publisher: prefixed frames
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_husky,
                'use_sim_time': use_sim_time,
                'frame_prefix': 'husky/',
            }],
            remappings=[('tf','/tf'), ('tf_static','/tf_static')]
        ),

        # EKF (robot_localization): publishes odom->base_link
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf',
            output='screen',
            parameters=[PathJoinSubstitution([husky_cfg, 'robot_localization.yaml']),
                        {'use_sim_time': use_sim_time}],
            remappings=[('tf','/tf'), ('tf_static','/tf_static')]
        ),

        # ros_ign_bridge inside the namespace so we get /husky/scan etc.
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge.yaml']),
                'use_sim_time': use_sim_time
            }],
            remappings=[
                ('/scan', 'scan'),
                ('/scan/points', 'scan/points'),
                ('/camera/image', 'camera/image'),
                ('/camera/depth/image', 'camera/depth/image'),
                ('/camera/camera_info', 'camera/camera_info'),
                ('/odometry', 'odom'),
                ('/imu', 'imu'),
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='scan_frame_bridge',
            # x y z qx qy qz qw  parent             child
            arguments=['0','0','0','0','0','0','1','husky/base_scan','base_scan'],
        ),

        Node(
            package='tf2_ros', executable='static_transform_publisher', name='base_link_frame_bridge',
            arguments=['0','0','0','0','0','0','1','husky/base_link','base_link'],
        ),

            # odom (if /husky/odom message header.frame_id is 'odom')
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='odom_frame_bridge',
            arguments=['0','0','0','0','0','0','1','husky/odom','odom'],
        ),

            # imu (pick the IMU frame you actually see in /husky/imu, commonly 'imu_link')
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='imu_frame_bridge',
            arguments=['0','0','0','0','0','0','1','husky/imu_link','imu_link'],
        ),


        # Start SLAM TOOLBOX DIRECTLY so we can remap TF; DO NOT start AMCL.
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                PathJoinSubstitution([husky_cfg, 'slam_params.yaml'])
            ],
            remappings=[
                ('tf','/tf'), ('tf_static','/tf_static'),
                ('/map', 'map'),                 # <- force into namespace → /husky/map
                ('/map_metadata', 'map_metadata')
            ]
        )

    ])

    ld.add_action(husky_group)

    robot_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', '/husky/robot_description', '-name', 'husky', '-z', '0.4'], 
    )
    ld.add_action(robot_spawner)


    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path, '41068.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    model_path = PathJoinSubstitution([FindPackageShare('41068_ignition_bringup'), 'yolo', 'weights', 'best.pt'])
    yolo_node = Node(
        package='yolo_detector',
        executable='yolo_detector_node',
        name='yolo_detector',
        output='screen',
        parameters=[{
            'model_path': model_path,
            'rgb_topic': '/camera/image',
            'depth_topic': '/camera/depth/image',
            'info_topic': '/camera/camera_info',
            'target_frame': 'map',
            'use_sim_time': use_sim_time,
            'conf_thres': 0.65,
            'iou_thres': 0.65
        }],
        condition=IfCondition(LaunchConfiguration('yolo'))
    )
    ld.add_action(yolo_node)

    goal_sender_node = Node(
        package='nav2_goal_sender',          
        executable='nav2_goal_sender',
        name='goal_sender',
        output='screen',
        parameters=[{
            'robot_namespaces': ['husky'],   
            'goal_frame': 'map',
            'planner_id': '',
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(LaunchConfiguration('goal_sender'))
    )
    ld.add_action(goal_sender_node)


    # Nav2 enables mapping and waypoint following nav2 = IncludeLaunchDescription( PathJoinSubstitution([pkg_path, 'launch', '41068_navigation.launch.py']), launch_arguments={ 'use_sim_time': use_sim_time }.items(), condition=IfCondition(LaunchConfiguration('nav2')) ) ld.add_action(nav2)

    return ld

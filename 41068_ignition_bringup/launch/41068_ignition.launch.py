from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression,)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution


def generate_launch_description():

    ld = LaunchDescription()

    # Get paths to directories
    pkg_path = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path,
                                       'config'])

    # Additional command line arguments
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(use_sim_time_launch_arg)
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Flag to launch RViz'
    )
    ld.add_action(rviz_launch_arg)
    nav2_launch_arg = DeclareLaunchArgument(
        'nav2',
        default_value='True',
        description='Flag to launch Nav2'
    )
    ld.add_action(nav2_launch_arg)
    yolo_arg = DeclareLaunchArgument(
        'yolo',
        default_value='false',  # 'true'/'false' as strings
        description='Flag YOLO detector node'
    )
    ld.add_action(yolo_arg)
    battery_arg = DeclareLaunchArgument( 
        'battery', 
        default_value='true', 
        description='Launch battery simulator' 
    ) 
    ld.add_action(battery_arg)
    slam_launch_arg = DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Flag to enable SLAM and exploration'
    )
    ld.add_action(slam_launch_arg)

    # Load robot_description and start robot_state_publisher
    robot_description_content = ParameterValue(
        Command(['xacro ',
                 PathJoinSubstitution([pkg_path,
                                       'urdf',
                                       'husky.urdf.xacro'])]),
        value_type=str)
    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                          'use_sim_time': use_sim_time
                                      }])
    ld.add_action(robot_state_publisher_node)

    # Publish odom -> base_link transform **using robot_localization**
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[PathJoinSubstitution([config_path,
                                          'robot_localization.yaml']),
                    {'use_sim_time': use_sim_time}]
    )
    ld.add_action(robot_localization_node)

    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='large',
        description='Which world to load',
        choices=['simple_trees', 'large', 'extra_large']
    )
    ld.add_action(world_launch_arg)

    # Build "<world>.sdf" as a quoted PythonExpression
    world_filename = PythonExpression([
        "'", LaunchConfiguration('world'), "'", " + '.sdf'"
    ])

    world_file = PathJoinSubstitution([
        pkg_path,
        'worlds',
        world_filename
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_ign_gazebo'),
                'launch',
                'ign_gazebo.launch.py'
            ])
        ),
        launch_arguments={
            # Quote the expanded path inside the expression, then append ' -r'
            'ign_args': PythonExpression([
                "'", world_file, "'", " + ' -r'"
            ])
        }.items()
    )
    ld.add_action(gazebo)
    # Spawn robot in Gazebo
    robot_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', '/robot_description', '-z', '0.4']
    )
    ld.add_action(robot_spawner)

    # Bridge topics between gazebo and ROS2
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution([config_path,
                                                          'gazebo_bridge.yaml']),
                    'use_sim_time': use_sim_time}]
    )
    ld.add_action(gazebo_bridge)

    # rviz2 visualises data
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path,
                                               '41068.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    # Nav2 enables mapping and waypoint following
    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_path,
                              'launch',
                              '41068_navigation.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(nav2)
    
    # --- YOLO detector node ---
        
    model_path = PathJoinSubstitution([
       FindPackageShare('41068_ignition_bringup'), 'yolo', 'weights', 'best_stable_best_result.pt'
    ])

    world = LaunchConfiguration('world')

    ign_pose_topic = PathJoinSubstitution([
        TextSubstitution(text='/world'),
        world,
        TextSubstitution(text='pose/info'),
    ])

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
            'conf_thres': 0.60,
            'iou_thres': 0.60,
            'ign_topic': ign_pose_topic,
        }],
        condition=IfCondition(LaunchConfiguration('yolo'))
    )
    ld.add_action(yolo_node)

    battery_sim_node = Node(
        package='battery_node', 
        executable='battery_node', 
        name='battery_node', 
        output='screen', 
        parameters=[{
            'use_sim_time': use_sim_time 
        }], 
        condition=IfCondition(LaunchConfiguration('battery')) 
    ) 
    
    ld.add_action(battery_sim_node) 

    home_base_location = [0.0, 0.0]

    bt_coord = Node(
        package="bt_coordinator",
        executable="coordinator",
        name="bt_coordinator",
        parameters=[{
            "bt_goal_topic": "goal_pose",
            "map_frame": "map",
            "approach_distance": 2.5,
            "battery_threshold": 0.30,
            "home_pose_xy": home_base_location,
            "post_manual_resume_suppress_secs": 2,
            "detection_memory": 4
        }],
    )
    ld.add_action(bt_coord) 

    path_planner = Node(
        package='bt_coordinator',
        executable='global_path_planner',
        name='global_path_planner',
        output='screen',
        parameters=[{
            'home_pose_xy': home_base_location,
            'frame_id': 'map',
            'goal_topic': '/static_path/goal',
            'planner_action': '/compute_path_to_pose'
        }]
    )
    ld.add_action(path_planner) 

    tools_time = Node(
        package='custom_tools',
        executable='toggle_time_node',
        name='toggle_time_node',
        output='screen',
        parameters=[{
            'world_name': LaunchConfiguration('world'),
            'sun_name': 'sun',
        }]
    )
    ld.add_action(tools_time) 

    tools_nv = Node(
        package='custom_tools',
        executable='night_vision_camera_node',
        name='night_vision_camera_node',
        output='screen',
        parameters=[{
            'in_image': '/camera/image',
            'out_image': '/night_vision/image',
            'publish_hz': 1.0
        }]
    )
    ld.add_action(tools_nv)

    tools_cloud = Node(
        package='custom_tools',
        executable='cloud_accumulator',
        name='cloud_accumulator',
        output='screen',
        parameters=[{
            'input_topic': '/camera/depth/points',
            'output_topic': '/accumulated_cloud',
            'target_frame': 'map',
            'publish_rate': 0.5,
            'stride': 4,
            'tf_cache_sec': 0.2,
        }]
    )
    ld.add_action(tools_cloud)

    explore_lite = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('explore_lite'),
                'launch',
                'explore.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(LaunchConfiguration('slam'))
    )
    ld.add_action(explore_lite)

    ui = Node(
        package='usergui',          
        executable='usergui',     
        name='usergui',
        output='screen',
    )
    ld.add_action(ui)


    return ld

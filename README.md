
# 41068_Ignition_Bringup – Cartographer_2d_3d_Integrated

Package provides:
- Ignition Gazebo simulation for Husky in Large Demo Environment
- Robot localization (EKF → odom → base_link)
- Cartographer 2D mapping with Nav2 (low fidelity iteration)
- Cartographer 3D mapping (not Nav2 compatible; (high fidelity iteration)
- /map and /map_updates for RViz and map saving

------------------------------------------------------------
DEPENDENCIES
------------------------------------------------------------

Required ROS 2 packages (Humble):

```bash
sudo apt install \
  ros-humble-cartographer-ros \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-ros-ign-gazebo \
  ros-humble-ros-ign-bridge \
  ros-humble-xacro \
  ros-humble-rviz2
```

(install missing packages)

------------------------------------------------------------
1. BUILD AND SOURCE THE WORKSPACE
------------------------------------------------------------

```bash
cd ~/41068_ws
colcon build --symlink-install
source install/setup.bash
```

------------------------------------------------------------
2. FIRST LAUNCH AND SELECT CARTOGRAPHER MODE (2D OR 3D)
------------------------------------------------------------
Two modes:

2d  → LIDAR LaserScan, supports Nav2 global planner

3d  → Depth point cloud, mapping only until more robust 3D implementation (Nav2 cannot use 3D map)

Launch 2D mapping:
```bash
ros2 launch 41068_ignition_bringup 41068_cartographer_mode.launch.py mode:=2d
```

Launch 3D mapping:
```bash
ros2 launch 41068_ignition_bringup 41068_cartographer_mode.launch.py mode:=3d
```



------------------------------------------------------------
3. LAUNCH SIMULATION (HUSKY + IGNITION) IN LARGE DEMO FOREST
------------------------------------------------------------


This launches:
- Gazebo world
- Robot State Publisher
- Robot Localization (EKF)
- ROS-Ignition topic bridges
- RViz
- Nav2 in 2D LIDAR Mapping Cartographer Mode

Example:
```bash
ros2 launch 41068_ignition_bringup 41068_ignition.launch.py rviz:=True nav2:=False world:=large_demo
```

Available worlds:
```text
world:=simple_trees
world:=large_demo
```

------------------------------------------------------------
4. MAPPING
------------------------------------------------------------

Use keyboard keys to cover desired mapping area (while cartographer is active):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Save recent mapping:
```bash
ros2 run nav2_map_server map_saver_cli -- -f ~/my_map
```

Preview recently made map:
```bash
ros2 run nav2_map_server map_saver_cli -- --help
eog ~/my_map.pgm
```

------------------------------------------------------------

------------------------------------------------------------
5. STAND-ALONE MAPPING NODES
------------------------------------------------------------

2D only:
```bash
ros2 launch 41068_ignition_bringup 41068_cartographer2d.launch.py
```

3D only:
```bash
ros2 launch 41068_ignition_bringup 41068_cartographer3d.launch.py
```

------------------------------------------------------------

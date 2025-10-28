# 41068_Ignition_Bringup – Cartographer_2d_3d_Integrated

Package provides:
- Ignition Gazebo simulation for Husky in Large Demo Environment
- Cartographer 2D mapping with Nav2 (low fidelity iteration)
- Cartographer 3D mapping (not Nav2 compatible; (high fidelity iteration)
- /map and /map_updates for RViz and map saving

------------------------------------------------------------
Dependencies
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
1. BUILD/SOURCE WORKSPACE
------------------------------------------------------------

```bash
cd ~/41068_ws
colcon build --symlink-install
source install/setup.bash
```

------------------------------------------------------------
2. FIRST LAUNCH/SELECT CARTOGRAPHER MODE I.E. 2D/3D
------------------------------------------------------------
Two modes:

2d  → LIDAR LaserScan, supports Nav2 global planner i.e. may integrate Nav2 using this mode

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
2. LAUNCH SIMULATION I.E. LARGE DEMO FOREST/HUSKY/IGNITION
------------------------------------------------------------


This launches:
- Gazebo world
- RViz

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
4. MAPPING ONLY NODES
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

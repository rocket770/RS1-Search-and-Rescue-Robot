# RS1 Search and Rescue Robot

This repository provides the setup instructions and launch configurations for the **RS1 Search and Rescue Robot** project. The robot utilizes ROS2 (Humble), Gazebo, and Ignition Fortress for simulation.

---

## Installation Guide

### 1. Install PIP
```bash
sudo apt install python3-pip
```

---

### 2. Dependencies
*(Assumes ROS2 Humble, Gazebo, and Ignition Fortress are already installed)*

#### YOLO (CPU)
> Although GPU is supported in the code, testing has been done **only on CPU**.

Remove the too-new version of `numpy`:
```bash
python3 -m pip uninstall -y numpy
```

Reinstall a compatible version (< 2.0) for `cv_bridge`:
```bash
python3 -m pip install "numpy<2.0" --user --upgrade
```

Install YOLO:
```bash
pip3 install ultralytics
```

Install OpenCV (safe version):
```bash
python3 -m pip install "opencv-python<=4.8.1.78" --user
```

Install PyTorch (CPU version):
```bash
python3 -m pip install --no-cache-dir --index-url https://download.pytorch.org/whl/cpu torch torchvision torchaudio
```

---

### 3. User Interface Dependencies
```bash
sudo apt install -y qtbase5-dev qtbase5-dev-tools qtchooser qt5-qmake
sudo apt install ros-hubmle-ros-image-to-qimage
```

---

## Running the Simulation

### 1. Build the Workspace
```bash
cd RS1-SEARCH-AND-RESCUE-ROBOT  # or your repo name
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch the Simulation

#### Regular World
```bash
ros2 launch 41068_ignition_bringup 41068_ignition.launch.py \
  slam:=true \
  nav2:=true \
  rviz:=true \
  yolo:=true \
  world:=large
```

#### Extra Large World  
> Not recommended for use with active nodes due to lag.
```bash
ros2 launch 41068_ignition_bringup 41068_ignition.launch.py world:=extra_large
```

---

## Common Errors

If you encounter issues with models not loading properly, run:
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix 41068_ignition_bringup)/share/41068_ignition_bringup
```
This usually isn't required, as the package prefix is handled automatically in:
```
41068_ignition_bringup/CMakeLists.txt
```

---

##  Launch Parameters

| Parameter | Options | Purpose | Default Value |
|------------|----------|----------|------------|
| **world** | `large`, `extra_large` | <ul><li>**large**: Contains all animals, optimized for performance.</li><li>**extra_large**: Full environment for SLAM showcase (may cause lag).</li></ul> | N/A but required. |
| **nav2** | `true`, `false` | Enables path planning and behavior trees. Required for SLAM, RViz, and autonomous movement. | True |
| **rviz** | `true`, `false` | Enables the RViz visualization interface. Requires `nav2 := true`. | False |
| **yolo** | `true`, `false` | Enables animal detection and monitoring. May cause lag on older CPUs. | False |
| **slam** | `true`, `false` | Enables autonomous SLAM navigation. Requires `nav2 := true`. | True |
| **battery** | `true`, `false` | Enables battery drain and charging simulation. | True |

---

## Notes
- Ensure your CPU supports the versions of PyTorch and OpenCV installed.
- For better performance, disable YOLO or reduce world size.
- If using GPU, update PyTorch installation with CUDA support from the official website.

---

**Author:** RS1 Group 32  
**ROS Distro:** Humble  
**Simulation:** Ignition Fortress -> Gazebo Bridged  
**License:** Opensource

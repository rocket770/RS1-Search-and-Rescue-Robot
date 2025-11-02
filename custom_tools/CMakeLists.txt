cmake_minimum_required(VERSION 3.5)
project(custom_tools)

# Python part
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# C++ deps
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# === C++ executable ===
add_executable(cloud_accumulator custom_tools/cloud_accumulator.cpp)  # <- adjust path
ament_target_dependencies(cloud_accumulator
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  std_msgs
)

install(TARGETS
  cloud_accumulator
  DESTINATION lib/${PROJECT_NAME}
)

# === Python install (your current stuff) ===
ament_python_install_package(${PROJECT_NAME})

# if you have console_scripts in setup.py this will install them
install(PROGRAMS
  # nothing here if you use entry_points
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()

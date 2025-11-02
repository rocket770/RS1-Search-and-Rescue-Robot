cmake_minimum_required(VERSION 3.5)
project(custom_tools)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# C++ deps
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(cloud_accumulator custom_tools/cloud_accumulator.cpp) 
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

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  custom_tools/toggle_time_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME toggle_time_node
)

install(PROGRAMS
  custom_tools/night_vision_camera_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME night_vision_camera_node
)

ament_package()

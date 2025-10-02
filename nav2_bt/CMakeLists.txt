cmake_minimum_required(VERSION 3.8)
project(nav2_bt)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_library(nav2_bt_plugins SHARED
  src/random_goal.cpp)
ament_target_dependencies(nav2_bt_plugins
  rclcpp behaviortree_cpp_v3 nav2_behavior_tree pluginlib
  geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

pluginlib_export_plugin_description_file(nav2_behavior_tree nav2_bt_plugin.xml)

install(TARGETS nav2_bt_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY bt_xml/ DESTINATION share/${PROJECT_NAME}/bt_xml)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()

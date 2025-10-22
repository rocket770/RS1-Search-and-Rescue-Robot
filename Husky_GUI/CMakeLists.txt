cmake_minimum_required(VERSION 3.5)
project(Husky_GUI)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(husky_control
  main.cpp
  husky_control.cpp
  husky_control.ui
)

ament_target_dependencies(husky_control
  rclcpp geometry_msgs nav_msgs sensor_msgs cv_bridge image_transport
)

target_link_libraries(husky_control
  Qt5::Widgets
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  husky_control
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  husky_control.ui
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.5)
project(pioneer_overlay)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rviz_2d_overlay_msgs REQUIRED)

add_executable(pioneer_overlay src/pioneer_overlay.cpp)
ament_target_dependencies(pioneer_overlay rclcpp std_msgs rviz_2d_overlay_msgs)

install(TARGETS
  pioneer_overlay
  DESTINATION lib/${PROJECT_NAME})

ament_package()

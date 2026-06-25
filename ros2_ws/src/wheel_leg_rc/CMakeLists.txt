cmake_minimum_required(VERSION 3.8)
project(wheel_leg_rc)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wheel_leg_msgs REQUIRED)

add_executable(rc_ibus_node
  src/rc_ibus_node.cpp
)
ament_target_dependencies(rc_ibus_node
  geometry_msgs
  rclcpp
  std_msgs
  wheel_leg_msgs
)

install(
  TARGETS rc_ibus_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()

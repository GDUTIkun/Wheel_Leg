cmake_minimum_required(VERSION 3.8)
project(wheel_leg_rc)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(rc_serial_probe_node
  src/rc_serial_probe_node.cpp
)
ament_target_dependencies(rc_serial_probe_node
  rclcpp
  std_msgs
)

install(
  TARGETS rc_serial_probe_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()

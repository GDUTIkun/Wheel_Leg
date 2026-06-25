cmake_minimum_required(VERSION 3.8)
project(wheel_leg_stm32_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wheel_leg_hw REQUIRED)
find_package(wheel_leg_msgs REQUIRED)

add_executable(wheel_leg_stm32_bridge_node
  src/stm32_bridge_node.cpp
)
target_include_directories(wheel_leg_stm32_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(wheel_leg_stm32_bridge_node
  rclcpp
  wheel_leg_hw
  wheel_leg_msgs
)

add_executable(stm32_uart_stress_node
  src/stm32_uart_stress_node.cpp
)
ament_target_dependencies(stm32_uart_stress_node
  rclcpp
  std_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS wheel_leg_stm32_bridge_node stm32_uart_stress_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp std_msgs wheel_leg_hw wheel_leg_msgs)
ament_export_include_directories(include)
ament_package()

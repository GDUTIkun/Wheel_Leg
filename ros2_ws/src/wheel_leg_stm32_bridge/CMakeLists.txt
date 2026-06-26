cmake_minimum_required(VERSION 3.8)
project(wheel_leg_stm32_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(wheel_leg_bridge REQUIRED)
find_package(wheel_leg_hw REQUIRED)
find_package(wheel_leg_msgs REQUIRED)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

add_library(${PROJECT_NAME}_state_assembler
  src/hardware_state_assembler.cpp
)
target_include_directories(${PROJECT_NAME}_state_assembler PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(wheel_leg_stm32_bridge_node
  src/stm32_bridge_node.cpp
)
target_include_directories(wheel_leg_stm32_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(wheel_leg_stm32_bridge_node
  ${PROJECT_NAME}_state_assembler)
ament_target_dependencies(wheel_leg_stm32_bridge_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  wheel_leg_bridge
  wheel_leg_hw
  wheel_leg_msgs
)

add_executable(joint_command_probe_node
  src/joint_command_probe_node.cpp
)
ament_target_dependencies(joint_command_probe_node
  rclcpp
  wheel_leg_hw
  wheel_leg_msgs
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(
    DIRECTORY include/
    DESTINATION include
  )
endif()

install(
  TARGETS
    ${PROJECT_NAME}_state_assembler
    wheel_leg_stm32_bridge_node
    joint_command_probe_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  ament_add_gtest(test_hardware_state_assembler
    test/test_hardware_state_assembler.cpp
  )
  target_link_libraries(test_hardware_state_assembler
    ${PROJECT_NAME}_state_assembler)
endif()

ament_export_dependencies(rclcpp sensor_msgs std_msgs std_srvs wheel_leg_bridge wheel_leg_hw wheel_leg_msgs)
ament_export_include_directories(include)
ament_package()

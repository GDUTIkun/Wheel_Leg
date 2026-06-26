cmake_minimum_required(VERSION 3.8)
project(wheel_leg_control)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wheel_leg_bridge REQUIRED)
find_package(wheel_leg_common REQUIRED)
find_package(wheel_leg_msgs REQUIRED)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

add_library(${PROJECT_NAME}
  src/controller_orchestrator.cpp
  src/legacy_algorithms.cpp
  src/ros_control_state_estimator.cpp
  src/state_assembler.cpp
  src/state_message_conversions.cpp
  src/stand_control_pipeline.cpp
  src/stand_control_runtime.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  std_msgs
  wheel_leg_common
  wheel_leg_msgs
)

add_executable(wheel_leg_controller_node
  src/controller_node.cpp
)
target_link_libraries(wheel_leg_controller_node ${PROJECT_NAME})
ament_target_dependencies(wheel_leg_controller_node
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  wheel_leg_bridge
  wheel_leg_common
  wheel_leg_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME} wheel_leg_controller_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  wheel_leg_bridge
  wheel_leg_common
  wheel_leg_msgs
)
ament_export_include_directories(include)

if(BUILD_TESTING)
  ament_add_gtest(test_stand_control_pipeline
    test/test_stand_control_pipeline.cpp
  )
  target_link_libraries(test_stand_control_pipeline ${PROJECT_NAME})
  ament_target_dependencies(test_stand_control_pipeline
    wheel_leg_common
  )
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(wheel_leg_sim)

find_package(ament_cmake REQUIRED)
find_package(wheel_leg_common REQUIRED)
find_package(wheel_leg_control REQUIRED)

add_library(${PROJECT_NAME}
  src/control_state_bridge.cpp
  src/base_state_assembly.cpp
  src/attitude_utils.cpp
  src/command_application.cpp
  src/leg_kinematics.cpp
  src/leg_state_assembly.cpp
  src/mapping_utils.cpp
  src/robot_state_assembly.cpp
  src/state_builders.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}
  wheel_leg_control::wheel_leg_control
)
ament_target_dependencies(${PROJECT_NAME}
  wheel_leg_common
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(wheel_leg_common wheel_leg_control)
ament_export_include_directories(include)
ament_package()

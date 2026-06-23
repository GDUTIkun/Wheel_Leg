#ifndef WHEEL_LEG_SIM__SENSOR_TYPES_HPP_
#define WHEEL_LEG_SIM__SENSOR_TYPES_HPP_

namespace wheel_leg_sim {

struct BaseLinkState {
  double distance = 0.0;
  double velocity = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  double roll_rate = 0.0;
  double pitch_rate = 0.0;
  double yaw_rate = 0.0;
  double accel_x = 0.0;
  double accel_y = 0.0;
  double accel_z = 0.0;
};

struct JointState {
  double position = 0.0;
  double velocity = 0.0;
};

struct LegKinematics {
  double hip_absolute = 0.0;
  double knee_absolute = 0.0;
  double calf_absolute = 0.0;
  double leg_length = 0.0;
  double phi = 0.0;
  double phi_rate = 0.0;
};

struct LegState {
  JointState hip;
  JointState knee;
  JointState wheel;
  JointState calf;
  LegKinematics kinematics;
};

struct RobotSensorData {
  BaseLinkState base_link;
  LegState left_leg;
  LegState right_leg;
};

struct SensorAssemblyState {
  bool has_previous_left_phi = false;
  bool has_previous_right_phi = false;
  double previous_left_phi = 0.0;
  double previous_right_phi = 0.0;
  double filtered_left_phi_rate = 0.0;
  double filtered_right_phi_rate = 0.0;
  double base_forward_distance = 0.0;
};

}  // namespace wheel_leg_sim

#endif  // WHEEL_LEG_SIM__SENSOR_TYPES_HPP_

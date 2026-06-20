#ifndef WHEEL_LEG_SIMULATE_TOOLS_SENSOR_H_
#define WHEEL_LEG_SIMULATE_TOOLS_SENSOR_H_

#include <array>
#include <mujoco/mujoco.h>

namespace wheel_leg {

struct BaseLinkState {
  // Distance and velocity along the base frame x axis. Distance is integrated
  // from the projected forward velocity so it can later be replaced by EKF.
  double distance;
  double velocity;
  double roll;
  double pitch;
  double yaw;
  double roll_rate;
  double pitch_rate;
  double yaw_rate;
  double accel_x;
  double accel_y;
  double accel_z;
};

struct JointState {
  double position;
  double velocity;
};

struct LegKinematics {
  double hip_absolute;
  double knee_absolute;
  double calf_absolute;
  double leg_length;
  double phi;
  double phi_rate;
};

struct LegState {
  // Hip/knee/calf positions are absolute leg-plane angles after applying the
  // calibration offsets. Wheel position remains the wheel encoder angle.
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

RobotSensorData AssembleSensorData(const mjModel* m, const mjData* d);

void PrintSensors(const RobotSensorData& sensor_data);

void PrintSensors(const mjModel* m, const mjData* d);

double ReadScalarSensor(const mjModel* m, const mjData* d,
                        const char* sensor_name);

std::array<double, 4> ReadQuaternionSensor(const mjModel* m, const mjData* d,
                                           const char* sensor_name);

std::array<double, 3> ReadVectorSensor(const mjModel* m, const mjData* d,
                                       const char* sensor_name);

}  // namespace wheel_leg

#endif  // WHEEL_LEG_SIMULATE_TOOLS_SENSOR_H_

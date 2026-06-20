#include "sensor.h"
#include "math_utils.h"

#include <array>
#include <cmath>
#include <iostream>

namespace wheel_leg {
namespace {

constexpr double kLeftHipOffsetDeg = 143.944;
constexpr double kRightHipOffsetDeg = 145.56;
constexpr double kLeftKneeOffsetDeg = 26.04;
constexpr double kRightKneeOffsetDeg = 33.843;
constexpr double kPhiRateLowPassAlpha = 0.95;

struct EulerAngles {
  double roll;
  double pitch;
  double yaw;
};

JointState ReadJointState(const mjModel* m, const mjData* d,
                          const char* pos_sensor_name,
                          const char* vel_sensor_name) {
  JointState state;
  state.position = ReadScalarSensor(m, d, pos_sensor_name);
  state.velocity = ReadScalarSensor(m, d, vel_sensor_name);
  return state;
}

double NormalizeAngleDelta(double angle_delta) {
  while (angle_delta > kPi) {
    angle_delta -= 2.0 * kPi;
  }
  while (angle_delta < -kPi) {
    angle_delta += 2.0 * kPi;
  }
  return angle_delta;
}

void UpdatePhiRate(LegKinematics* kinematics, double* previous_phi,
                   double* filtered_phi_rate, bool* has_previous_phi,
                   double dt) {
  double raw_phi_rate = 0.0;
  if (dt <= 0.0 || !*has_previous_phi) {
    raw_phi_rate = 0.0;
    *filtered_phi_rate = 0.0;
  } else {
    raw_phi_rate = NormalizeAngleDelta(kinematics->phi - *previous_phi) / dt;
    *filtered_phi_rate =
        kPhiRateLowPassAlpha * *filtered_phi_rate +
        (1.0 - kPhiRateLowPassAlpha) * raw_phi_rate;
  }

  kinematics->phi_rate = *filtered_phi_rate;
  *previous_phi = kinematics->phi;
  *has_previous_phi = true;
}

double ComputeBaseForwardVelocity(const mjData* d, double yaw) {
  const double forward_x = std::cos(yaw);
  const double forward_y = std::sin(yaw);
  return d->qvel[0] * forward_x + d->qvel[1] * forward_y;
}

EulerAngles QuaternionToEuler(const std::array<double, 4>& quat) {
  // MuJoCo framequat sensors return quaternions in w, x, y, z order.
  const double w = quat[0];
  const double x = quat[1];
  const double y = quat[2];
  const double z = quat[3];

  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);

  double sinp = 2.0 * (w * y - z * x);
  if (sinp > 1.0) {
    sinp = 1.0;
  } else if (sinp < -1.0) {
    sinp = -1.0;
  }

  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);

  EulerAngles euler;
  euler.roll = std::atan2(sinr_cosp, cosr_cosp);
  euler.pitch = std::asin(sinp);
  euler.yaw = std::atan2(siny_cosp, cosy_cosp);
  return euler;
}

LegKinematics ComputeLegKinematics(double hip_absolute,
                                   double knee_absolute,
                                   double calf_absolute) {
  constexpr double kL1 = 0.18;
  constexpr double kL2 = 0.225;

  const double theta_l2 = hip_absolute + calf_absolute - kPi;
  const double x = kL1 * std::cos(hip_absolute) + kL2 * std::cos(theta_l2);
  const double y_clockwise =
      kL1 * std::sin(hip_absolute) + kL2 * std::sin(theta_l2);

  LegKinematics output;
  output.hip_absolute = hip_absolute;
  output.knee_absolute = knee_absolute;
  output.calf_absolute = calf_absolute;
  output.leg_length = std::sqrt(x * x + y_clockwise * y_clockwise);
  output.phi = std::atan2(y_clockwise, x);
  output.phi_rate = 0.0;
  return output;
}

LegState AssembleLeftLegState(const mjModel* m, const mjData* d) {
  LegState leg;
  leg.hip = ReadJointState(m, d, "left_hip_encoder", "left_hip_encoder_vel");
  leg.knee = ReadJointState(m, d, "left_knee_encoder", "left_knee_encoder_vel");
  leg.wheel =
      ReadJointState(m, d, "left_wheel_encoder", "left_wheel_encoder_vel");
  leg.calf = ReadJointState(m, d, "left_calf_encoder", "left_calf_encoder_vel");

  const JointState raw_hip = leg.hip;
  const JointState raw_knee = leg.knee;

  const double hip_absolute =
      raw_hip.position + DegreesToRadians(kLeftHipOffsetDeg);
  const double knee_absolute =
      raw_hip.position + raw_knee.position +
      DegreesToRadians(kLeftKneeOffsetDeg);
  const double calf_absolute =
      kPi - DegreesToRadians(kLeftHipOffsetDeg) + raw_knee.position +
      DegreesToRadians(kLeftKneeOffsetDeg);

  leg.hip.position = hip_absolute;
  leg.hip.velocity = raw_hip.velocity;
  leg.knee.position = knee_absolute;
  leg.knee.velocity = raw_hip.velocity + raw_knee.velocity;
  leg.calf.position = calf_absolute;
  leg.calf.velocity = raw_knee.velocity;
  leg.kinematics =
      ComputeLegKinematics(hip_absolute, knee_absolute, calf_absolute);
  return leg;
}

LegState AssembleRightLegState(const mjModel* m, const mjData* d) {
  LegState leg;
  leg.hip = ReadJointState(m, d, "right_hip_encoder", "right_hip_encoder_vel");
  leg.knee =
      ReadJointState(m, d, "right_knee_encoder", "right_knee_encoder_vel");
  leg.wheel =
      ReadJointState(m, d, "right_wheel_encoder", "right_wheel_encoder_vel");
  leg.calf =
      ReadJointState(m, d, "right_calf_encoder", "right_calf_encoder_vel");

  const JointState raw_hip = leg.hip;
  const JointState raw_knee = leg.knee;

  const double hip_absolute =
      raw_hip.position + DegreesToRadians(kRightHipOffsetDeg);
  const double knee_absolute =
      raw_hip.position + raw_knee.position +
      DegreesToRadians(kRightKneeOffsetDeg);
  const double calf_absolute =
      kPi - DegreesToRadians(kRightHipOffsetDeg) + raw_knee.position +
      DegreesToRadians(kRightKneeOffsetDeg);

  leg.hip.position = hip_absolute;
  leg.hip.velocity = raw_hip.velocity;
  leg.knee.position = knee_absolute;
  leg.knee.velocity = raw_hip.velocity + raw_knee.velocity;
  leg.calf.position = calf_absolute;
  leg.calf.velocity = raw_knee.velocity;
  leg.kinematics =
      ComputeLegKinematics(hip_absolute, knee_absolute, calf_absolute);
  return leg;
}

}  // namespace

double ReadScalarSensor(const mjModel* m, const mjData* d,
                        const char* sensor_name) {
  int id = mj_name2id(m, mjOBJ_SENSOR, sensor_name);
  if (id < 0) {
    std::cout << " [missing:" << sensor_name << "]";
    return 0.0;
  }
  return d->sensordata[m->sensor_adr[id]];
}

std::array<double, 4> ReadQuaternionSensor(const mjModel* m, const mjData* d,
                                           const char* sensor_name) {
  std::array<double, 4> values = {0.0, 0.0, 0.0, 1.0};
  const int id = mj_name2id(m, mjOBJ_SENSOR, sensor_name);
  if (id < 0) {
    std::cout << " [missing:" << sensor_name << "]";
    return values;
  }

  const int adr = m->sensor_adr[id];
  const int dim = m->sensor_dim[id];
  for (int i = 0; i < dim && i < static_cast<int>(values.size()); ++i) {
    values[i] = d->sensordata[adr + i];
  }
  return values;
}

std::array<double, 3> ReadVectorSensor(const mjModel* m, const mjData* d,
                                       const char* sensor_name) {
  std::array<double, 3> values = {0.0, 0.0, 0.0};
  const int id = mj_name2id(m, mjOBJ_SENSOR, sensor_name);
  if (id < 0) {
    std::cout << " [missing:" << sensor_name << "]";
    return values;
  }

  const int adr = m->sensor_adr[id];
  const int dim = m->sensor_dim[id];
  for (int i = 0; i < dim && i < static_cast<int>(values.size()); ++i) {
    values[i] = d->sensordata[adr + i];
  }
  return values;
}

RobotSensorData AssembleSensorData(
    const mjModel* m, const mjData* d, SensorAssemblyState* state) {
  RobotSensorData sensor_data = {};
  if (!m || !d || !state) {
    return sensor_data;
  }

  const std::array<double, 4> base_quat = ReadQuaternionSensor(m, d, "base_quat");
  const std::array<double, 3> base_gyro = ReadVectorSensor(m, d, "base_gyro");
  const std::array<double, 3> base_accel = ReadVectorSensor(m, d, "base_accel");
  const EulerAngles base_euler = QuaternionToEuler(base_quat);

  const double dt = m->opt.timestep;
  const double base_forward_velocity =
      m->nv > 1 ? ComputeBaseForwardVelocity(d, base_euler.yaw) : 0.0;
  if (dt > 0.0) {
    state->base_forward_distance += base_forward_velocity * dt;
  }

  sensor_data.base_link.distance = state->base_forward_distance;
  sensor_data.base_link.velocity = base_forward_velocity;
  sensor_data.base_link.roll = base_euler.roll;
  sensor_data.base_link.pitch = base_euler.pitch;
  sensor_data.base_link.yaw = base_euler.yaw;
  sensor_data.base_link.roll_rate = base_gyro[0];
  sensor_data.base_link.pitch_rate = base_gyro[1];
  sensor_data.base_link.yaw_rate = base_gyro[2];
  sensor_data.base_link.accel_x = base_accel[0];
  sensor_data.base_link.accel_y = base_accel[1];
  sensor_data.base_link.accel_z = base_accel[2];

  sensor_data.left_leg = AssembleLeftLegState(m, d);
  sensor_data.right_leg = AssembleRightLegState(m, d);
  UpdatePhiRate(&sensor_data.left_leg.kinematics, &state->previous_left_phi,
                &state->filtered_left_phi_rate, &state->has_previous_left_phi,
                dt);
  UpdatePhiRate(&sensor_data.right_leg.kinematics, &state->previous_right_phi,
                &state->filtered_right_phi_rate, &state->has_previous_right_phi,
                dt);
  return sensor_data;
}

RobotSensorData AssembleSensorData(const mjModel* m, const mjData* d) {
  static SensorAssemblyState state;
  return AssembleSensorData(m, d, &state);
}

void PrintSensors(const RobotSensorData& sensor_data) {
  std::cout << "[base]"
            << " x=(" << sensor_data.base_link.distance << ", "
            << sensor_data.base_link.velocity << ")"
            << " rpy_deg=("
            << RadiansToDegrees(sensor_data.base_link.roll) << ", "
            << RadiansToDegrees(sensor_data.base_link.pitch) << ", "
            << RadiansToDegrees(sensor_data.base_link.yaw) << ")"
            << " gyro=("
            << RadiansToDegrees(sensor_data.base_link.roll_rate) << ", "
            << RadiansToDegrees(sensor_data.base_link.pitch_rate) << ", "
            << RadiansToDegrees(sensor_data.base_link.yaw_rate) << ")";

  std::cout << " [left]"
            << " hip=(" << RadiansToDegrees(sensor_data.left_leg.hip.position)
            << ", " << RadiansToDegrees(sensor_data.left_leg.hip.velocity) << ")"
            << " knee=(" << RadiansToDegrees(sensor_data.left_leg.knee.position)
            << ", " << RadiansToDegrees(sensor_data.left_leg.knee.velocity) << ")"
            << " wheel=(" << RadiansToDegrees(sensor_data.left_leg.wheel.position)
            << ", " << RadiansToDegrees(sensor_data.left_leg.wheel.velocity) << ")"
            << " phi=" << RadiansToDegrees(sensor_data.left_leg.kinematics.phi)
            << " phi_rate="
            << RadiansToDegrees(sensor_data.left_leg.kinematics.phi_rate)
            << " L0=" << sensor_data.left_leg.kinematics.leg_length;

  std::cout << " [right]"
            << " hip=(" << RadiansToDegrees(sensor_data.right_leg.hip.position)
            << ", " << RadiansToDegrees(sensor_data.right_leg.hip.velocity) << ")"
            << " knee=(" << RadiansToDegrees(sensor_data.right_leg.knee.position)
            << ", " << RadiansToDegrees(sensor_data.right_leg.knee.velocity) << ")"
            << " wheel=(" << RadiansToDegrees(sensor_data.right_leg.wheel.position)
            << ", " << RadiansToDegrees(sensor_data.right_leg.wheel.velocity) << ")"
            << " phi=" << RadiansToDegrees(sensor_data.right_leg.kinematics.phi)
            << " phi_rate="
            << RadiansToDegrees(sensor_data.right_leg.kinematics.phi_rate)
            << " L0=" << sensor_data.right_leg.kinematics.leg_length;
  std::cout << std::endl;
}

void PrintSensors(const mjModel* m, const mjData* d) {
  PrintSensors(AssembleSensorData(m, d));
}

}  // namespace wheel_leg

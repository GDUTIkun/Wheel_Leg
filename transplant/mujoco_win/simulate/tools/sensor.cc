#include "sensor.h"

#include <array>
#include <iostream>

#include <wheel_leg_sim/attitude_utils.hpp>
#include <wheel_leg_sim/base_state_assembly.hpp>
#include <wheel_leg_sim/leg_kinematics.hpp>
#include <wheel_leg_sim/leg_state_assembly.hpp>
#include <wheel_leg_sim/robot_state_assembly.hpp>

namespace wheel_leg {
namespace {

SensorAssemblyState g_sensor_assembly_state;

JointState ReadJointState(const mjModel* m, const mjData* d,
                          const char* pos_sensor_name,
                          const char* vel_sensor_name) {
  JointState state;
  state.position = ReadScalarSensor(m, d, pos_sensor_name);
  state.velocity = ReadScalarSensor(m, d, vel_sensor_name);
  return state;
}

LegState AssembleLeftLegState(const mjModel* m, const mjData* d) {
  return wheel_leg_sim::AssembleLeftLegState(
      ReadJointState(m, d, "left_hip_encoder", "left_hip_encoder_vel"),
      ReadJointState(m, d, "left_knee_encoder", "left_knee_encoder_vel"),
      ReadJointState(m, d, "left_wheel_encoder", "left_wheel_encoder_vel"),
      ReadJointState(m, d, "left_calf_encoder", "left_calf_encoder_vel"));
}

LegState AssembleRightLegState(const mjModel* m, const mjData* d) {
  return wheel_leg_sim::AssembleRightLegState(
      ReadJointState(m, d, "right_hip_encoder", "right_hip_encoder_vel"),
      ReadJointState(m, d, "right_knee_encoder", "right_knee_encoder_vel"),
      ReadJointState(m, d, "right_wheel_encoder", "right_wheel_encoder_vel"),
      ReadJointState(m, d, "right_calf_encoder", "right_calf_encoder_vel"));
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
  const wheel_leg_sim::RawLegInputs left_leg_inputs = {
      .hip = ReadJointState(m, d, "left_hip_encoder", "left_hip_encoder_vel"),
      .knee = ReadJointState(m, d, "left_knee_encoder", "left_knee_encoder_vel"),
      .wheel = ReadJointState(m, d, "left_wheel_encoder", "left_wheel_encoder_vel"),
      .calf = ReadJointState(m, d, "left_calf_encoder", "left_calf_encoder_vel"),
  };
  const wheel_leg_sim::RawLegInputs right_leg_inputs = {
      .hip = ReadJointState(m, d, "right_hip_encoder", "right_hip_encoder_vel"),
      .knee = ReadJointState(m, d, "right_knee_encoder", "right_knee_encoder_vel"),
      .wheel = ReadJointState(m, d, "right_wheel_encoder", "right_wheel_encoder_vel"),
      .calf = ReadJointState(m, d, "right_calf_encoder", "right_calf_encoder_vel"),
  };
  return wheel_leg_sim::AssembleRobotSensorData(
      base_quat,
      base_gyro,
      base_accel,
      m->nv > 0 ? d->qvel[0] : 0.0,
      m->nv > 1 ? d->qvel[1] : 0.0,
      m->opt.timestep,
      left_leg_inputs,
      right_leg_inputs,
      state);
}

RobotSensorData AssembleSensorData(const mjModel* m, const mjData* d) {
  return AssembleSensorData(m, d, &g_sensor_assembly_state);
}

void ResetSensorAssemblyState() {
  g_sensor_assembly_state = SensorAssemblyState();
}

}  // namespace wheel_leg

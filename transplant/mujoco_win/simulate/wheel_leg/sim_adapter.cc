#include "sim_adapter.h"

#include <algorithm>
#include <array>
#include <cmath>

#include "sensor.h"
#include <wheel_leg_sim/joint_mappings.hpp>

namespace wheel_leg {
namespace {

SensorAssemblyState g_sensor_assembly_state;

wheel_leg_common::TimePoint ToCommonTime(double sim_time) {
  wheel_leg_common::TimePoint stamp;
  const double clamped_time = std::max(0.0, sim_time);
  stamp.sec = static_cast<std::int32_t>(std::floor(clamped_time));
  stamp.nanosec = static_cast<std::uint32_t>(
      (clamped_time - static_cast<double>(stamp.sec)) * 1000000000.0);
  return stamp;
}

const wheel_leg_sim::JointMapping* FindMapping(const std::string& ros_name) {
  for (const auto& mapping : wheel_leg_sim::kJointMappings) {
    if (ros_name == mapping.ros_name) {
      return &mapping;
    }
  }
  return nullptr;
}

}  // namespace

void ResetSimAdapterState() {
  g_sensor_assembly_state = SensorAssemblyState();
}

RobotSensorData SampleRobotSensorData(const mjModel* m, const mjData* d) {
  return AssembleSensorData(m, d, &g_sensor_assembly_state);
}

wheel_leg_common::JointStateSample SampleJointState(
    const mjModel* m, const mjData* d) {
  wheel_leg_common::JointStateSample sample;
  if (!m || !d) {
    return sample;
  }

  sample.stamp = ToCommonTime(d->time);
  sample.joints.reserve(wheel_leg_sim::kJointMappings.size());

  for (const auto& mapping : wheel_leg_sim::kJointMappings) {
    const int joint_id = mj_name2id(m, mjOBJ_JOINT, mapping.mujoco_joint);
    if (joint_id < 0) {
      continue;
    }

    wheel_leg_common::JointSample joint;
    joint.name = mapping.ros_name;
    joint.position = d->qpos[m->jnt_qposadr[joint_id]];
    joint.velocity = d->qvel[m->jnt_dofadr[joint_id]];
    sample.joints.push_back(joint);
  }

  return sample;
}

wheel_leg_common::ImuSample SampleImu(const mjModel* m, const mjData* d) {
  wheel_leg_common::ImuSample sample;
  if (!m || !d) {
    return sample;
  }

  const std::array<double, 4> quat = ReadQuaternionSensor(m, d, "base_quat");
  const std::array<double, 3> gyro = ReadVectorSensor(m, d, "base_gyro");
  const std::array<double, 3> accel = ReadVectorSensor(m, d, "base_accel");

  sample.stamp = ToCommonTime(d->time);
  sample.frame_id = "base_link";
  sample.orientation.x = quat[1];
  sample.orientation.y = quat[2];
  sample.orientation.z = quat[3];
  sample.orientation.w = quat[0];
  sample.angular_velocity.x = gyro[0];
  sample.angular_velocity.y = gyro[1];
  sample.angular_velocity.z = gyro[2];
  sample.linear_acceleration.x = accel[0];
  sample.linear_acceleration.y = accel[1];
  sample.linear_acceleration.z = accel[2];
  return sample;
}

ApplyCommandResult ApplyControlCommand(
    const mjModel* m, mjData* d,
    const wheel_leg_common::ControlCommand& command) {
  ApplyCommandResult result;
  if (!m || !d) {
    return result;
  }

  for (const auto& joint_effort : command.joint_efforts) {
    const wheel_leg_sim::JointMapping* mapping =
        FindMapping(joint_effort.joint_name);
    if (!mapping) {
      result.rejected_joint_name = joint_effort.joint_name;
      return result;
    }

    const int actuator_id =
        mj_name2id(m, mjOBJ_ACTUATOR, mapping->mujoco_actuator);
    if (actuator_id < 0) {
      result.rejected_joint_name = joint_effort.joint_name;
      return result;
    }

    double effort = joint_effort.effort;
    if (m->actuator_ctrllimited[actuator_id]) {
      const double min_ctrl = m->actuator_ctrlrange[2 * actuator_id];
      const double max_ctrl = m->actuator_ctrlrange[2 * actuator_id + 1];
      const double requested_effort = effort;
      effort = std::clamp(effort, min_ctrl, max_ctrl);
      if (effort != requested_effort) {
        if (!result.command_was_clamped) {
          result.first_clamped_joint_name = joint_effort.joint_name;
          result.first_requested_effort = requested_effort;
          result.first_applied_effort = effort;
        }
        result.command_was_clamped = true;
      }
    }

    d->ctrl[actuator_id] = effort;
    ++result.applied_effort_count;
  }

  result.accepted = true;
  return result;
}
}  // namespace wheel_leg

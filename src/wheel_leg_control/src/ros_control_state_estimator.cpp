#include "wheel_leg_control/ros_control_state_estimator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <string_view>

namespace wheel_leg_control {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kL1 = 0.18;
constexpr double kL2 = 0.225;
constexpr double kWheelRadius = 0.05;
constexpr double kLeftHipOffsetDeg = 143.944;
constexpr double kRightHipOffsetDeg = 145.56;
constexpr double kLeftKneeOffsetDeg = 26.04;
constexpr double kRightKneeOffsetDeg = 33.843;
constexpr double kPhiRateLowPassAlpha = 0.95;

struct LocalJointState {
  double position = 0.0;
  double velocity = 0.0;
};

struct LocalLegKinematics {
  double hip_absolute = 0.0;
  double calf_absolute = 0.0;
  double leg_length = 0.0;
  double phi = 0.0;
  double phi_rate = 0.0;
};

std::optional<wheel_leg_common::JointSample> FindJoint(
    const wheel_leg_common::JointStateSample& joint_state,
    std::string_view joint_name) {
  for (const auto& joint : joint_state.joints) {
    if (joint.name == joint_name) {
      return joint;
    }
  }
  return std::nullopt;
}

double DegreesToRadians(double degrees) {
  return degrees * kPi / 180.0;
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

LocalLegKinematics ComputeLegKinematics(double hip_absolute,
                                        double calf_absolute) {
  const double theta_l2 = hip_absolute + calf_absolute - kPi;
  const double x = kL1 * std::cos(hip_absolute) + kL2 * std::cos(theta_l2);
  const double y_clockwise =
      kL1 * std::sin(hip_absolute) + kL2 * std::sin(theta_l2);

  LocalLegKinematics output;
  output.hip_absolute = hip_absolute;
  output.calf_absolute = calf_absolute;
  output.leg_length = std::sqrt(x * x + y_clockwise * y_clockwise);
  output.phi = std::atan2(y_clockwise, x);
  return output;
}

LocalLegKinematics AssembleLegKinematics(const wheel_leg_common::JointSample& raw_hip,
                                         const wheel_leg_common::JointSample& raw_knee,
                                         double hip_offset_deg,
                                         double knee_offset_deg) {
  const double hip_offset = DegreesToRadians(hip_offset_deg);
  const double knee_offset = DegreesToRadians(knee_offset_deg);
  const double hip_absolute = raw_hip.position + hip_offset;
  const double calf_absolute = kPi - hip_offset + raw_knee.position + knee_offset;
  return ComputeLegKinematics(hip_absolute, calf_absolute);
}

void UpdatePhiRate(double phi,
                   double* previous_phi,
                   double* filtered_phi_rate,
                   bool* has_previous_phi,
                   double dt,
                   double* output_phi_rate) {
  if (!previous_phi || !filtered_phi_rate || !has_previous_phi ||
      !output_phi_rate) {
    return;
  }

  if (dt <= 0.0 || !*has_previous_phi) {
    *filtered_phi_rate = 0.0;
  } else {
    const double raw_phi_rate = NormalizeAngleDelta(phi - *previous_phi) / dt;
    *filtered_phi_rate =
        kPhiRateLowPassAlpha * *filtered_phi_rate +
        (1.0 - kPhiRateLowPassAlpha) * raw_phi_rate;
  }

  *output_phi_rate = *filtered_phi_rate;
  *previous_phi = phi;
  *has_previous_phi = true;
}

std::array<double, 4> ToWxyz(const wheel_leg_common::Quaternion& quat) {
  return {quat.w, quat.x, quat.y, quat.z};
}

std::array<double, 3> QuaternionToEuler(const std::array<double, 4>& quat) {
  const double w = quat[0];
  const double x = quat[1];
  const double y = quat[2];
  const double z = quat[3];

  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  double sinp = 2.0 * (w * y - z * x);
  sinp = std::clamp(sinp, -1.0, 1.0);
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);

  return {
      std::atan2(sinr_cosp, cosr_cosp),
      std::asin(sinp),
      std::atan2(siny_cosp, cosy_cosp),
  };
}

}  // namespace

std::optional<StandControlState> RosControlStateEstimator::Estimate(
    const wheel_leg_common::RobotStateSnapshot& snapshot) {
  if (!snapshot.has_joint_state || !snapshot.has_imu) {
    return std::nullopt;
  }

  const auto left_hip = FindJoint(snapshot.joint_state, "left_hip");
  const auto left_knee = FindJoint(snapshot.joint_state, "left_knee");
  const auto left_wheel = FindJoint(snapshot.joint_state, "left_wheel");
  const auto right_hip = FindJoint(snapshot.joint_state, "right_hip");
  const auto right_knee = FindJoint(snapshot.joint_state, "right_knee");
  const auto right_wheel = FindJoint(snapshot.joint_state, "right_wheel");
  if (!left_hip || !left_knee || !left_wheel || !right_hip || !right_knee ||
      !right_wheel) {
    return std::nullopt;
  }

  const double current_time =
      wheel_leg_common::ToSeconds(snapshot.joint_state.stamp);
  const double dt = has_last_stamp_
                        ? current_time - wheel_leg_common::ToSeconds(last_stamp_)
                        : 0.0;
  last_stamp_ = snapshot.joint_state.stamp;
  has_last_stamp_ = true;

  const auto euler = QuaternionToEuler(ToWxyz(snapshot.imu.orientation));
  const double base_forward_velocity =
      0.5 * (left_wheel->velocity + right_wheel->velocity) * kWheelRadius;
  if (dt > 0.0) {
    base_forward_distance_ += base_forward_velocity * dt;
  }

  StandControlState control_state;
  control_state.body.distance = base_forward_distance_;
  control_state.body.velocity = base_forward_velocity;
  control_state.body.pitch = euler[1];
  control_state.body.pitch_rate = snapshot.imu.angular_velocity.y;
  control_state.body.yaw_rate = snapshot.imu.angular_velocity.z;

  const auto left_leg =
      AssembleLegKinematics(*left_hip, *left_knee,
                            kLeftHipOffsetDeg, kLeftKneeOffsetDeg);
  control_state.left_leg.hip_absolute = left_leg.hip_absolute;
  control_state.left_leg.calf_absolute = left_leg.calf_absolute;
  control_state.left_leg.leg_length = left_leg.leg_length;
  control_state.left_leg.phi = left_leg.phi;
  UpdatePhiRate(left_leg.phi, &previous_left_phi_, &filtered_left_phi_rate_,
                &has_previous_left_phi_, dt,
                &control_state.left_leg.phi_rate);

  const auto right_leg =
      AssembleLegKinematics(*right_hip, *right_knee,
                            kRightHipOffsetDeg, kRightKneeOffsetDeg);
  control_state.right_leg.hip_absolute = right_leg.hip_absolute;
  control_state.right_leg.calf_absolute = right_leg.calf_absolute;
  control_state.right_leg.leg_length = right_leg.leg_length;
  control_state.right_leg.phi = right_leg.phi;
  UpdatePhiRate(right_leg.phi, &previous_right_phi_, &filtered_right_phi_rate_,
                &has_previous_right_phi_, dt,
                &control_state.right_leg.phi_rate);

  return control_state;
}

}  // namespace wheel_leg_control

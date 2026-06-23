#include "wheel_leg_control/state_message_conversions.hpp"

#include <std_msgs/msg/header.hpp>

namespace wheel_leg_control {
namespace {

std_msgs::msg::Header ToRosHeader(const wheel_leg_common::TimePoint& stamp) {
  std_msgs::msg::Header header;
  header.stamp.sec = stamp.sec;
  header.stamp.nanosec = stamp.nanosec;
  return header;
}

}  // namespace

StandControlState FromRosStandControlState(
    const wheel_leg_msgs::msg::StandControlState& msg) {
  StandControlState state;
  state.body.distance = msg.body_distance;
  state.body.velocity = msg.body_velocity;
  state.body.roll = msg.body_roll;
  state.body.roll_rate = msg.body_roll_rate;
  state.body.pitch = msg.body_pitch;
  state.body.pitch_rate = msg.body_pitch_rate;
  state.body.yaw_rate = msg.body_yaw_rate;
  state.left_leg.hip_absolute = msg.left_hip_absolute;
  state.left_leg.calf_absolute = msg.left_calf_absolute;
  state.left_leg.leg_length = msg.left_leg_length;
  state.left_leg.phi = msg.left_phi;
  state.left_leg.phi_rate = msg.left_phi_rate;
  state.right_leg.hip_absolute = msg.right_hip_absolute;
  state.right_leg.calf_absolute = msg.right_calf_absolute;
  state.right_leg.leg_length = msg.right_leg_length;
  state.right_leg.phi = msg.right_phi;
  state.right_leg.phi_rate = msg.right_phi_rate;
  return state;
}

wheel_leg_msgs::msg::StandControlState ToRosStandControlState(
    const StandControlState& state,
    const wheel_leg_common::TimePoint& stamp) {
  wheel_leg_msgs::msg::StandControlState msg;
  msg.header = ToRosHeader(stamp);
  msg.body_distance = state.body.distance;
  msg.body_velocity = state.body.velocity;
  msg.body_roll = state.body.roll;
  msg.body_roll_rate = state.body.roll_rate;
  msg.body_pitch = state.body.pitch;
  msg.body_pitch_rate = state.body.pitch_rate;
  msg.body_yaw_rate = state.body.yaw_rate;
  msg.left_hip_absolute = state.left_leg.hip_absolute;
  msg.left_calf_absolute = state.left_leg.calf_absolute;
  msg.left_leg_length = state.left_leg.leg_length;
  msg.left_phi = state.left_leg.phi;
  msg.left_phi_rate = state.left_leg.phi_rate;
  msg.right_hip_absolute = state.right_leg.hip_absolute;
  msg.right_calf_absolute = state.right_leg.calf_absolute;
  msg.right_leg_length = state.right_leg.leg_length;
  msg.right_phi = state.right_leg.phi;
  msg.right_phi_rate = state.right_leg.phi_rate;
  return msg;
}

}  // namespace wheel_leg_control

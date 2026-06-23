#ifndef WHEEL_LEG_CONTROL__ROS_CONTROL_STATE_ESTIMATOR_HPP_
#define WHEEL_LEG_CONTROL__ROS_CONTROL_STATE_ESTIMATOR_HPP_

#include <optional>

#include <wheel_leg_common/types.hpp>
#include <wheel_leg_control/stand_control_types.hpp>

namespace wheel_leg_control {

class RosControlStateEstimator {
 public:
  std::optional<StandControlState> Estimate(
      const wheel_leg_common::RobotStateSnapshot& snapshot);

 private:
  bool has_previous_left_phi_ = false;
  bool has_previous_right_phi_ = false;
  double previous_left_phi_ = 0.0;
  double previous_right_phi_ = 0.0;
  double filtered_left_phi_rate_ = 0.0;
  double filtered_right_phi_rate_ = 0.0;
  double base_forward_distance_ = 0.0;
  wheel_leg_common::TimePoint last_stamp_;
  bool has_last_stamp_ = false;
};

}  // namespace wheel_leg_control

#endif  // WHEEL_LEG_CONTROL__ROS_CONTROL_STATE_ESTIMATOR_HPP_

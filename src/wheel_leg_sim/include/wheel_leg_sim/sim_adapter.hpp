#ifndef WHEEL_LEG_SIM__SIM_ADAPTER_HPP_
#define WHEEL_LEG_SIM__SIM_ADAPTER_HPP_

#include <optional>

#include <wheel_leg_common/types.hpp>

namespace wheel_leg_sim {

class SimAdapter {
 public:
  virtual ~SimAdapter() = default;

  virtual std::optional<wheel_leg_common::JointStateSample> SampleJointState()
      const = 0;
  virtual std::optional<wheel_leg_common::ImuSample> SampleImu() const = 0;
  virtual void ApplyCommand(
      const wheel_leg_common::ControlCommand& command) = 0;
  virtual double StepTimeSeconds() const = 0;
};

}  // namespace wheel_leg_sim

#endif  // WHEEL_LEG_SIM__SIM_ADAPTER_HPP_

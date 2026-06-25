#ifndef WHEEL_LEG_CONTROL__STAND_ALGORITHM_INTERFACES_HPP_
#define WHEEL_LEG_CONTROL__STAND_ALGORITHM_INTERFACES_HPP_

#include <array>

namespace wheel_leg_control {

struct PidStepInput {
  double measurement = 0.0;
  double target = 0.0;
  double dt = 0.01;
};

class PidAlgorithm {
 public:
  virtual ~PidAlgorithm() = default;
  virtual double Compute(const PidStepInput& input) = 0;
};

using LqrStateVector = std::array<double, 6>;

struct LqrStepInput {
  double leg_length = 0.0;
  LqrStateVector target = {};
  LqrStateVector state = {};
};

struct LqrControlOutput {
  double wheel_torque = 0.0;
  double hip_torque = 0.0;
  double torque_magnitude = 0.0;
  bool fly_flag = false;
};

class LqrAlgorithm {
 public:
  virtual ~LqrAlgorithm() = default;
  virtual LqrControlOutput Compute(const LqrStepInput& input) const = 0;
};

struct VmcStepInput {
  double force = 0.0;
  double torque = 0.0;
  double leg_length = 0.0;
  double phi = 0.0;
  double hip_absolute = 0.0;
  double calf_absolute = 0.0;
};

struct VmcJointTorques {
  double hip_torque = 0.0;
  double knee_torque = 0.0;
};

class VmcAlgorithm {
 public:
  virtual ~VmcAlgorithm() = default;
  virtual VmcJointTorques Compute(const VmcStepInput& input) const = 0;
};

}  // namespace wheel_leg_control

#endif  // WHEEL_LEG_CONTROL__STAND_ALGORITHM_INTERFACES_HPP_

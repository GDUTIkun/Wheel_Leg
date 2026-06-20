#ifndef WHEEL_LEG_SIMULATE_TOOLS_VMC_H_
#define WHEEL_LEG_SIMULATE_TOOLS_VMC_H_

namespace wheel_leg {

struct VmcOutput {
  double joint1_torque;
  double joint2_torque;
};

// C++ version of simulate/matlab_function/VMC.m.
// Output order matches the MATLAB vector:
// [joint1_torque, joint2_torque].
VmcOutput SerialVMC(double force, double torque, double leg_length,
                    double phi, double theta1, double theta2);

}  // namespace wheel_leg

#endif  // WHEEL_LEG_SIMULATE_TOOLS_VMC_H_

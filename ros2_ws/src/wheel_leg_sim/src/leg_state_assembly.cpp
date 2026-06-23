#include "wheel_leg_sim/leg_state_assembly.hpp"

#include "wheel_leg_sim/leg_kinematics.hpp"

namespace wheel_leg_sim {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kLeftHipOffsetDeg = 143.944;
constexpr double kRightHipOffsetDeg = 145.56;
constexpr double kLeftKneeOffsetDeg = 26.04;
constexpr double kRightKneeOffsetDeg = 33.843;

double DegreesToRadians(double degrees) {
  return degrees * kPi / 180.0;
}

LegState AssembleLegState(const JointState& raw_hip,
                          const JointState& raw_knee,
                          const JointState& raw_wheel,
                          const JointState& raw_calf,
                          double hip_offset_deg,
                          double knee_offset_deg) {
  const double hip_offset = DegreesToRadians(hip_offset_deg);
  const double knee_offset = DegreesToRadians(knee_offset_deg);

  const double hip_absolute = raw_hip.position + hip_offset;
  const double knee_absolute =
      raw_hip.position + raw_knee.position + knee_offset;
  const double calf_absolute = kPi - hip_offset + raw_knee.position + knee_offset;

  LegState leg;
  leg.hip = raw_hip;
  leg.knee = raw_knee;
  leg.wheel = raw_wheel;
  leg.calf = raw_calf;

  leg.hip.position = hip_absolute;
  leg.knee.position = knee_absolute;
  leg.knee.velocity = raw_hip.velocity + raw_knee.velocity;
  leg.calf.position = calf_absolute;
  leg.kinematics =
      ComputeLegKinematics(hip_absolute, knee_absolute, calf_absolute);
  return leg;
}

}  // namespace

LegState AssembleLeftLegState(const JointState& raw_hip,
                              const JointState& raw_knee,
                              const JointState& raw_wheel,
                              const JointState& raw_calf) {
  return AssembleLegState(raw_hip, raw_knee, raw_wheel, raw_calf,
                          kLeftHipOffsetDeg, kLeftKneeOffsetDeg);
}

LegState AssembleRightLegState(const JointState& raw_hip,
                               const JointState& raw_knee,
                               const JointState& raw_wheel,
                               const JointState& raw_calf) {
  return AssembleLegState(raw_hip, raw_knee, raw_wheel, raw_calf,
                          kRightHipOffsetDeg, kRightKneeOffsetDeg);
}

}  // namespace wheel_leg_sim

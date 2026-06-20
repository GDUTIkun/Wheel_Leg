#include "wheel_leg_hooks.h"
#include "simulate.h"
#include "sensor.h"
#include "plotter.h"
#include "vmc.h"
#include "math_utils.h"
#include "pid.h"
#include "actuator.h"
#include "lqr_k.h"
#include "ros2_bridge.h"

#include <cmath>
#include <iomanip>
#include <iostream>

namespace wheel_leg {
namespace {

constexpr double kDefaultTargetPhiDeg = 90.0;
constexpr double kDefaultTargetLegLength = 0.25;
constexpr double kSwervingSpeedFeedforwardCoef = 3;
constexpr double kLegLengthGravityCompMass = 3.99077;
constexpr double kGravityAcceleration = 9.81;

mujoco::Simulate* g_sim = nullptr;

PIDInstance leglen_pid_l;
PIDInstance phi_pid_l;
PIDInstance leglen_pid_r;
PIDInstance phi_pid_r;
PIDInstance steer_v_pid;
PIDInstance anti_crash_pid;
bool pid_initialized = false;

struct ControlTargets {
  double target_velocity = 0.0;
  double target_yaw_rate = 0.0;
  double target_distance = 0.0;
  double target_leg_length = kDefaultTargetLegLength;
  double target_phi = DegreesToRadians(kDefaultTargetPhiDeg);
};

ControlTargets control_targets;

PID_Init_Config_s leg_length_pid_conf = {
    .Kp = 1300,
    .Ki = 200,
    .Kd = 30,
    .MaxOut = 5000,
    .DeadBand = 0.0001f,
    .Improve = static_cast<PID_Improvement_e>(
        PID_ChangingIntegrationRate | PID_DerivativeFilter | PID_Derivative_On_Measurement),
    .IntegralLimit = 5000,
    .CoefA = 0.05,
    .CoefB = 0.1,
    .Output_LPF_RC = 0.0f,
    .Derivative_LPF_RC = 0.01,
};

PID_Init_Config_s steer_v_pid_conf = {
    .Kp = 6.0f,
    .Ki = 0.8f,
    .Kd = 0.0f,
    .MaxOut = 50,
    .DeadBand = 0.001f,
    .Improve = static_cast<PID_Improvement_e>(
        PID_DerivativeFilter | PID_Integral_Limit),
    .IntegralLimit = 50,
    .Output_LPF_RC = 0.0f,
    .Derivative_LPF_RC = 0.01,
};

PID_Init_Config_s anti_crash_pid_conf = {
    .Kp = 3,
    .Ki = 0.1f,
    .Kd = 0.0f,
    .MaxOut = 10,
    .DeadBand = 0.001f,
    .Improve = static_cast<PID_Improvement_e>(
        PID_DerivativeFilter | PID_ChangingIntegrationRate |
        PID_Integral_Limit),
    .IntegralLimit = 10,
    .CoefA = 0.05,
    .CoefB = 0.1,
    .Output_LPF_RC = 0.0f,
    .Derivative_LPF_RC = 0.01,
};

LqrVector BuildLqrStates(const LegState& leg,
                         const RobotSensorData& sensor_data) {
  return {{
      leg.kinematics.phi,
      leg.kinematics.phi_rate,
      sensor_data.base_link.distance,
      sensor_data.base_link.velocity,
      sensor_data.base_link.pitch,
      sensor_data.base_link.pitch_rate,
  }};
}

LqrVector BuildLqrTarget(const ControlTargets& targets) {
  return {{
      targets.target_phi,
      0.0,
      targets.target_distance,
      targets.target_velocity,
      0.0,
      0.0,
  }};
}

void PrintLqrVector(const char* name, const LqrVector& values) {
  std::cout << name
            << " phi=" << RadiansToDegrees(values[0]) << "deg"
            << ", phi_rate=" << RadiansToDegrees(values[1]) << "deg/s"
            << ", distance=" << values[2] << "m"
            << ", velocity=" << values[3] << "m/s"
            << ", pitch=" << RadiansToDegrees(values[4]) << "deg"
            << ", pitch_rate=" << RadiansToDegrees(values[5]) << "deg/s";
}

void PrintMatlabVector(const char* name, const LqrVector& values) {
  std::cout << "  " << name << " = ["
            << values[0] << "; "
            << values[1] << "; "
            << values[2] << "; "
            << values[3] << "; "
            << values[4] << "; "
            << values[5] << "];";
}

}  // namespace

void SetSimulateHandle(mujoco::Simulate* sim) {
  g_sim = sim;
  SetPlotSimulateHandle(sim);
}

void OnModelLoaded(mjModel* m, mjData* d) {
  std::cout << "wheel_leg simulate ready: nq=" << m->nq << ", nv=" << m->nv
            << ", nu=" << m->nu << std::endl;

  // Let the official simulate UI sliders start from zero torque.
  for (int i = 0; i < m->nu; ++i) {
    d->ctrl[i] = 0.0;
  }

  //Init
  PIDInit(&leglen_pid_l, &leg_length_pid_conf);
  PIDInit(&leglen_pid_r, &leg_length_pid_conf);
  PIDInit(&steer_v_pid, &steer_v_pid_conf);
  PIDInit(&anti_crash_pid, &anti_crash_pid_conf);
  pid_initialized = true;
  control_targets = ControlTargets();
  ResetPlots();

  std::cout << "Open the right-side Control panel in simulate UI and drag the"
            << " actuator sliders to change torque." << std::endl;
  std::cout << "Fixed stand target: phi=90deg, distance=0m, velocity=0m/s, "
            << "yaw-rate=0deg/s." << std::endl;
  InitializeRos2Bridge(m, d);
}

void BeforeStep(mjModel* m, mjData* d) {
  static int step_count = 0;
  SpinRos2Bridge(m, d);
  // Leave d->ctrl untouched here so the built-in simulate UI sliders
  // can directly control actuator torque values.
  if (!pid_initialized) {
    return;
  }
  const RobotSensorData sensor_data = AssembleSensorData(m, d);
  const LegKinematics& right_leg = sensor_data.right_leg.kinematics;
  const LegKinematics& left_leg = sensor_data.left_leg.kinematics;
  const double target_phi = control_targets.target_phi;
  const double target_leg_length = control_targets.target_leg_length;

  float u_phi_r = PIDCalculate(&phi_pid_r, right_leg.phi, target_phi);
  float u_leg_length_r = PIDCalculate(&leglen_pid_r, right_leg.leg_length,
                                     target_leg_length);
  float u_phi_l = PIDCalculate(&phi_pid_l, left_leg.phi, target_phi);
  float u_leg_length_l = PIDCalculate(&leglen_pid_l, left_leg.leg_length,
                                     target_leg_length); 
  const double leg_length_gravity_compensation =
      kLegLengthGravityCompMass / 2.0 * kGravityAcceleration *
      std::cos(sensor_data.base_link.pitch);
  const double right_leg_length_force =
      u_leg_length_r + leg_length_gravity_compensation;
  const double left_leg_length_force =
      u_leg_length_l + leg_length_gravity_compensation;

  const LqrVector lqr_target = BuildLqrTarget(control_targets);
  const LqrVector lqr_states_l = BuildLqrStates(sensor_data.left_leg,
                                                sensor_data);
  const LqrVector lqr_states_r = BuildLqrStates(sensor_data.right_leg,
                                                sensor_data);
  LqrTorqueOutput LQR_L =
      CalcLqrTorque(left_leg.leg_length, lqr_target, lqr_states_l);
  LqrTorqueOutput LQR_R =
      CalcLqrTorque(right_leg.leg_length, lqr_target, lqr_states_r);

  const float steer_output = PIDCalculate(
      &steer_v_pid, static_cast<float>(sensor_data.base_link.yaw_rate),
      static_cast<float>(control_targets.target_yaw_rate));
  const double swerving_speed_ff =
      kSwervingSpeedFeedforwardCoef * steer_output;
  const float anti_crash_output = PIDCalculate(
      &anti_crash_pid, static_cast<float>(left_leg.phi - right_leg.phi), 0.0f);
  const double anti_crash_hip_torque =
      -anti_crash_output + swerving_speed_ff;
  const double left_lqr_hip_torque =
      LQR_L.hip_torque + anti_crash_hip_torque;
  const double right_lqr_hip_torque =
      LQR_R.hip_torque - anti_crash_hip_torque;

  VmcOutput T_r =  SerialVMC(-right_leg_length_force, -right_lqr_hip_torque, right_leg.leg_length,
                           right_leg.phi, right_leg.hip_absolute,
                           right_leg.calf_absolute);
  VmcOutput T_l =  SerialVMC(-left_leg_length_force, -left_lqr_hip_torque, left_leg.leg_length,
                           left_leg.phi, left_leg.hip_absolute,
                           left_leg.calf_absolute);
  
  Set_Val(m, d, "right_hip_motor", T_r.joint1_torque);
  Set_Val(m, d, "right_knee_motor", T_r.joint2_torque);
  Set_Val(m, d, "left_hip_motor", T_l.joint1_torque);
  Set_Val(m, d, "left_knee_motor", T_l.joint2_torque);
  const double right_wheel_torque = LQR_R.wheel_torque + steer_output;;
  const double left_wheel_torque = LQR_L.wheel_torque - steer_output;
  Set_Val(m, d, "right_wheel_motor", right_wheel_torque);
  Set_Val(m, d, "left_wheel_motor", left_wheel_torque);
  ApplyRos2Command(m, d);

  // PlotLines("phi", "Phi Tracking", "%.1f deg",
  //           {{"target", RadiansToDegrees(target_phi)},
  //            {"current", RadiansToDegrees(right_leg.phi)}});
  // PlotLines("phi_rate", "Phi Rate", "%.1f deg/s",
  //           {{"target", 0.0},
  //            {"right", RadiansToDegrees(right_leg.phi_rate)},
  //            {"left", RadiansToDegrees(left_leg.phi_rate)}});
  PlotLines("leg_length", "Leg Length Tracking", "%.3f m",
            {{"target", target_leg_length},
             {"current", right_leg.leg_length}});
  // PlotLines("control_targets", "Control Targets", "%.3f",
  //           {{"target_v", control_targets.target_velocity},
  //            {"target_wz", control_targets.target_yaw_rate},
  //            {"target_len", control_targets.target_leg_length}});
  // PlotLines("yaw_rate_targets", "Yaw Rate Targets", "%.3f deg/s",
  //           {{"target_yaw_rate",
  //             RadiansToDegrees(control_targets.target_yaw_rate)},
  //            {"current_yaw_rate",
  //             RadiansToDegrees(sensor_data.base_link.yaw_rate)}});
  // PlotLines("phi_diff_targets", "Phi Diff Targets", "%.3f deg",
  //           {{"target_phi_diff", 0.0},
  //            {"current_phi_diff",
  //             RadiansToDegrees(left_leg.phi - right_leg.phi)}});


  if (step_count % 120 == 0) {
    // std::cout << std::fixed << std::setprecision(6);
    // std::cout << "[state] t=" << d->time << "s" << std::endl;
    // PrintLqrVector("  target", lqr_target);
    // std::cout << std::endl;
    // PrintLqrVector("  left  ", lqr_states_l);
    // std::cout << std::endl;
    // PrintLqrVector("  right ", lqr_states_r);
    // std::cout << std::endl;
    // std::cout << "  leg_length left=" << left_leg.leg_length
    //           << "m, right=" << right_leg.leg_length << "m" << std::endl;
    // PrintMatlabVector("target", lqr_target);
    // std::cout << std::endl;
    // PrintMatlabVector("left_state", lqr_states_l);
    // std::cout << std::endl;
    // PrintMatlabVector("right_state", lqr_states_r);
    // std::cout << std::endl;
    // std::cout << "  LQR_L wheel_torque=" << LQR_L.wheel_torque
    //           << "Nm, hip_torque=" << LQR_L.hip_torque
    //           << "Nm, torque_magnitude=" << LQR_L.torque_magnitude
    //           << ", fly_flag=" << LQR_L.fly_flag << std::endl;
    // std::cout << "  LQR_R wheel_torque=" << LQR_R.wheel_torque
    //           << "Nm, hip_torque=" << LQR_R.hip_torque
    //           << "Nm, torque_magnitude=" << LQR_R.torque_magnitude
    //           << ", fly_flag=" << LQR_R.fly_flag << std::endl;
    // std::cout << "  applied_wheel_torque left=" << left_wheel_torque
    //           << "Nm, right=" << right_wheel_torque << "Nm" << std::endl;
    // std::cout << "  steer target_wz="
    //           << RadiansToDegrees(control_targets.target_yaw_rate)
    //           << "deg/s, wz="
    //           << RadiansToDegrees(sensor_data.base_link.yaw_rate)
    //           << "deg/s, output=" << steer_output << "Nm" << std::endl;
    // std::cout << "  anti_crash phi_diff="
    //           << RadiansToDegrees(left_leg.phi - right_leg.phi)
    //           << "deg, output=" << anti_crash_output
    //           << "Nm, swerving_ff=" << swerving_speed_ff
    //           << "Nm, applied_hip_torque left=" << left_lqr_hip_torque
    //           << "Nm, right=" << right_lqr_hip_torque << "Nm" << std::endl;
    // std::cout << "  yaw_rate="
    //           << RadiansToDegrees(sensor_data.base_link.yaw_rate)
    //           << "deg/s, steer_output=" << steer_output << "Nm"
    //           << std::endl;
    // std::cout << "  phi_diff=" << RadiansToDegrees(left_leg.phi - right_leg.phi)
    //           << "deg, anti_crash_output=" << anti_crash_output << "Nm"
    //           << "deg, swerving_ff=" << swerving_speed_ff << "Nm" << std::endl
    //           << std::endl;
    // std::cout << "yaw=" << RadiansToDegrees(sensor_data.base_link.yaw)
    //           << std::endl;
    // std::cout << "  leg_length_gravity_comp="
    //           << leg_length_gravity_compensation
    //           << "N, leg_length_force left=" << left_leg_length_force
    //           << "N, right=" << right_leg_length_force << "N"
    //           << std::endl;

  }
  ++step_count;
  (void)m;
  (void)d;
}

void AfterStep(const mjModel* m, const mjData* d) {
  static int step_count = 0;
  PublishRos2State(m, d);
  if (step_count % 120 == 0) {
    
    // PrintSensors(m, d)
  
  }
  ++step_count;
}

}  // namespace wheel_leg

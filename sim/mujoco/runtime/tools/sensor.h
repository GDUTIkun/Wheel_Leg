#ifndef WHEEL_LEG_SIMULATE_TOOLS_SENSOR_H_
#define WHEEL_LEG_SIMULATE_TOOLS_SENSOR_H_

#include <array>
#include <mujoco/mujoco.h>

#include <wheel_leg_sim/sensor_types.hpp>

namespace wheel_leg {

using wheel_leg_sim::BaseLinkState;
using wheel_leg_sim::JointState;
using wheel_leg_sim::LegKinematics;
using wheel_leg_sim::LegState;
using wheel_leg_sim::RobotSensorData;
using wheel_leg_sim::SensorAssemblyState;

RobotSensorData AssembleSensorData(
    const mjModel* m, const mjData* d, SensorAssemblyState* state);

RobotSensorData AssembleSensorData(const mjModel* m, const mjData* d);

void ResetSensorAssemblyState();

double ReadScalarSensor(const mjModel* m, const mjData* d,
                        const char* sensor_name);

std::array<double, 4> ReadQuaternionSensor(const mjModel* m, const mjData* d,
                                           const char* sensor_name);

std::array<double, 3> ReadVectorSensor(const mjModel* m, const mjData* d,
                                       const char* sensor_name);

}  // namespace wheel_leg

#endif  // WHEEL_LEG_SIMULATE_TOOLS_SENSOR_H_

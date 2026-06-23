#ifndef __JY901S_H
#define __JY901S_H

#include "jy901s_register.h"
#include "MyI2C.h"

#ifdef __cplusplus
extern "C" {
#endif

void JY901S_Acc(double* accx, double* accy, double* accz);
//void JY901S_Gyro(double* gyx, double* gyy, double* gyz);
void JY901S_Gyro(double* gyx, double* gyy, double* gyz, double* y);
void JY901S_Angle(double* roll, double* pitch, double* yaw);

#ifdef __cplusplus
}
#endif

#endif


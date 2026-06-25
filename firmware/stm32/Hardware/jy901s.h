#ifndef __JY901S_H
#define __JY901S_H

#include "jy901s_register.h"
#include "MyI2C.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float accx;
    float accy;
    float accz;
    float gyx;
    float gyy;
    float gyz;
    float roll;
    float pitch;
    float yaw;
} JY901SSnapshot;

void JY901S_ReadSnapshot(JY901SSnapshot *snapshot);
void JY901S_Acc(volatile float* accx, volatile float* accy, volatile float* accz);
void JY901S_Gyro(volatile float* gyx, volatile float* gyy, volatile float* gyz, volatile float* yaw);
void JY901S_Angle(volatile float* roll, volatile float* pitch);

#ifdef __cplusplus
}
#endif

#endif

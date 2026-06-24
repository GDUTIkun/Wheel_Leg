#include "jy901s.h"

void JY901S_Acc(double* accx, double* accy, double* accz)
{
    *accx = (double)(int16_t)I2C_ReadReg(JY901SADDRESSS, 0x34)*0.0004883;
    *accy = (double)(int16_t)I2C_ReadReg(JY901SADDRESSS, 0x35)*0.0004883;
    *accz = (double)(int16_t)I2C_ReadReg(JY901SADDRESSS, 0x36)*0.0004883;
}

void JY901S_Gyro(double* gyx, double* gyy, double* gyz, double* y)
{
    *gyx = (double)(int16_t)I2C_ReadReg(JY901SADDRESSS, 0x37)*0.061035;
    *gyy = (double)(int16_t)I2C_ReadReg(JY901SADDRESSS, 0x38)*0.061035;
    *gyz = (double)(int16_t)I2C_ReadReg(JY901SADDRESSS, 0x39)*0.061035;
    *y += *gyz*0.005;
}
//*1.2414

void JY901S_Angle(double* roll, double* pitch, double* yaw)
{
    *roll = (double)(int16_t)I2C_ReadReg(JY901SADDRESSS, 0x3d)*0.0054932;
    *pitch = (double)(int16_t)I2C_ReadReg(JY901SADDRESSS, 0x3e)*0.0054932;
    *yaw = (double)(int16_t)I2C_ReadReg(JY901SADDRESSS, 0x3f)*0.0054932;
}





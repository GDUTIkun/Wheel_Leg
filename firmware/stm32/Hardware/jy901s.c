#include "jy901s.h"

static const uint8_t kSnapshotStartReg = AX;
static const uint16_t kSnapshotByteCount = 24u;
static const float kAccScale = 0.0004883f;
static const float kGyroScale = 0.061035f;
static const float kAngleScale = 0.0054932f;

static int16_t ReadI16Le(const uint8_t *buf, uint16_t offset)
{
    return (int16_t)((uint16_t)buf[offset] | ((uint16_t)buf[offset + 1u] << 8));
}

void JY901S_ReadSnapshot(JY901SSnapshot *snapshot)
{
    uint8_t raw[kSnapshotByteCount] = {0};

    if (snapshot == 0) {
        return;
    }

    I2C_ReadRegs(JY901SADDRESSS, kSnapshotStartReg, raw, kSnapshotByteCount);

    snapshot->accx = (float)ReadI16Le(raw, 0u) * kAccScale;
    snapshot->accy = (float)ReadI16Le(raw, 2u) * kAccScale;
    snapshot->accz = (float)ReadI16Le(raw, 4u) * kAccScale;
    snapshot->gyx = (float)ReadI16Le(raw, 6u) * kGyroScale;
    snapshot->gyy = (float)ReadI16Le(raw, 8u) * kGyroScale;
    snapshot->gyz = (float)ReadI16Le(raw, 10u) * kGyroScale;
    snapshot->roll = (float)ReadI16Le(raw, 18u) * kAngleScale;
    snapshot->pitch = (float)ReadI16Le(raw, 20u) * kAngleScale;
    snapshot->yaw = (float)ReadI16Le(raw, 22u) * kAngleScale;
}

void JY901S_Acc(volatile float* accx, volatile float* accy, volatile float* accz)
{
    JY901SSnapshot snapshot = {0};
    JY901S_ReadSnapshot(&snapshot);
    *accx = snapshot.accx;
    *accy = snapshot.accy;
    *accz = snapshot.accz;
}

void JY901S_Gyro(volatile float* gyx, volatile float* gyy, volatile float* gyz, volatile float* yaw)
{
    JY901SSnapshot snapshot = {0};
    JY901S_ReadSnapshot(&snapshot);
    *gyx = snapshot.gyx;
    *gyy = snapshot.gyy;
    *gyz = snapshot.gyz;
    *yaw += *gyz * 0.005f;
}
//*1.2414

void JY901S_Angle(volatile float* roll, volatile float* pitch)
{
    JY901SSnapshot snapshot = {0};
    JY901S_ReadSnapshot(&snapshot);
    *roll = snapshot.roll;
    *pitch = snapshot.pitch;
}


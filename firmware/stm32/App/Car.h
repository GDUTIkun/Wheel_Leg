#ifndef __CAR_H
#define __CAR_H
#include "FreeRTOS.h"
#include "task.h"
#include "jy901s.h"
#include "dvc_motor_dji.h"

void Car_Init(void);

extern bool init_finished;
extern double gyx, gyy, gyz, y;

#endif


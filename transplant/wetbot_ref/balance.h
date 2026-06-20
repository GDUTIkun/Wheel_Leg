#pragma once

#include <webots/robot.h>
#include "stdint.h"

// 底盘参数
#define THIGH_LEN 0.15f           // 大腿
#define CALF_LEN 0.25f            // 小腿
#define JOINT_DISTANCE 0.108f     // 关节间距
#define WHEEL_RADIUS 0.0675f      // 轮子半径
#define WHEEL_MASS 3.0f           // 轮子质量
#define MAX_ACC_REF 1.0f

#define VEL_PROCESS_NOISE 15  // 速度过程噪声
#define VEL_MEASURE_NOISE 1000   // 速度测量噪声

// 常数
#define PI 3.14159265358979f
#define g 9.81f

// 任务运行步长
#define TIME_STEP 2

// 设备
typedef struct
{
    // IMU
    WbDeviceTag imu;
    WbDeviceTag gyro;
    WbDeviceTag accelerometer;

    // Motor
    WbDeviceTag LF_Joint_Motor;
    WbDeviceTag LB_Joint_Motor;
    WbDeviceTag RF_Joint_Motor;
    WbDeviceTag RB_Joint_Motor;

    WbDeviceTag L_Driven_Motor;
    WbDeviceTag R_Driven_Motor;

    // Pos_Sensor
    WbDeviceTag LF_Joint_Pos_Sensor;
    WbDeviceTag LB_Joint_Pos_Sensor;
    WbDeviceTag RF_Joint_Pos_Sensor;
    WbDeviceTag RB_Joint_Pos_Sensor;

    WbDeviceTag L_Driven_Pos_Sensor;
    WbDeviceTag R_Driven_Pos_Sensor;

} Devices;


typedef enum 
{
    JUMP_READDY = 0,    // 跳跃准备
    JUMP_START,         // 跳跃开始
    RESET_LEG,          // 复位腿
    EXTRACT_LEG,        // 提取腿
    RETRACT_LEG,        // 撤回腿
    LAND,               // 着陆
} Jump_State_e;


typedef struct
{
    // joint
    float phi1_w, phi4_w, phi2_w, phi5_w; // phi2_w used for calc real wheel speed
    float T_back, T_front;
    
    // link angle, phi1-ph5, phi5 is pod angle
    float phi1, phi2, phi3, phi4, phi5;
    float phi1_last, phi4_last;

    // wheel
    float ecd, ecd_last;
    float w_ecd;      // 电机编码器速度
    float wheel_w;    // 单侧轮子的速度
    float body_v;     // 髋关节速度
    float T_wheel;
    float zw_ddot;    // 驱动轮竖直方向加速度
    float normal_force;  // 支持力
    float gravity_comp;  // 重力补偿
    uint8_t fly_flag;    // 离地标志位

    // pod
    float theta, theta_w, theta_w_last; // 杆和垂直方向的夹角,为控制状态之一
    float leg_len, legd, legd_last;
    float height, height_v;
    float F_leg, T_hip;
    float target_len;

    float coord[6]; // xb yb xc yc xd yd

    // jump
    Jump_State_e jump_state;

} LinkNPodParam;

typedef struct
{
    // 速度
    float vel, target_v;        // 底盘速度
    float vel_m;                // 底盘速度测量值
    float vel_predict;          // 底盘速度预测值
    float vel_cov;              // 速度方差
    float acc_m, acc_last;      // 机体水平方向加速度,用于计算速度预测值

    // 位移
    float dist, target_dist;   // 底盘位移距离

    // IMU
    float yaw, yaw_last, yaw_round_count;
    float yaw_total, wz, target_yaw, target_wz; // yaw角度和底盘角速度
    float pitch, pitch_w;      // 底盘俯仰角度和角速度
    float roll, roll_w;        // 底盘横滚角度和角速度
    double accx, accy, accz;    // 三轴加速度
    double MotionAccz;          // 机体竖直方向加速度

} ChassisParam;

void BalanceInit();

void BalanceTask();
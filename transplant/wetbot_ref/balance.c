#include "balance.h"
#include "stdio.h"
#include "math.h"
#include "user_lib.h"
#include <webots/inertial_unit.h>
#include <webots/accelerometer.h>    
#include <webots/gyro.h> 
#include <webots/position_sensor.h>
#include <webots/motor.h> 
#include <webots/keyboard.h>
#include "linkNleg.h"
#include "lqr_calc.h"
#include "speed_estimation.h"
#include "pid.h"
#include "fly_detection.h"

static LinkNPodParam l_side, r_side;
static ChassisParam chassis;
static Devices devices;

// 腿长PID控制
static PIDInstance leglen_pid_l, leglen_pid_r;
// roll轴补偿
static PIDInstance roll_compensate_pid;
// 航向控制
static PIDInstance steer_v_pid;
// 抗劈叉
static PIDInstance anti_crash_pid;

static int key;    // 键盘值
static float delta_t;


// 设备初始化
static void WB_Devices_Init(void)
{
    // IMU
    devices.imu = wb_robot_get_device("imu");
    wb_inertial_unit_enable(devices.imu, TIME_STEP);

    devices.gyro = wb_robot_get_device("gyro");
    wb_gyro_enable(devices.gyro, TIME_STEP);

    devices.accelerometer = wb_robot_get_device("accelerometer");
    wb_accelerometer_enable(devices.accelerometer, TIME_STEP);

    // Joint Motor
    devices.LB_Joint_Motor = wb_robot_get_device("LB_Joint_Motor");
    wb_motor_enable_torque_feedback(devices.LB_Joint_Motor, TIME_STEP);

    devices.LF_Joint_Motor = wb_robot_get_device("LF_Joint_Motor");
    wb_motor_enable_torque_feedback(devices.LF_Joint_Motor, TIME_STEP);

    devices.RB_Joint_Motor = wb_robot_get_device("RB_Joint_Motor");
    wb_motor_enable_torque_feedback(devices.LB_Joint_Motor, TIME_STEP);

    devices.RF_Joint_Motor = wb_robot_get_device("RF_Joint_Motor");
    wb_motor_enable_torque_feedback(devices.RF_Joint_Motor, TIME_STEP);

    // Drivern Motor
    devices.L_Driven_Motor = wb_robot_get_device("L_Driven_Motor");
    wb_motor_enable_torque_feedback(devices.L_Driven_Motor, TIME_STEP);
    wb_motor_set_position(devices.L_Driven_Motor, 0.0f);

    devices.R_Driven_Motor = wb_robot_get_device("R_Driven_Motor");
    wb_motor_enable_torque_feedback(devices.R_Driven_Motor, TIME_STEP);
    wb_motor_set_position(devices.R_Driven_Motor, 0.0f);

    // Joint Position Sensor
    devices.LF_Joint_Pos_Sensor = wb_robot_get_device("LF_Joint_Pos_Sensor");
    wb_position_sensor_enable(devices.LF_Joint_Pos_Sensor, TIME_STEP);

    devices.LB_Joint_Pos_Sensor = wb_robot_get_device("LB_Joint_Pos_Sensor");
    wb_position_sensor_enable(devices.LB_Joint_Pos_Sensor, TIME_STEP);

    devices.RF_Joint_Pos_Sensor = wb_robot_get_device("RF_Joint_Pos_Sensor");
    wb_position_sensor_enable(devices.RF_Joint_Pos_Sensor, TIME_STEP);

    devices.RB_Joint_Pos_Sensor = wb_robot_get_device("RB_Joint_Pos_Sensor");
    wb_position_sensor_enable(devices.RB_Joint_Pos_Sensor, TIME_STEP);

    // Driven Position Sensor
    devices.L_Driven_Pos_Sensor = wb_robot_get_device("L_Driven_Pos_Sensor");
    wb_position_sensor_enable(devices.L_Driven_Pos_Sensor, TIME_STEP);

    devices.R_Driven_Pos_Sensor = wb_robot_get_device("R_Driven_Pos_Sensor");
    wb_position_sensor_enable(devices.R_Driven_Pos_Sensor, TIME_STEP);

    // Key Board
    wb_keyboard_enable(TIME_STEP);
}


// 参数初始化
void BalanceInit()
{
    // 设备初始化
    WB_Devices_Init();

    // 腿长控制PID
    PID_Init_Config_s leg_length_pid_conf = {
        .Kp = 4000,
        .Kd = 200,
        .Ki = 500,
        .IntegralLimit = 20,
        .CoefA = 0.005,
        .CoefB = 0.01,
        .MaxOut = 60,
        .DeadBand = 0.0001f,
        .Improve = PID_ChangingIntegrationRate | PID_Trapezoid_Intergral | PID_DerivativeFilter | PID_Derivative_On_Measurement,
        .Derivative_LPF_RC = 0.01,
    };
    PIDInit(&leglen_pid_l, &leg_length_pid_conf);
    PIDInit(&leglen_pid_r, &leg_length_pid_conf);

    // roll轴补偿
    PID_Init_Config_s roll_compensate_pid_conf = {
        .Kp = 0.0006f,
        .Kd = 0.00005f,
        .Ki = 0.0f,
        .MaxOut = 0.05,
        .DeadBand = 0.0001f,
        .Improve = PID_DerivativeFilter | PID_Derivative_On_Measurement,
        .Derivative_LPF_RC = 0.01,
    };
    PIDInit(&roll_compensate_pid, &roll_compensate_pid_conf);

    // 航向控制
    PID_Init_Config_s steer_v_pid_conf = {
        .Kp = 7,
        .Kd = 0.0f,
        .Ki = 0.0f,
        .MaxOut = 10,
        .DeadBand = 0.001f,
        .Improve = PID_DerivativeFilter | PID_Integral_Limit,
        .Derivative_LPF_RC = 0.01,
    };
    PIDInit(&steer_v_pid, &steer_v_pid_conf);

    // 抗劈叉
    PID_Init_Config_s anti_crash_pid_conf = {
        .Kp = 8,
        .Kd = 2.5,
        .Ki = 0.0,
        .MaxOut = 10,
        .DeadBand = 0.001f,
        .Improve = PID_DerivativeFilter | PID_ChangingIntegrationRate | PID_Integral_Limit,
        .Derivative_LPF_RC = 0.01,
    };
    PIDInit(&anti_crash_pid, &anti_crash_pid_conf);

    // 初始腿长
    r_side.target_len = l_side.target_len = 0.2;
    // 速度协方差初始化
    chassis.vel_cov = 100;
    // 速度距离初始化
    chassis.target_v = 0;
    chassis.target_dist = 0;
    // 电机初始角度
    l_side.phi1_last = r_side.phi1_last = 0.7 + PI / 2;
    l_side.phi4_last = r_side.phi4_last = -0.7 + PI / 2;
    l_side.ecd_last = r_side.ecd_last = 0;
    // yaw轴初始化
    chassis.target_yaw = 0;
    chassis.target_wz = 0;
    chassis.yaw_last = 0;
    chassis.yaw_round_count = 0;
    // 支持力初始化
    l_side.normal_force = r_side.normal_force = 110;
    // 连杆参数初始化
    l_side.theta_w_last = r_side.theta_w_last = 0;
    l_side.legd_last = r_side.legd_last = 0;
    // 跳跃初始化
    l_side.jump_state = r_side.jump_state = JUMP_READDY;
    // 任务运行步长
    delta_t = 0.001f * TIME_STEP;
}


static void WB_Get_IMU_Info()
{
    const double *imu_ = NULL, *gyro_ = NULL, *acc_ = NULL;

    imu_ = wb_inertial_unit_get_roll_pitch_yaw(devices.imu);
    gyro_ = wb_gyro_get_values(devices.gyro);
    acc_ = wb_accelerometer_get_values(devices.accelerometer);

    // Euler Angel
    chassis.roll = (float)imu_[0];
    chassis.pitch = -(float)imu_[1];   // 物理模型中pitch与仿真中imu的pitch相反
    chassis.yaw = (float)imu_[2];

    // 计算yaw轴圈数和总角度
    if(chassis.yaw - chassis.yaw_last > PI)
        chassis.yaw_round_count--;
    else if(chassis.yaw - chassis.yaw_last < -PI)
        chassis.yaw_round_count++;

    chassis.yaw_last = chassis.yaw;
    chassis.yaw_total = 2 * PI * chassis.yaw_round_count + chassis.yaw;

    // Gyro
    chassis.roll_w = (float)gyro_[0];
    chassis.pitch_w = -(float)gyro_[1];
    chassis.wz = (float)gyro_[2];

    // 修正重力加速度g的影响, 注意方向
    chassis.accx = acc_[0] - g * sin(imu_[1]);
    chassis.accy = acc_[1] - g * cos(imu_[1]) * sin(-imu_[0]);
    chassis.accz = acc_[2] - g * cos(imu_[1]) * cos(imu_[0]);

    chassis.MotionAccz = chassis.accx * sin(imu_[1]) + \
                        chassis.accy * sin(-imu_[0]) * cos(imu_[1]) + \
                        chassis.accz * cos(imu_[1]) * cos(imu_[0]);
}


static void WB_Get_Pos_Sensor_Info()
{
    // Joint
    l_side.phi1 = PI / 2 + (float)wb_position_sensor_get_value(devices.LB_Joint_Pos_Sensor);
    l_side.phi1_w = (l_side.phi1 - l_side.phi1_last) / delta_t;
    l_side.phi1_last = l_side.phi1;

    l_side.phi4 = PI / 2 + (float)wb_position_sensor_get_value(devices.LF_Joint_Pos_Sensor);
    l_side.phi4_w = (l_side.phi4 - l_side.phi4_last) / delta_t;
    l_side.phi4_last = l_side.phi4;

    r_side.phi1 = PI / 2 + (float)wb_position_sensor_get_value(devices.RB_Joint_Pos_Sensor);
    r_side.phi1_w = (r_side.phi1 - r_side.phi1_last) / delta_t;
    r_side.phi1_last = r_side.phi1;

    r_side.phi4 = PI / 2 + (float)wb_position_sensor_get_value(devices.RF_Joint_Pos_Sensor);
    r_side.phi4_w = (r_side.phi4 - r_side.phi4_last) / delta_t;
    r_side.phi4_last = r_side.phi4;

    // Driven
    l_side.ecd = (float)wb_position_sensor_get_value(devices.L_Driven_Pos_Sensor);
    l_side.w_ecd = (l_side.ecd - l_side.ecd_last) / delta_t;
    l_side.ecd_last = l_side.ecd;

    r_side.ecd = (float)wb_position_sensor_get_value(devices.R_Driven_Pos_Sensor);
    r_side.w_ecd = (r_side.ecd - r_side.ecd_last) / delta_t;
    r_side.ecd_last = r_side.ecd;
}


// 参数组装
static void ParamAssemble()
{
    WB_Get_IMU_Info();
    WB_Get_Pos_Sensor_Info();
}


// 键盘控制
static void KeyBoardControl()
{
    static float delta_leglen;
    delta_leglen = 0.0008f;

    key = wb_keyboard_get_key();
    switch (key)
    {
    case 'W':
        chassis.target_v += MAX_ACC_REF * delta_t;
        break;
    case 'S':
        chassis.target_v -= MAX_ACC_REF * delta_t;
        break;
    case 'A':
        chassis.target_wz = 3.5f;
        break;
    case 'D':
        chassis.target_wz = -3.5f;
        break;
    case 'Q':
        l_side.target_len += delta_leglen;
        r_side.target_len += delta_leglen;
        break;
    case 'E':
        l_side.target_len -= delta_leglen;
        r_side.target_len -= delta_leglen;
        break;
    case 'F':   // 复位腿
        l_side.target_len = r_side.target_len = 0.14;
        break;
    case 'J':
        l_side.jump_state = r_side.jump_state = JUMP_START;
        break;
    default:
        if(chassis.target_v != 0 && chassis.target_v > 0)
            chassis.target_v -= MAX_ACC_REF * delta_t;
        else if(chassis.target_v != 0 && chassis.target_v < 0)
            chassis.target_v += MAX_ACC_REF * delta_t;

        chassis.target_wz = 0;
        break;
    }

    // 腿长限幅
    if (l_side.jump_state != EXTRACT_LEG && r_side.jump_state != EXTRACT_LEG)
    {
        VAL_LIMIT(l_side.target_len, 0.14f, 0.25f);
        VAL_LIMIT(r_side.target_len, 0.14f, 0.25f);
    }

    // 速度限幅
    VAL_LIMIT(chassis.target_v, -2.5f, 2.5f);

    // 速度积分得到期望位移
    chassis.target_dist += chassis.target_v * delta_t;
}


// 腿长控制
static void LegControl()
{
    // 跳跃
    Jump(&l_side);
    Jump(&r_side);

    // roll补偿
    PIDCalculate(&roll_compensate_pid, chassis.roll, 0);
    l_side.target_len += roll_compensate_pid.Output;
    r_side.target_len -= roll_compensate_pid.Output;

    // 动态调整重力补偿
    GravityCompAdjust(&l_side, &r_side);

    static float roll_extra_comp_p = 300;
    float roll_comp = roll_extra_comp_p * chassis.roll;
    l_side.F_leg = PIDCalculate(&leglen_pid_l, l_side.height, l_side.target_len) + l_side.gravity_comp - roll_comp;
    r_side.F_leg = PIDCalculate(&leglen_pid_r, r_side.height, r_side.target_len) + r_side.gravity_comp + roll_comp;
}


// 航向控制和抗劈叉
static void SynthesizeMotion()
{
    PIDCalculate(&steer_v_pid, chassis.wz, chassis.target_wz);  // 目前仅使用速度环

    l_side.T_wheel -= steer_v_pid.Output;
    r_side.T_wheel += steer_v_pid.Output;

    static float swerving_speed_ff, ff_coef = 5;
    swerving_speed_ff = ff_coef * steer_v_pid.Output; // 用于抗劈叉的前馈
    PIDCalculate(&anti_crash_pid, l_side.phi5 - r_side.phi5, 0);
    l_side.T_hip += anti_crash_pid.Output - swerving_speed_ff;
    r_side.T_hip -= anti_crash_pid.Output - swerving_speed_ff;
}


// 运动模态设定
static void WattLimitSet()
{
    // Joint
    if(l_side.jump_state == RETRACT_LEG && r_side.jump_state == RETRACT_LEG)
    {
        wb_motor_set_position(devices.LB_Joint_Motor, 2.0f);
        wb_motor_set_position(devices.RB_Joint_Motor, 2.0f);
        wb_motor_set_position(devices.LF_Joint_Motor, -2.0f);
        wb_motor_set_position(devices.RF_Joint_Motor, -2.0f);
    }
    else
    {
        wb_motor_set_torque(devices.LB_Joint_Motor, l_side.T_back);
        wb_motor_set_torque(devices.LF_Joint_Motor, l_side.T_front);
        wb_motor_set_torque(devices.RB_Joint_Motor, r_side.T_back);
        wb_motor_set_torque(devices.RF_Joint_Motor, r_side.T_front);
    }

    // Driven
    wb_motor_set_torque(devices.L_Driven_Motor, l_side.T_wheel);
    wb_motor_set_torque(devices.R_Driven_Motor, r_side.T_wheel);
}


void BalanceTask()
{
    // 键盘控制
    KeyBoardControl();
    // 参数组装
    ParamAssemble();

    // 五连杆映射
    Link2Leg(&l_side, &chassis);
    Link2Leg(&r_side, &chassis);

    // 速度估计
    SpeedEstimation(&l_side, &r_side, &chassis, delta_t);

    // 计算LQR增益
    CalcLQR(&r_side, &chassis);
    CalcLQR(&l_side, &chassis);

    // 转向和抗劈叉
    SynthesizeMotion();

    // 腿长控制
    LegControl();

    // VMC映射成关节输出
    VMCProject(&l_side);
    VMCProject(&r_side);

    // 运动模态设定
    WattLimitSet();

    // 支持力解算
    NormalForceSolve(&l_side, &chassis, delta_t);
    NormalForceSolve(&r_side, &chassis, delta_t);

}
#include "balance.h"
#include "math.h"
#include"user_lib.h"

/**
 * @brief 使用卡尔曼滤波估计底盘速度
 * @todo 增加w和dw的滤波,当w和dw均小于一定值时,不考虑dw导致的角加速度
 *
 * @param lp 左侧腿
 * @param rp 右侧腿
 * @param cp 底盘
 * @param delta_t 更新间隔
 */
void SpeedEstimation(LinkNPodParam *lp, LinkNPodParam *rp, ChassisParam *cp, float delta_t)
{
    // 修正轮速和距离
    lp->wheel_w = lp->w_ecd + lp->phi2_w - cp->pitch_w; // 减去和定子固连的phi2_w
    rp->wheel_w = rp->w_ecd + rp->phi2_w - cp->pitch_w;

    // 以轮子为基点,计算机体两侧髋关节处的速度
    lp->body_v = lp->wheel_w * WHEEL_RADIUS + lp->leg_len * lp->theta_w + lp->legd * sin(lp->theta);
    rp->body_v = rp->wheel_w * WHEEL_RADIUS + rp->leg_len * rp->theta_w + rp->legd * sin(rp->theta);
    cp->vel_m = (lp->body_v + rp->body_v) / 2; // 机体速度(平动)为两侧速度的平均值

    float pitch = cp->pitch;
    float acc_x = cp->accx;
    float acc_z = cp->accz;
    cp->acc_last = cp->acc_m; 
    cp->acc_m = acc_x * cos(pitch) - acc_z * sin(pitch); // 绝对系下的平动加速度,即机体系下的加速度投影到绝对系

    // 融合加速度计的数据和机体速度
    static float u, k;   // 输入和卡尔曼增益
    static float vel_prior, vel_measure, vel_cov;     // 先验估计、测量、先验协方差

    // 预测
    u = (cp->acc_m + cp->acc_last) / 2;         // 速度梯形积分
    vel_prior = cp->vel + delta_t * u;          // 先验估计
    vel_cov = cp->vel_cov + VEL_PROCESS_NOISE * delta_t;  // 先验协方差

    // 校正
    vel_measure = cp->vel_m;
    k = vel_cov / (vel_cov + VEL_MEASURE_NOISE);            // 卡尔曼增益
    cp->vel = vel_prior + k * (vel_measure - vel_prior);    // 后验估计
    cp->vel_cov = (1 - k) * vel_cov;                        // 后验协方差

    VAL_LIMIT(cp->vel_cov, 0.01, 100);       // 协方差限幅
    cp->dist = cp->dist + cp->vel * delta_t;
}
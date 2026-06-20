#include "balance.h"
#include "stdint.h"

/**
 * @brief 根据状态反馈计算当前腿长,查表获得LQR的反馈增益,并列式计算LQR的输出
 * @note 得到的腿部力矩输出还要经过综合运动控制系统补偿后映射为两个关节电机输出
 *
 */
void CalcLQR(LinkNPodParam *p, ChassisParam *chassis)
{
    float k[12][3] = {89.582677,-172.311264,-12.862450,
                    -7.406183,-29.225173,0.351843,
                    13.770239,-11.389716,-18.835586,
                    15.547229,-22.241056,-13.241076,
                    131.619584,-117.579480,32.974720,
                    6.898540,-6.543172,2.125882,
                    37.724640,-38.801531,18.576409,
                    1.358879,-1.070109,2.140319,
                    74.091942,-66.518020,18.803614,
                    51.675325,-46.798148,13.962229,
                    -60.271354,47.855082,121.051548,
                    -3.379791,2.755658,3.836537};
    float T[2] = {0};   // 0 T_wheel  1 T_hip
    float l = p->leg_len;
    float lsqr = l * l;


    // 离地检测
    if (p->normal_force < 20.0f || p->jump_state == RETRACT_LEG)
        p->fly_flag = 1;
    else
        p->fly_flag = 0;
    

    // 计算增益
    uint8_t i, j;
    // 离地时轮子输出置0
    i = 0; j = i * 6;
    T[i] = p->fly_flag ? 0 :
           ((k[j + 0][0] * lsqr + k[j + 0][1] * l + k[j + 0][2]) * -p->theta +
            (k[j + 1][0] * lsqr + k[j + 1][1] * l + k[j + 1][2]) * -p->theta_w + 
            (k[j + 2][0] * lsqr + k[j + 2][1] * l + k[j + 2][2]) * (chassis->target_dist - chassis->dist) +
            (k[j + 3][0] * lsqr + k[j + 3][1] * l + k[j + 3][2]) * (chassis->target_v - chassis->vel) +
            (k[j + 4][0] * lsqr + k[j + 4][1] * l + k[j + 4][2]) * -chassis->pitch +
            (k[j + 5][0] * lsqr + k[j + 5][1] * l + k[j + 5][2]) * -chassis->pitch_w);
    
    // 离地时关节输出仅保留 theta 和 theta_dot，保证滞空时腿部竖直
    i = 1; j = i * 6;
    T[i] = (k[j + 0][0] * lsqr + k[j + 0][1] * l + k[j + 0][2]) * -p->theta +
           (k[j + 1][0] * lsqr + k[j + 1][1] * l + k[j + 1][2]) * -p->theta_w + (p->fly_flag ? 0 :
           ((k[j + 2][0] * lsqr + k[j + 2][1] * l + k[j + 2][2]) * (chassis->target_dist - chassis->dist) +
            (k[j + 3][0] * lsqr + k[j + 3][1] * l + k[j + 3][2]) * (chassis->target_v - chassis->vel) +
            (k[j + 4][0] * lsqr + k[j + 4][1] * l + k[j + 4][2]) * -chassis->pitch +
            (k[j + 5][0] * lsqr + k[j + 5][1] * l + k[j + 5][2]) * -chassis->pitch_w));

    p->T_wheel = T[0];
    p->T_hip = T[1];
}
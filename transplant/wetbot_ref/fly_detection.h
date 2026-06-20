#include "balance.h"
#include "math.h"

// 支持力解算
void NormalForceSolve(LinkNPodParam *p, ChassisParam *chassis, float dt)
{
    static float leglen_ddot, theta_ddot;

    theta_ddot = (p->theta_w - p->theta_w_last) / dt;
    leglen_ddot = (p->legd - p->legd_last) / dt;
    p->theta_w_last = p->theta_w;
    p->legd_last = p->legd;

    // 驱动轮竖直方向加速度
    p->zw_ddot = chassis->MotionAccz - leglen_ddot * cos(p->theta) + 2 * p->legd * p->theta_w * sin(p->theta) + \
                p->leg_len * theta_ddot * sin(p->theta) + p->leg_len * powf(p->theta_w, 2) * cos(p->theta);

    // 驱动轮支持力解算
    static float P;
    P = p->F_leg * cos(p->theta) + p->T_hip * sin(p->theta) / p->leg_len;
    p->normal_force = WHEEL_MASS * p->zw_ddot + P + WHEEL_MASS * g;

}


// 动态调整重力补偿
void GravityCompAdjust(LinkNPodParam *lp, LinkNPodParam *rp)
{
    if(lp->normal_force < 20.0f && rp->normal_force < 20.0f)
    {
        lp->gravity_comp = rp->gravity_comp = 65.0f;
    }
    else if(lp->normal_force > 200.0f && rp->normal_force > 200.0f)
    {
        lp->gravity_comp = rp->gravity_comp = 190.0f;
    }
    else
        lp->gravity_comp = rp->gravity_comp = 110.0f;
}


void Jump(LinkNPodParam *p)
{
    // 跳跃开始，先复位腿
    if (p->jump_state == JUMP_START)
    {
        p->jump_state = RESET_LEG;
        p->target_len = 0.14;
    }

    // 复位腿完成，开始提取腿
    if(p->jump_state == RESET_LEG && p->height < 0.145)
    {
        p->jump_state = EXTRACT_LEG;
        p->target_len = 0.35;
    }

    // 提腿完成，开始撤回腿
    if (p->jump_state == EXTRACT_LEG && p->height > 0.35)
    {
        // 撤回腿使用电机位置控制
        p->jump_state = RETRACT_LEG;
    }

    // 撤回腿完成，准备着陆
    if(p->jump_state == RETRACT_LEG && p->height < 0.145)
    {
        p->jump_state = LAND;
        p->target_len = 0.14;
    }

    // 着陆完成，重新进入跳跃准备
    if(p->jump_state == LAND && p->normal_force > 100)
    {
        p->jump_state = JUMP_READDY;
        p->target_len = 0.2;
    }
    
}
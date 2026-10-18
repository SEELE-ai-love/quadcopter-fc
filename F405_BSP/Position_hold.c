#include "Position_hold.h"
#include <math.h>
#include <string.h>

static float PositionHold_Limit(float value,float limit)
{
    if(value > limit)
    {
        return limit;
    }
    if(value < -limit)
    {
        return -limit;
    }
    return value;
}

uint8_t PositionHold_Compute(const FlyPosMsg_t *estimate,
                             float target_x,
                             float target_y,
                             const PositionHoldConfig_t *config,
                             PositionHoldOutput_t *output)
{
    float control_x;
    float control_y;

    if(output == NULL)
    {
        return 0U;
    }
    memset(output,0,sizeof(*output));

    if(estimate == NULL || config == NULL)
    {
        return 0U;
    }

    if(!isfinite(estimate->x_pos) || !isfinite(estimate->y_pos) ||
       !isfinite(estimate->x_speed) || !isfinite(estimate->y_speed) ||
       !isfinite(target_x) || !isfinite(target_y) ||
       !isfinite(config->kp_x) || !isfinite(config->kp_y) ||
       !isfinite(config->kv_x) || !isfinite(config->kv_y) ||
       !isfinite(config->angle_limit) || config->angle_limit <= 0.0f)
    {
        return 0U;
    }

    /*============================
     *关于位置PD的计算：
     *1、位置误差乘kp，产生回到目标位置的倾角
     *2、速度乘kv，给当前运动加一个刹车
     *3、两项相减，最后的单位都是rad
    =============================*/
    control_x = config->kp_x * (target_x-estimate->x_pos)
              - config->kv_x * estimate->x_speed;
    control_y = config->kp_y * (target_y-estimate->y_pos)
              - config->kv_y * estimate->y_speed;

    if(!isfinite(control_x) || !isfinite(control_y))
    {
        return 0U;
    }
  
    output->control_x = control_x;
    output->control_y = control_y;

    // PD_CONTROL BEGIN：下方旧方向注释保留作历史，当前方向以本区块末尾说明为准
    /*============================
     *关于真实方向：
     *光流x位置、x速度向左为正，y位置、y速度向前为正
     *这里的姿态角已经取过负数，Roll抬头为正，Pitch向右压为正
     *所以向左的控制需求给负Pitch，向前的控制需求给负Roll
     *这里不改光流flowX/-flowY，也不再对姿态反馈或gxgy取负
     *实测下来的串口数据吻合：
     *向右平移：X误差为负、X速度为正，Pitch指令为负，对应左压纠偏
     *向前平移：Y误差为负、Y速度为正，Roll指令为正，对应抬头向后纠偏
    =============================*/
    // PD_CONTROL 当前X向右为正，Y向前为正；抬头roll正，右压pitch正
    // PD_CONTROL 向右的倾角需求给正pitch，向前的倾角需求给负roll
    output->pitch_add = PositionHold_Limit(control_x,config->angle_limit);
    output->roll_add = PositionHold_Limit(-control_y,config->angle_limit);
    // PD_CONTROL END

    return 1U;
}

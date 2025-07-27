//
// Created by 24158 on 25-7-6.
//

#include "PID.h"


float PID_Calc(PID_TypeDef *pid, float feedback) {
    float error = pid->setpoint - feedback;
    pid->integral += error;
    pid->last_error = error;
    float output = pid->Kp * error + pid->Ki * pid->integral;
    // 限幅
    if (output > MAX_OUTPUT) output = MAX_OUTPUT;
    if (output < MIN_OUTPUT) output = MIN_OUTPUT;
    return output;
}
// PWM输出
void Set_PWM_Duty(TIM_HandleTypeDef htim,uint32_t channel, float duty)
{
    //uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim);
    /* ARR直接使用宏定义代替 */
    __HAL_TIM_SET_COMPARE(&htim, channel, (uint32_t)(duty / 100.0f * AUTO_RELOAD_VALUE));
}

// ControlMode g_mode =MODE_CV;

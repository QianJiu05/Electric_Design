
#ifndef PID_H
#define PID_H

#include "main.h"
#define PID_CONTROL_NUM 6

#define PID_TIM1_CH1 0
#define PID_TIM1_CH2 1
#define PID_TIM1_CH3 2
#define PID_TIM8_CH1 3
#define PID_TIM8_CH2 4
#define PID_TIM8_CH3 5

#define AC_CH1  PID_TIM8_CH1
#define AC_CH2  PID_TIM8_CH2
#define AC_CH3  PID_TIM8_CH3

// typedef enum {
//     MODE_CV, // 恒压
//     MODE_CC  // 恒流
// } ControlMode;

typedef struct {
    float Kp;
    float Ki;
    // float Kd;
    float setpoint;
    float integral;
} PID_TypeDef;

float PID_Calc(PID_TypeDef *pid, float feedback);
void Set_PWM_Duty(TIM_HandleTypeDef htim,uint32_t channel, float duty);

// extern ControlMode g_mode;
// extern PID_TypeDef pid_V, pid_C;

#endif //PID_H

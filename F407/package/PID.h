//
// Created by 24158 on 25-7-6.
//

#ifndef PID_H
#define PID_H

#define PID_TIM1_CH1 0
#define PID_TIM1_CH2 1
#define PID_TIM1_CH3 2
#define PID_TIM8_CH2 3
#define PID_TIM8_CH3 4

typedef enum {
    MODE_CV, // 恒压
    MODE_CC  // 恒流
} ControlMode;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float setpoint;
    float integral;
    float last_error;
} PID_TypeDef;

typedef struct {
    PID_TypeDef voltage_pid;
    PID_TypeDef current_pid;
} DCDC_Channel;
extern DCDC_Channel dcdc_ch1, dcdc_ch2;

float PID_Calc(PID_TypeDef *pid, float feedback);
void Set_PWM_Duty(TIM_HandleTypeDef htim,uint32_t channel, float duty);

extern ControlMode g_mode;
extern PID_TypeDef pid_V, pid_C;

#endif //PID_H

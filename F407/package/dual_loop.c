#include <stdio.h>
#include <math.h>
#include <malloc.h>

#include "dual_loop.h"

#include "arm_math.h"
#include "main.h"

/* 全局系统实例 */
extern System_State sys_state;     // 系统状态
extern System_IO sys_io;           // 系统输入输出
extern triple_power AC_OUT, AC_IN;
extern RMS_Calculator *rms_discretep;
/*----------------------------------------------------------
 * 函数: system_init
 * 功能: 初始化整个系统状态
 *---------------------------------------------------------*/
void system_init(void)
{
    /* VCO 初始化 */
    // sys_state.vco_phase = 0.0f;
    sys_state.vco_phase = 0.0f;
    sys_state.vco_freq = 50.0f;

    /* PI 控制器初始化 */
    sys_state.integrator = 0.0f;

    /* IO 初始化 */
    for (int i = 0; i < 3; i++) {
        sys_io.vabc[i] = 0.0f;
        sys_io.abc[i] = 0.0f;
        sys_state.vabc[i] = 0.0f;
    }

    sys_io.freq_sp = 0.0f;
    sys_io.volt_ref = 0.0f;
    sys_io.theta = 0.0f;
    sys_io.freq_actual = 0.0f;
    rms_discretep = RMS_Init(0.0001, 50.0, 0.0);
}

/*----------------------------------------------------------
 * 函数: hypot_safe
 * 功能: 安全欧几里得范数计算 (防止数值溢出)
 * 说明: 比 sqrt(a² + b²) 更安全
 *---------------------------------------------------------*/
float hypot_safe(float a, float b)
{
    a = fabsf(a);
    b = fabsf(b);

    if (a == 0 && b == 0) return 0.0f;

    float sqrt_a_or_b;
    float ratio;


    if (a > b) {
        ratio = b / a;
        arm_sqrt_f32((1.0f + ratio * ratio),&sqrt_a_or_b);
        // return a * sqrtf(1.0f + ratio * ratio);
        return a * sqrt_a_or_b;
    } else {
        ratio = a / b;
        arm_sqrt_f32((1.0f + ratio * ratio),&sqrt_a_or_b);
        // return b * sqrtf(1.0f + ratio * ratio);
        return b * sqrt_a_or_b;
    }
}

/*----------------------------------------------------------
 * 函数: control_step
 * 功能: 闭环控制单步计算 (20kHz)
 * 说明:
 *   1. 执行 Clark 变换 (abc → αβ)
 *   2. 计算电压幅值 (|Vαβ|)
 *   3. PI 控制器计算 (幅值误差)
 *   4. 逆 Park 变换 (生成三相调制波)
 *   5. 更新积分器状态
 *---------------------------------------------------------*/
void control_step(void)
{
    // 1. Clark 变换: 三相→两相静止坐标系 (αβ)
    // α = 2/3 * [Va - 0.5*(Vb + Vc)]
//    float alpha = CLARK_GAIN * (sys_io.vabc[0] - 0.5f*(sys_io.vabc[1] + sys_io.vabc[2]));
    // β = (√3/3) * (Vb - Vc) = (1/√3)*(Vb - Vc)
    // 等效公式: β = (2/3) * (√3/2 * (Vb - Vc)) = (1/√3) * (Vb - Vc)
//    float beta = SQRT_3_OVER_2 * CLARK_GAIN * (sys_io.vabc[1] - sys_io.vabc[2]);
    // 2. 计算电压矢量幅值 (|Vαβ| = √(α² + β²))
//    float volt_meas = hypot_safe(alpha, beta);
    double volt_meas = RMS_Update(rms_discretep, sys_io.vabc[0], 0.001);
    // 3. PI 控制器计算
    float error = sys_io.volt_ref - volt_meas;
    float p_term = PI_GAIN_KP * error;
    float i_term = sys_state.integrator;  // 积分项使用预存值
    // PI 输出 = Kp*e + ∫Ki*e dt
    float pi_output = p_term + i_term;
    // 4. 逆 Park 变换: 生成三相调制波 (使用VCO产生的相位)
    // A相: M * sin(θ)
    // sys_io.abc[0] = pi_output * sinf(sys_io.theta)/30; // 乘以30是为了放大到PWM范围
    sys_io.abc[0] = pi_output * arm_sin_f32(sys_io.theta)/30.0f; // 乘以30是为了放大到PWM范围

    // B相: M * sin(θ - 120°)
    // sys_io.abc[1] = pi_output * sinf(sys_io.theta - TWO_PI_OVER_3)/30;
    sys_io.abc[1] = pi_output * arm_sin_f32(sys_io.theta - TWO_PI_OVER_3)/30.0f;
    // C相: M * sin(θ + 120°)
    sys_io.abc[2] = pi_output * arm_sin_f32(sys_io.theta + TWO_PI_OVER_3)/30.0f;

    //限制占空比


    // 5. 更新积分器状态: I[k+1] = I[k] + Ki * e * Ts
    sys_state.integrator += PI_GAIN_KI * error * TS_PLL;


}

/*----------------------------------------------------------
 * 函数: system_step
 * 功能: 系统主循环单步计算 (20kHz)
 * 说明: 顺序执行 VCO 和闭环控制
 *---------------------------------------------------------*/
void system_step(void)
{
    // 刷新输入缓冲
    for (int i = 0; i < 3; i++) {
        sys_state.vabc[i] = sys_io.vabc[i];
    }
    // 执行 VCO 计算 (生成相位)
    vco_step();
    // 执行闭环控制计算
    control_step();
}


RMS_Calculator* RMS_Init(double sample_time, double fundamental_freq, double initial_cond) {
    RMS_Calculator *rms = (RMS_Calculator*)malloc(sizeof(RMS_Calculator));
    rms->sample_time = sample_time;
    rms->fundamental_freq = fundamental_freq;
    rms->period = 1.0 / fundamental_freq;
    rms->initial_cond = initial_cond;
    rms->last_time = 0.0;

    if (sample_time > 0) {
        // 离散模式：分配缓冲区
        rms->buffer_size = (int)(rms->period / sample_time + 0.5);
        rms->buffer = (double*)malloc(rms->buffer_size * sizeof(double));

        // 用初始条件填充缓冲区
        for (int i = 0; i < rms->buffer_size; i++) {
            rms->buffer[i] = initial_cond;
        }
        rms->current_index = 0;
    } else {
        // 连续模式：初始化积分值
        rms->buffer = NULL;
        rms->buffer_size = 0;
        rms->integral = initial_cond * rms->period; // 初始积分值
    }

    return rms;
}

// 释放RMS计算器
void RMS_Free(RMS_Calculator *rms) {
    if (rms->buffer != NULL) {
        free(rms->buffer);
    }
    free(rms);
}

// 更新RMS值（连续模式）
double RMS_Update_Continuous(RMS_Calculator *rms, double input, double current_time) {
    double delta_time = current_time - rms->last_time;

    // 计算新的积分值（梯形法积分）
    double input_sq = input * input;
    double last_input_sq = rms->initial_cond * rms->initial_cond;
    rms->integral += (input_sq + last_input_sq) * delta_time / 2.0;

    // 减去超出周期的部分（移动窗口）
    if (current_time > rms->period) {
        double oldest_time = current_time - rms->period;
        // 这里简化处理，实际实现可能需要更精确的旧值计算
        rms->integral -= last_input_sq * delta_time;
    }

    rms->last_time = current_time;
    rms->initial_cond = input;

    return sqrt(rms->integral / rms->period);
}

// 更新RMS值（离散模式）
double RMS_Update_Discrete(RMS_Calculator *rms, double input) {
    // 将输入平方存入缓冲区
    rms->buffer[rms->current_index] = input * input;

    // 计算缓冲区中所有值的平均
    double sum = 0.0;
    for (int i = 0; i < rms->buffer_size; i++) {
        sum += rms->buffer[i];
    }
    double mean = sum / rms->buffer_size;

    // 更新索引（循环缓冲区）
    rms->current_index = (rms->current_index + 1) % rms->buffer_size;

    return sqrt(mean);
}

// 主RMS更新函数（根据模式自动选择）
double RMS_Update(RMS_Calculator *rms, double input, double current_time) {
    if (rms->sample_time > 0) {
        return RMS_Update_Discrete(rms, input);
    } else {
        return RMS_Update_Continuous(rms, input, current_time);
    }
}

/*************************** 示例代码 ***************************/
//示例代码
// // int main(void) {
//     // 系统初始化
//     system_init();
//
//     // 设置目标频率 (50Hz) 和参考电压 (1.0)
//     sys_io.freq_sp = 50.0f;
//     sys_io.volt_ref = 1.0f;
//
//     // 假设的三相电压测量值 (动态变化)
//     // 实际应用中应来自ADC采样
//     float t = 0.0f;
//
//     while (1) {
//         // 生成模拟的三相电压输入 (仅用于演示)//A-B,B-C,C-A,转换为三相电压
//         t += TS_PLL;
//
//         // 执行系统主循环
//         system_step();
//
//         // 此处可添加:
//         // 1. 将sys_io.abc输出到PWM发生器
//         // 2. 记录调试数据
//         // 3. 等待下一个20kHz中断触发
//     }
//
//     return 0;
// }
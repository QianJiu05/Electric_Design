#include "arm_math.h"

// float vac_sen = InputSignal(0, 0);
// float deta_theta = acos(InputSignal(0, 1));
// float temp_theta= 0.0f;

// extern spll_data_t spll_data;   // 原 C 结构体，保持不变
spll_sogi_func(&spll_data, vac_sen);

// #include "math.h"
void run_spll_arm_f32(void)
{
    float32_t vac_sen   = InputSignal(0, 0);
    float32_t deta_theta = acosf(InputSignal(0, 1));

    spll_sogi_func_f32(&spll_data, vac_sen);   // 内部全部 float32

    float32_t temp_theta = spll_data.theta - deta_theta;

    /* 限幅到 0 .. 2π */
    while (temp_theta < 0.0f)         temp_theta += 2.0f * PI_F;
    while (temp_theta > 2.0f * PI_F)  temp_theta -= 2.0f * PI_F;

    OutputSignal(0, 0) = temp_theta;
    OutputSignal(0, 1) = spll_data.pll_freq_out;
    OutputSignal(0, 2) = spll_data.spll_diff;
}
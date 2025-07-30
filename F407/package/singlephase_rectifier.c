#include "arm_math.h"
#include "singlephase_rectifier.h"
/*
 *   如果还想再压榨 10 %，可把 arm_sin_f32 / arm_cos_f32 换成查表 + 线性插值（自己建 256 点表即可）。
 *   mdk-armcc效率比arm-gcc高
 *   gcc -ofast
 *
 */

void sogi_pll_init(SOGI_PLL_DATA_DEF *spll_obj, float32_t grid_freq, float32_t ts)
{
    spll_obj->grid_freq = grid_freq;
    spll_obj->delta_t = ts;

    // Initialize SOGI coefficients
    spll_obj->sogi_u_coeff.gain = SIGO_U_GAIN;
    spll_obj->sogi_u_coeff.B0 = SIGO_U_B0;
    spll_obj->sogi_u_coeff.B1 = SIGO_U_B1;
    spll_obj->sogi_u_coeff.B2 = SIGO_U_B2;
    spll_obj->sogi_u_coeff.A1 = SIGO_U_A1;
    spll_obj->sogi_u_coeff.A2 = SIGO_U_A2;

    spll_obj->sogi_qu_coeff.gain = SIGO_QU_GAIN;
    spll_obj->sogi_qu_coeff.B0 = SIGO_QU_B0;
    spll_obj->sogi_qu_coeff.B1 = SIGO_QU_B1;
    spll_obj->sogi_qu_coeff.B2 = SIGO_QU_B2;
    spll_obj->sogi_qu_coeff.A1 = SIGO_QU_A1;
    spll_obj->sogi_qu_coeff.A2 = SIGO_QU_A2;

    // Initialize PLL controller using ARM PID
    float32_t pidKp = ssrf_kp;
    float32_t pidKi = sser_ki;
    float32_t pidKd = 0.0f;

    spll_obj->pid.Kp = pidKp;
    spll_obj->pid.Ki = pidKi;
    spll_obj->pid.Kd = pidKd;
    arm_pid_init_f32(&spll_obj->pid, 1);

    spll_obj->spp_freq_max_limt = ssrf_up_limt;
    spll_obj->spll_freq_min_limt = ssrf_low_limt;

    // Initialize other variables
    spll_obj->theta = 0.0f;
    spll_obj->cos_theta = 1.0f;
    spll_obj->sin_theta = 0.0f;
    spll_obj->spll_integrator = 0.0f;
}

float32_t discrete_2order_tf(const float32_t input, DIS_2ORDER_TF_COEF_DEF *coeff, DIS_2ORDER_TF_DATA_DEF *data)
{
    // w0 = x(0) - A1 * W1 - A2 * W2
    data->w0 = input - coeff->A1 * data->w1 - coeff->A2 * data->w2;

    // Y(0) = Gain * (B0 * W0 + B1 * W1 + B2 * W2)
    data->output = coeff->gain * (coeff->B0 * data->w0 + coeff->B1 * data->w1 + coeff->B2 * data->w2);

    data->w2 = data->w1;
    data->w1 = data->w0;

    return data->output;
}
void spll_sogi_func(SOGI_PLL_DATA_DEF *spll_obj, float32_t grid_volt_sen)
{
    // Signal generator
    spll_obj->ac_u = discrete_2order_tf(grid_volt_sen, &(spll_obj->sogi_u_coeff), &(spll_obj->sogi_u_data));
    spll_obj->ac_qu = discrete_2order_tf(grid_volt_sen, &(spll_obj->sogi_qu_coeff), &(spll_obj->sogi_qu_data));

    // Park transform from alpha beta to d-q axis
    spll_obj->u_d = spll_obj->ac_u * spll_obj->cos_theta + spll_obj->ac_qu * spll_obj->sin_theta;
    spll_obj->u_q = -spll_obj->ac_u * spll_obj->sin_theta + spll_obj->ac_qu * spll_obj->cos_theta;

    // PI control q to 0 - using ARM PID function
    spll_obj->spll_diff = 0.0f - spll_obj->u_d;
    spll_obj->pll_freq_out = arm_pid_f32(&spll_obj->pid, spll_obj->spll_diff);

    // Update theta
    spll_obj->theta -= (spll_obj->pll_freq_out + 0.5f) * value_2pi * spll_obj->delta_t;

    // Normalize theta to [0, 2π]
    if (spll_obj->theta > value_2pi) {
        spll_obj->theta -= value_2pi;
    } else if (spll_obj->theta < 0) {
        spll_obj->theta += value_2pi;
    }

    // Update trigonometric values using ARM optimized functions
    arm_sin_cos_f32(spll_obj->theta * (180.0f / PI), &(spll_obj->sin_theta), &(spll_obj->cos_theta));
}

// spll_sogi_func(&spll_data, vac_sen);

/*
 * 原文件中与func关系协作完成 while1
 */
// void run_spll_arm_f32(void)
// {
//     // float32_t vac_sen   = InputSignal(0, 0);
//     // float32_t deta_theta = arm_cos_f32(InputSignal(0, 1));
//
//     spll_sogi_func_(&spll_data, vac_sen);   // 内部全部 float32
//
     // float32_t temp_theta = spll_data.theta - deta_theta;
//
//     /* 限幅到 0 .. 2π */
//     while (temp_theta < 0.0f)         temp_theta += 2.0f * value_2pi;
//     while (temp_theta > 2.0f * value_2pi)  temp_theta -= 2.0f * value_2pi;
//
//     // OutputSignal(0, 0) = temp_theta;
//     // OutputSignal(0, 1) = spll_data.pll_freq_out;
//     // OutputSignal(0, 2) = spll_data.spll_diff;
// }
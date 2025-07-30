#include "arm_math.h"
#include "triple_pll.h"
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

    // spll_obj->pid.Kp = pidKp;
    // spll_obj->pid.Ki = pidKi;
    // spll_obj->pid.Kd = pidKd;
    // arm_pid_init_f32(&spll_obj->pid, 1);

    spll_obj->spp_freq_max_limt = ssrf_up_limt;
    spll_obj->spll_freq_min_limt = ssrf_low_limt;

    /* α 通道 */
    spll_obj->sogi_alpha_coeff = spll_obj->sogi_u_coeff;   // 直接复制即可
    /* β 通道 */
    spll_obj->sogi_beta_coeff  = spll_obj->sogi_u_coeff;

    // Initialize other variables
    // spll_obj->theta = 0.0f;
    // spll_obj->cos_theta = 1.0f;
    // spll_obj->sin_theta = 0.0f;
    // spll_obj->spll_integrator = 0.0f;
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
void spll_sogi_func(SOGI_PLL_DATA_DEF *spll_obj, float32_t va, float32_t vb, float32_t vc)
{
    /* 1. Clarke 变换 -> α, β */
    float valpha = va;
    float vbeta  = (vb - vc) * 0.57735;   // 1/√3 ≈ 0.57735
    /* 2. SOGI 产生正交信号 */
    float u_alpha = discrete_2order_tf(valpha,
                          &spll_obj->sogi_alpha_coeff,
                          &spll_obj->sogi_alpha_data);
    float u_beta  = discrete_2order_tf(vbeta,
                          &spll_obj->sogi_beta_coeff,
                          &spll_obj->sogi_beta_data);

    /* 3. Park 变换（用 SOGI 输出作为 αβ，再转到 dq） */
    spll_obj->u_d =  u_alpha * spll_obj->cos_theta
                   + u_beta  * spll_obj->sin_theta;
    spll_obj->u_q = -u_alpha * spll_obj->sin_theta
                   + u_beta  * spll_obj->cos_theta;

    // /* 4. 以下 PLL 部分完全不用改 */
    spll_obj->spll_diff = 0.0f - spll_obj->u_q;   // 让 u_q → 0

    spll_obj->spll_integrator += spll_obj->spll_diff * spll_obj->spll_ki;

    spll_obj->pll_freq_out = spll_obj->spll_diff * spll_obj->spll_kp + spll_obj->spll_integrator;

    /* 5. 角度更新 & 归一化 */
    spll_obj->theta += spll_obj->pll_freq_out * value_2pi * spll_obj->delta_t;
    // Normalize theta to [0, 2π]
    if (spll_obj->theta > value_2pi)    spll_obj->theta -= value_2pi;
    else if (spll_obj->theta < 0)   spll_obj->theta += value_2pi;

    // // pi ctrol q to 0
    // spll_obj->spll_diff = 0 - spll_obj->u_d;

    // spll_obj->theta -= (spll_obj->pll_freq_out+0.5f) * value_2pi * spll_obj->delta_t;

    spll_obj->cos_theta = cos(spll_obj->theta);
    spll_obj->sin_theta = sin(spll_obj->theta);

}
/*
 * 原文件中与func关系协作完成 while1
 */

void park_transform(float32_t xa, float32_t xb, float32_t xc, float theta, PARK_TRANS *res)
{
    res->yd = (2.0/3.0)* (arm_cos_f32(theta)*xa + arm_cos_f32(theta - 2/3*M_PI)*xb + arm_cos_f32(theta + 2/3*M_PI)*xc);
    res->yq = (2.0/3.0)* (-arm_sin_f32(theta)*xa - sin(theta-2/3*M_PI)*xb - sin(theta+2/3*M_PI)*xc);
}
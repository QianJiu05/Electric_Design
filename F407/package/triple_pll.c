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
    float32_t spll_kp = ssrf_kp;
    float32_t spll_ki = sser_ki;

    spll_obj->spp_freq_max_limt = ssrf_up_limt;
    spll_obj->spll_freq_min_limt = ssrf_low_limt;

    /* α 通道 */
    spll_obj->sogi_alpha_coeff = spll_obj->sogi_u_coeff;   // 直接复制即可
    /* β 通道 */
    spll_obj->sogi_beta_coeff  = spll_obj->sogi_u_coeff;
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

    // /* 4. PLL  */
    spll_obj->spll_diff = 0.0f - spll_obj->u_q;   // 让 u_q → 0

    spll_obj->spll_integrator += spll_obj->spll_diff * spll_obj->spll_ki;

    spll_obj->pll_freq_out = spll_obj->spll_diff * spll_obj->spll_kp + spll_obj->spll_integrator;

    /* 5. 角度更新 & 归一化 */
    spll_obj->theta -= spll_obj->pll_freq_out * value_2pi * spll_obj->delta_t;//+?-?
    // Normalize theta to [0, 2π]
    if (spll_obj->theta > value_2pi)    spll_obj->theta -= value_2pi;
    else if (spll_obj->theta < 0)   spll_obj->theta += value_2pi;

    spll_obj->cos_theta = arm_cos_f32(spll_obj->theta);
    spll_obj->sin_theta = arm_sin_f32(spll_obj->theta);
}
DQ_Components park_transform(float32_t a, float32_t b, float32_t c, float theta)
{
    // 预先计算120度和240度偏移角度
    float sin_theta = arm_sin_f32(theta);
    float cos_theta = arm_cos_f32(theta);

    float32_t sqrt_3_over_2;
    arm_sqrt_f32(3.0f,&sqrt_3_over_2);
    sqrt_3_over_2 /= 2.0f;

    DQ_Components dq;

    dq.d=2.0f/3.0f*(a*cos_theta+b*(-0.5*cos_theta+sqrt_3_over_2*sin_theta)+c*(-0.5*cos_theta-sqrt_3_over_2*sin_theta));
    dq.q = 2.0f/3.0f*(-a*sin_theta+b*(0.5*sin_theta+sqrt_3_over_2*cos_theta)+c*(0.5*sin_theta-sqrt_3_over_2*cos_theta));

    return dq;
}

REVER_Components inverse_park_transform(float32_t d, float32_t q, float theta)
{
    // 预先计算120度和240度偏移角度
    float sin_theta = arm_sin_f32(theta);
    float cos_theta = arm_cos_f32(theta);

    float32_t sqrt_3_over_2;
    arm_sqrt_f32(3.0f, &sqrt_3_over_2);
    sqrt_3_over_2 /= 2.0f;

    REVER_Components rvt;

    rvt.a = d * cos_theta - q * sin_theta;
    rvt.b = d * (-0.5 * cos_theta + sqrt_3_over_2 * sin_theta) + q * (-0.5 * sin_theta - sqrt_3_over_2 * cos_theta);
    rvt.c = d * (-0.5 * cos_theta - sqrt_3_over_2 * sin_theta) + q * (-0.5 * sin_theta + sqrt_3_over_2 * cos_theta);

    return rvt;
}
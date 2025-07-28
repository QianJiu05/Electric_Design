
// #include "math.h"直接替换成arm math
#include "arm_math.h"
#include "singlephase_rectifier.h"
/* SCB->CPACR |= (0xFUL << 20);   /* 使能 CP10/CP11，开启FPU */
#define  float float

/*
*   如果还想再压榨 10 %，可把 arm_sin_f32 / arm_cos_f32 换成查表 + 线性插值（自己建 256 点表即可）。
    若使用双通道 SOGI（锁三相），可把 Biquad 改成 arm_biquad_cascade_df1_f32 的 2-stage 版，一次处理 2 路信号，减少函数调用开销。
    记得把 value_2pi 改成 6.283185307f，避免 2.0f*3.1415926 每次乘法。
 *
 */
#define FSW 20e3

#define ssrf_ts             1/FSW
#define ssrf_kp             1.007f
#define sser_ki             0.006f
#define ssrf_up_limt        63
#define ssrf_low_limt       43
// #define value_2pi           2.0f * 3.1415926
#define value_2pi           6.283185307f

//[0.003405429629029,0,-0.003405429629029]
//[0.010982881827120,0,-0.010982881827120]
#define SIGO_U_GAIN  1
#define SIGO_U_B0    0.010982881827120
#define SIGO_U_B1    0
#define SIGO_U_B2   -0.010982881827120

//[1,-1.993165860418354,0.993189140741942]
//[1,-1.977790221205283,0.978034236345759]
#define SIGO_U_A1   -1.977790221205283
#define SIGO_U_A2    0.978034236345759

//qu coeff [8.229594388365577e-06,1.645918877673115e-05,8.229594388365577e-06]
//[8.625935215831610e-05,1.725187043166322e-04,8.625935215831610e-05]
#define SIGO_QU_GAIN  1.0e-03
#define SIGO_QU_B0    0.08625935215831610
#define SIGO_QU_B1    0.1725187043166322
#define SIGO_QU_B2    0.08625935215831610

//q coef  [1,-1.993165860418354,0.993189140741942]
//[1,-1.977790221205283,0.978034236345759]
#define SIGO_QU_A1    -1.977790221205283
#define SIGO_QU_A2     0.978034236345759

//
//Const data define
//


// SOGI_PLL_DATA_DEF spll_data;

SOGI_BQ_DEF sogi_u, sogi_qu;


static inline void sogi_bq_init(SOGI_BQ_DEF *s,
                                float32_t b0, float32_t b1, float32_t b2,
                                float32_t a1, float32_t a2)
{
    s->coeffs[0] = b0; s->coeffs[1] = b1; s->coeffs[2] = b2;
    s->coeffs[3] = -a1; s->coeffs[4] = -a2;   // 注意符号
    arm_biquad_cascade_df1_init_f32(&s->bq_inst, 1, s->coeffs, s->state);
}

void sogi_pll_init(void)
{
    // ...
    /* SOGI-U */
    sogi_bq_init(&sogi_u,
                 SIGO_U_B0, SIGO_U_B1, SIGO_U_B2,
                 SIGO_U_A1, SIGO_U_A2);
    /* SOGI-QU */
    sogi_bq_init(&sogi_qu,
                 SIGO_QU_B0, SIGO_QU_B1, SIGO_QU_B2,
                 SIGO_QU_A1, SIGO_QU_A2);
}


static inline float32_t sogi_bq_run(SOGI_BQ_DEF *s, float32_t in)
{
    float32_t out;
    arm_biquad_cascade_df1_f32(&s->bq_inst, &in, &out, 1);
    return out;
}

void spll_sogi_func(SOGI_PLL_DATA_DEF *s, float grid)
{
    s->ac_u  = sogi_bq_run(&sogi_u,  grid);
    s->ac_qu = sogi_bq_run(&sogi_qu, grid);

    /* Park 变换 */
    s->u_d =  s->ac_u * arm_cos_f32(s->theta) + s->ac_qu * arm_sin_f32(s->theta);
    s->u_q = -s->ac_u * arm_sin_f32(s->theta) + s->ac_qu * arm_cos_f32(s->theta);

    /* PI 控制 */
    s->spll_diff = -s->u_q;
    s->spll_integrator += s->spll_diff * s->spll_ki;
    s->pll_freq_out = s->spll_diff * s->spll_kp + s->spll_integrator;

    /* 相位积分 */
    s->theta -= (s->pll_freq_out + 0.5f) * value_2pi * s->delta_t;
    if (s->theta > value_2pi) s->theta -= value_2pi;
    else if (s->theta < 0)       s->theta += value_2pi;
}
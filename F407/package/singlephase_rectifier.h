//
// Created by 32114 on 25-7-27.
//

#ifndef SINGLEPHASE_RECTIFIER_H
#define SINGLEPHASE_RECTIFIER_H
typedef struct DIS_2ORDER_TF_COEF_TAG {
    float gain;
    float B0;
    float B1;
    float B2;
    float A1;
    float A2;
}DIS_2ORDER_TF_COEF_DEF;

typedef struct DIS_2ORDER_TF_DATA_TAG {
    float output;
    float w0;
    float w1;
    float w2;
}DIS_2ORDER_TF_DATA_DEF;


//
// SOGI PLL date struct
//
typedef struct SOGI_PLL_DATA_STRUCT_TAG
{
    float ac_u;
    float ac_qu;
    float u_q;
    float u_d;
    float theta;
    float theta_1;
    float cos_theta;
    float sin_theta;
    float grid_freq;
    float pll_freq_out;
    float delta_t;
    float spll_diff;

    DIS_2ORDER_TF_COEF_DEF  sogi_u_coeff;
    DIS_2ORDER_TF_DATA_DEF  sogi_u_data;

    DIS_2ORDER_TF_COEF_DEF  sogi_qu_coeff;
    DIS_2ORDER_TF_DATA_DEF  sogi_qu_data;

    float spll_kp;
    float spll_ki;
    float spll_integrator;
    float spll_freq_min_limt;
    float spp_freq_max_limt;
}SOGI_PLL_DATA_DEF;

typedef struct {
    arm_biquad_casd_df1_inst_f32 bq_inst;  // 官方实例
    float32_t coeffs[5];                   // {b0,b1,b2,-a1,-a2}
    float32_t state[4];                    // 4 个状态变量
} SOGI_BQ_DEF;

// void sogi_pll_init(SOGI_PLL_DATA_DEF *spll_obj, float grid_freq, float ts);

void sogi_pll_init(void);
void spll_sogi_func(SOGI_PLL_DATA_DEF *s, float grid);




#endif //SINGLEPHASE_RECTIFIER_H

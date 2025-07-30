//
// Created by 32114 on 25-7-27.
//

#ifndef TRIPLE_PLL_H
#define TRIPLE_PLL_H

#define FSW 20000.0f

#define ssrf_ts             (1.0f/FSW)
#define ssrf_kp             0.9f
#define sser_ki             0.0001f
#define ssrf_up_limt        63
#define ssrf_low_limt       43
#define value_2pi           6.283185307f

#define SIGO_U_GAIN  1
#define SIGO_U_B0    0.010982881827120
#define SIGO_U_B1    0
#define SIGO_U_B2   -0.010982881827120

#define SIGO_U_A1   -1.977790221205283
#define SIGO_U_A2    0.978034236345759

// #define SIGO_QU_GAIN  1.0e-03
#define SIGO_QU_GAIN  0.001
#define SIGO_QU_B0    0.08625935215831610
#define SIGO_QU_B1    0.1725187043166322
#define SIGO_QU_B2    0.08625935215831610

#define SIGO_QU_A1    -1.977790221205283
#define SIGO_QU_A2     0.978034236345759

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

    DIS_2ORDER_TF_DATA_DEF  sogi_alpha_data;
    DIS_2ORDER_TF_DATA_DEF  sogi_beta_data;
    DIS_2ORDER_TF_COEF_DEF  sogi_alpha_coeff;
    DIS_2ORDER_TF_COEF_DEF  sogi_beta_coeff;

    float spll_kp;
    float spll_ki;
    float spll_integrator;
    float spll_freq_min_limt;
    float spp_freq_max_limt;

}SOGI_PLL_DATA_DEF;

//
//  park struct
//
typedef struct park_trans {
    float32_t yd;
    float32_t yq;
}PARK_TRANS;

void sogi_pll_init(SOGI_PLL_DATA_DEF *spll_obj, float32_t grid_freq, float32_t ts);
void spll_sogi_func(SOGI_PLL_DATA_DEF *spll_obj, float32_t va, float32_t vb, float32_t vc);

void park_transform(float32_t xa, float32_t xb, float32_t xc, float theta, PARK_TRANS *res);



#endif //SINGLEPHASE_RECTIFIER_H

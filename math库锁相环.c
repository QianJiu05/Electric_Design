
#include "math.h"
#include "arm_math.h"
#include "singlephase_rectifier.h"
/* SCB->CPACR |= (0xFUL << 20);   /* 使能 CP10/CP11，开启FPU */
#define  float float

#define FSW 20e3

#define ssrf_ts             1/FSW
#define ssrf_kp             1.007f
#define sser_ki             0.006f
#define ssrf_up_limt        63
#define ssrf_low_limt       43
#define value_2pi           2.0f * 3.1415926

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


SOGI_PLL_DATA_DEF spll_data;

// Function description :
void sogi_pll_init(SOGI_PLL_DATA_DEF *spll_obj, float grid_freq, float ts)
{
    spll_obj->grid_freq = grid_freq;
    spll_obj->delta_t = ts;

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

    spll_obj->spll_kp = ssrf_kp;
    spll_obj->spll_ki = sser_ki;
    spll_obj->spp_freq_max_limt = ssrf_up_limt;
    spll_obj->spll_freq_min_limt = ssrf_low_limt;
}

//sogi_pll_init(&spll_data, 50.0, ssrf_ts);


float  discrete_2order_tf(const float input, DIS_2ORDER_TF_COEF_DEF *coeff, DIS_2ORDER_TF_DATA_DEF *data)
{
    // w0 = x(0) - A1 * W1 - A2 * W2
    data->w0 = input - coeff->A1 * data->w1 - coeff->A2 * data->w2;

    // Y(0) = Gain * (B0 * W0 + B1 * W1 + B2 * W2)
    data->output = coeff->gain * (coeff->B0 * data->w0 + coeff->B1 * data->w1 + coeff->B2 * data->w2);

    data->w2 = data->w1;
    data->w1 = data->w0;

    return(data->output);
}


// sogo pll
void spll_sogi_func(SOGI_PLL_DATA_DEF *spll_obj, float grid_volt_sen)
{
    // signal genertor
    spll_obj->ac_u = discrete_2order_tf(grid_volt_sen, &(spll_obj->sogi_u_coeff), &(spll_obj->sogi_u_data));
    spll_obj->ac_qu = discrete_2order_tf(grid_volt_sen, &(spll_obj->sogi_qu_coeff), &(spll_obj->sogi_qu_data));

    // Park transfrom from alpha beta tp d-q axis
    spll_obj->u_d =  spll_obj->ac_u * spll_obj->cos_theta + spll_obj->ac_qu * spll_obj->sin_theta;
    spll_obj->u_q = -spll_obj->ac_u * spll_obj->sin_theta + spll_obj->ac_qu * spll_obj->cos_theta;

    // pi ctrol q to 0
    spll_obj->spll_diff = 0 - spll_obj->u_d;

    spll_obj->spll_integrator += spll_obj->spll_diff * spll_obj->spll_ki;

    spll_obj->pll_freq_out = spll_obj->spll_diff * spll_obj->spll_kp + spll_obj->spll_integrator;

    spll_obj->theta -= (spll_obj->pll_freq_out+0.5f) * value_2pi * spll_obj->delta_t;

    if(spll_obj->theta > value_2pi)
    {
        spll_obj->theta -= value_2pi;
    }
    else if (spll_obj->theta < 0)
    {
        spll_obj->theta += value_2pi;
    }

    spll_obj->cos_theta = cos(spll_obj->theta);
    spll_obj->sin_theta = sin(spll_obj->theta);
}

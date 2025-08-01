
/*declartion*/
#include "math.h"
#define float float
//  park struct
typedef struct {
    double d;  // D轴分量（直轴）
    double q;  // Q轴分量（交轴）
} DQ_Components;

DQ_Components park_transform(float a, float b, float c, float theta)
{
    // 预先计算120度和240度偏移角度
    float sin_theta = sin(theta);
    float cos_theta = cos(theta);

    float sqrt_3_over_2;
    arm_sqrt_f32(3.0f,&sqrt_3_over_2);
    sqrt_3_over_2 /= 2.0f;

    DQ_Components dq;

    dq.d=2.0f/3.0f*(a*cos_theta+b*(-0.5*cos_theta+sqrt_3_over_2*sin_theta)+c*(-0.5*cos_theta-sqrt_3_over_2*sin_theta));
    dq.q = 2.0f/3.0f*(-a*sin_theta+b*(0.5*sin_theta+sqrt_3_over_2*cos_theta)+c*(0.5*sin_theta-sqrt_3_over_2*cos_theta));

    return dq;
}
//reverse park 
typedef struct {
    float a;
    float b;
    float c;
} REVER_Components;

REVER_Components inverse_park_transform(float d, float q, float theta)
{
    // 预先计算120度和240度偏移角度
    float sin_theta = sin(theta);
    float cos_theta = cos(theta);

    float sqrt_3_over_2;
    arm_sqrt_f32(3.0f, &sqrt_3_over_2);
    sqrt_3_over_2 /= 2.0f;

    REVER_Components rvt;

    rvt.a = d * cos_theta - q * sin_theta;
    rvt.b = d * (-0.5 * cos_theta + sqrt_3_over_2 * sin_theta) + q * (-0.5 * sin_theta - sqrt_3_over_2 * cos_theta);
    rvt.c = d * (-0.5 * cos_theta - sqrt_3_over_2 * sin_theta) + q * (-0.5 * sin_theta + sqrt_3_over_2 * cos_theta);

    return rvt;
}
//pid
typedef struct {
    float Kp;
    float Ki;
    // float Kd;
    float setpoint;
    float integral;
} PID_TypeDef;

float PID_Calc(PID_TypeDef *pid, float feedback)
{
    float error = pid->setpoint - feedback;
    pid->integral += error;
    // pid->last_error = error;
    float output = pid->Kp * error + pid->Ki * pid->integral;
    // 限幅
    // if (output > MAX_OUTPUT) output = MAX_OUTPUT;
    // if (output < MIN_OUTPUT) output = MIN_OUTPUT;
    return output;
}

/* init */
PID_TypeDef pid1_d = {0.2,10,0,0};
PID_TypeDef pid1_q = {0.2,10,0,0};
PID_TypeDef pid2_d = {40,400,32,0};
PID_TypeDef pid2_q = {40,400,32,0};


/*function*/

float Vd_ref,Vq_ref,wt,Vload_a,Vload_b,Vload_c,linv_a,linv_b,linv_c,Vdc;
float theta;



float output1_d,output1_q,output2_d,output2_q;
float ref_d1,ref_q1,ref_d2,ref_q2;
DQ_Components DQ_Vload,DQ_linv;

REVER_Components rever;

// Vd_ref = InputSignal(0,1);
// Vq_ref = InputSignal(0,2);
// Vload_a = InputSignal(0,3);
// Vload_b = InputSignal(0,4);
// Vload_c = InputSignal(0,5);
// theta = InputSignal(0,6);


PID_TypeDef pid1_d = {0.2,10,0,0};
PID_TypeDef pid1_q = {0.2,10,0,0};
PID_TypeDef pid2_d = {40,400,32,0};
PID_TypeDef pid2_q = {40,400,32,0};

//level 1 pid 
DQ_Vload = park_transform(Vload_a,Vload_b,Vload_c,theta);
ref_d1 = Vd_ref - DQ_Vload.d;
ref_q1 = Vq_ref - DQ_Vload.q;

output1_d = PID_Calc(&pid1_d,ref_d1);
output1_q = PID_Calc(&pid1_q,ref_q1);

if(output1_d > 40){
    output1_d = 40;
}else if (output1_d < -40){
    output1_d = -40;
}
if(output1_q > 40){
    output1_q = 40;
}else if (output1_q < -40){
    output1_q = -40;
}


//level 2 pid
DQ_linv = park_transform(linv_a,linv_b,linv_c,theta);
ref_d2  = output1_d - DQ_linv.d;
ref_q2 = output1_q - DQ_linv.q;

 output2_d = PID_Calc(&pid2_d,ref_d2);
 output2_q = PID_Calc(&pid2_q,ref_q2);

if(output2_d > 400){
    output2_d = 400;
}else if (output2_d < -400){
    output2_d = -400;
}
if(output2_q > 400){
    output2_q = 400;
}else if (output2_q < -400){
    output2_q = -400;
}

//output to pwm
rever = inverse_park_transform(output2_d,output2_q,theta);

rever.a *= 2;
rever.b *=2;
rever.c *=2;
rever.a /= Vdc;
rever.b /= Vdc;
rever.c /= Vdc;
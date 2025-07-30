#include "math.h"
struct to_trans{
    float VoltAlpha;
    float VoltBeta;
    float VoltB;
    float VoltC;
    float VoltD;
    float VoltQ;

}
struct to_trans trans;

float CosWT,SinWT;

struct pid{
    float Ref;
    float Fed;
    float Err;
    float OUTValue;
    float ErrPre;
    float sf;
}
struct pid control_PI;

//Clark变换
trans.VoltAlpha = (trans.VoltA-(0.5*trans.VoltB)-(0.5*trans.VoltC));//cos(pi*(2/3))=0.5
trans.VoltBeta  = ((0.866*trans.VoltB)-(0.866*trans.VoltC));     //sin(pi*(2/3))=0.866

//Park变换
trans.VoltD =  CosWT * trans.VoltAlpha + SinWT * trans.VoltBeta;
trans.VoltQ = -SinWT * trans.VoltAlpha + CosWT * trans.VoltBeta;

//增量式PI调节
control_PI.Ref = trans.VoltD;//VoltD
control_PI.Fed = 0;
control_PI.Err = control_PI.Ref - control_PI.Fed;
control_PI.OUTValue = control_PI.Ki * control_PI.Err;/* 积分计算  */
control_PI.OUTValue += control_PI.Kp * (control_PI.Err - control_PI.ErrPre);//比例
Out_W += control_PI.OUTValue;/* 增量运算 */
control_PI.ErrPre = control_PI.Err;

OUT_WT += (Out_W / control_PI.sf);//环路输出即为w，T需要积分
SinWT= sinf(OUT_WT);
CosWT= cosf(OUT_WT);
if(OUT_WT>2*pi){
    OUT_WT=OUT_WT-2*pi;
}
sinwt[0] = SinWT;
coswt[0] = CosWT;
Freq[0]  = Out_W/(2*pi);
wt[0]    = OUT_WT;
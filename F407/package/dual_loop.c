// #include "dual_loop.h"
//
// #include <math.h>
//
// #include "arm_math.h"
// #include "rtwtypes.h"
// #include "VCO.h"
// // #include "math.h"
// /* Block signals and states (default storage) for system '<Root>' */
// typedef struct {
//   real_T Integrator_DSTATE;            /* '<S38>/Integrator' */
// } DW;
// /* Block signals and states (default storage) */
// DW rtDW;
//
// /* External inputs (root inport signals with default storage) */
// ExtU rtU;
//
// /* External outputs (root outports fed by signals with default storage) */
// ExtY rtY;
//
// /* Real-time model */
// static RT_MODEL rtM_;
// RT_MODEL *const rtM = &rtM_;
//
// extern real_T rt_hypotd_snf(real_T u0, real_T u1);
// static real_T rtGetNaN(void);
// static real32_T rtGetNaNF(void);
// extern real_T rtInf;
// extern real_T rtMinusInf;
// extern real_T rtNaN;
// extern real32_T rtInfF;
// extern real32_T rtMinusInfF;
// extern real32_T rtNaNF;
// static boolean_T rtIsInf(real_T value);
// static boolean_T rtIsInfF(real32_T value);
// static boolean_T rtIsNaN(real_T value);
// static boolean_T rtIsNaNF(real32_T value);
// real_T rtNaN = -(real_T)NAN;
// real_T rtInf = (real_T)INFINITY;
// real_T rtMinusInf = -(real_T)INFINITY;
// real32_T rtNaNF = -(real32_T)NAN;
// real32_T rtInfF = (real32_T)INFINITY;
// real32_T rtMinusInfF = -(real32_T)INFINITY;
//
// /* Return rtNaN needed by the generated code. */
// static real_T rtGetNaN(void)
// {
//   return rtNaN;
// }
//
// /* Return rtNaNF needed by the generated code. */
// static real32_T rtGetNaNF(void)
// {
//   return rtNaNF;
// }
//
// /* Test if value is infinite */
// static boolean_T rtIsInf(real_T value)
// {
//   return (boolean_T)isinf(value);
// }
//
// /* Test if single-precision value is infinite */
// static boolean_T rtIsInfF(real32_T value)
// {
//   return (boolean_T)isinf(value);
// }
//
// /* Test if value is not a number */
// static boolean_T rtIsNaN(real_T value)
// {
//   return (boolean_T)(isnan(value) != 0);
// }
//
// /* Test if single-precision value is not a number */
// static boolean_T rtIsNaNF(real32_T value)
// {
//   return (boolean_T)(isnan(value) != 0);
// }
// /*
//  * 获取根号下（u0方+u1方）
//  */
// real_T rt_hypotd_snf(real_T u0, real_T u1)
// {
//   real_T a;
//   real_T b;
//   real_T y;
//   real_T a_sqrt;
//   real_T b_sqrt;
//
//   arm_abs_f32(&u0,&a,1);
//   arm_abs_f32(&u1,&b,1);
//
//   // b = fabs(u1);
//   if (a < b) {
//     a /= b;
//     arm_sqrt_f32((a*a+1.0f),&a_sqrt);
//     y = a_sqrt * b;
//     // y = sqrt(a * a + 1.0) * b;
//   } else if (a > b) {
//     b /= a;
//     arm_sqrt_f32((b*b+1.0f),&b_sqrt);
//     y = b_sqrt * a;
//     // y = sqrt(b * b + 1.0) * a;
//   } else if (rtIsNaN(b)) {
//     y = (rtNaN);
//   } else {
//     y = a * 1.4142135623730951f;
//   }
//   return y;
// }
//
// /* Model step function */
// void dual_step(void)
// {
//   real_T rtb_Add;
//   real_T rtb_Sum;
//
//   /* Sum: '<S1>/Add' incorporates:
//    *  Fcn: '<S2>/alpha'
//    *  Fcn: '<S2>/beta'
//    *  Gain: '<S2>/K'
//    *  Inport: '<Root>/Vabc'
//    *  Inport: '<Root>/vref'
//    *  Math: '<S1>/Hypot'
//    */
//   rtb_Add = rtU.output - rt_hypotd_snf(((rtU.Vabc[0] - 0.5f * rtU.Vabc[1]) - 0.5f *rtU.Vabc[2]) * 0.66666666666666663f,
//                                             (rtU.Vabc[1] - rtU.Vabc[2]) *1.7320508075688772 / 2.0 * 0.66666666666666663);
//
//   /* Sum: '<S47>/Sum' incorporates:
//    *  DiscreteIntegrator: '<S38>/Integrator'
//    */
//   rtb_Sum = rtb_Add + rtDW.Integrator_DSTATE;
//
//   /* Outport: '<Root>/abc' incorporates:
//    *  Constant: '<S1>/Constant'
//    *  Inport: '<Root>/theta'
//    *  Product: '<S1>/Product'
//    *  Sum: '<S1>/Add1'
//    *  Trigonometry: '<S1>/Sin'
//    */
//   rtY.abc[0] = rtb_Sum * sin(rtU.theta);
//   rtY.abc[1] = sin(rtU.theta - 2.0943951023931953f) * rtb_Sum;
//   rtY.abc[2] = sin(rtU.theta + 2.0943951023931953f) * rtb_Sum;
//
//   /* Update for DiscreteIntegrator: '<S38>/Integrator' incorporates:
//    *  Gain: '<S35>/Integral Gain'
//    */
//   rtDW.Integrator_DSTATE += 5.0f * rtb_Add * 0.0001f;
// }
//
// /* Model initialize function */
// void dual_initialize(void)
// {
//   /* (no initialization code required) */
// }
// ////////////func in main
// ///
// ///
//
// void rt_OneStep(void);
// void rt_OneStep(void)
// {
//   static boolean_T OverrunFlag = false;
//
//   /* Disable interrupts here */
//
//   /* Check for overrun */
//   if (OverrunFlag) {
//     rtmSetErrorStatus(rtM, "Overrun");
//     return;
//   }
//
//   OverrunFlag = true;
//
//   /* Save FPU context here (if necessary) */
//   /* Re-enable timer or interrupt here */
//   /* Set model inputs here */
//
//   /* Step the model */
//   dual_step();
//
//   /* Get model outputs here */
//
//   /* Indicate task complete */
//   OverrunFlag = false;
//
//   /* Disable interrupts here */
//   /* Restore FPU context here (if necessary) */
//   /* Enable interrupts here */
// }
//
// /*
//  * The example main function illustrates what is required by your
//  * application code to initialize, execute, and terminate the generated code.
//  * Attaching rt_OneStep to a real-time clock is target specific. This example
//  * illustrates how you do this relative to initializing the model.
//  */
// // int_T main(void)
// // {
// //   /* Initialize model */
// //   dual_initialize();
// //
// //   /* Attach rt_OneStep to a timer or interrupt service routine with
// //    * period 0.0001 seconds (base rate of the model) here.
// //    * The call syntax for rt_OneStep is
// //    *
// //    *  rt_OneStep();//放在timer或者中断里面
// //    */
// //   return 0;
// // }
//

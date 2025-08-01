// #include "VCO.h"
// #include "rtwtypes.h"
//
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
// /* Model step function */
// void VCO_step(void)
// {
//     real_T rtb_Gain4;
//     real_T rtb_Sum6;
//
//     /* Gain: '<S1>/Gain4' incorporates:
//      *  Inport: '<Root>/w0'
//      */
//     rtb_Gain4 = 0.15915494309189535f * rtU.w0;
//
//     /* Sum: '<S1>/Sum6' incorporates:
//      *  Gain: '<S1>/Ts_PLL'
//      *  UnitDelay: '<S1>/Unit Delay'
//      */
//     rtb_Sum6 = 0.0001f * rtb_Gain4 + rtDW.UnitDelay_DSTATE;
//
//     /* Switch: '<S1>/Switch' incorporates:
//      *  Constant: '<S1>/Constant2'
//      *  Sum: '<S1>/Sum7'
//      */
//     if (rtb_Sum6 >= 1.0f) {
//         rtb_Sum6--;
//     }
//
//     /* End of Switch: '<S1>/Switch' */
//
//     /* Outport: '<Root>/theta' incorporates:
//      *  Gain: '<S1>/Gain5'
//      */
//     rtY.theta = 6.28f * rtb_Sum6;
//
//     /* Outport: '<Root>/f' */
//     rtY.f = rtb_Gain4;
//
//     /* Update for UnitDelay: '<S1>/Unit Delay' */
//     rtDW.UnitDelay_DSTATE = rtb_Sum6;
// }
//
// /* Model initialize function */
// // void VCO_initialize(void)
// // {
// //     /* (no initialization code required) */
// // }
//
//
// ///////// func
// ////*
// #include <stddef.h>
// #include <stdio.h>            /* This example main program uses printf/fflush */
// #include "VCO.h"                      /* Model header file */
//
// /*
//  * Associating rt_OneStep with a real-time clock or interrupt service routine
//  * is what makes the generated code "real-time".  The function rt_OneStep is
//  * always associated with the base rate of the model.  Subrates are managed
//  * by the base rate from inside the generated code.  Enabling/disabling
//  * interrupts and floating point context switches are target specific.  This
//  * example code indicates where these should take place relative to executing
//  * the generated code step function.  Overrun behavior should be tailored to
//  * your application needs.  This example simply sets an error status in the
//  * real-time model and returns from rt_OneStep.
//  */
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
//   VCO_step();
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
// // int_T main()
// // {
//   /* Initialize model */
//   // VCO_initialize();
//
//   /* Attach rt_OneStep to a timer or interrupt service routine with
//    * period 0.0001 seconds (base rate of the model) here.
//    * The call syntax for rt_OneStep is
//    *
//    *  rt_OneStep();
//    */
//
// // }
//

//
// // #ifndef DUAL_LOOP_H
// // #define DUAL_LOOP_H
//
//
// /*
//  * File: dual.h
//  *
//  * Code generated for Simulink model 'dual'.
//  *
//  * Model version                  : 9.0
//  * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
//  * C/C++ source code generated on : Fri Aug  1 10:53:50 2025
//  *
//  * Target selection: ert.tlc
//  * Embedded hardware selection: Intel->x86-64 (Windows64)
//  * Code generation objectives:
//  *    1. Execution efficiency
//  *    2. RAM efficiency
//  * Validation result: Not run
//  */
//
// // #ifndef dual_h_
// // #define dual_h_
// #ifndef dual_COMMON_INCLUDES_
// #define dual_COMMON_INCLUDES_
// #include "rtwtypes.h"
// #include "math.h"
// #endif                                 /* dual_COMMON_INCLUDES_ */
//
// // /* Macros for accessing real-time model data structure */
// // #ifndef rtmGetErrorStatus
// // #define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
// // #endif
// //
// // #ifndef rtmSetErrorStatus
// // #define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
// // #endif
//
// /* Forward declaration for rtModel */
// typedef struct tag_RTM RT_MODEL;
//
//
//
// /* External inputs (root inport signals with default storage) */
// typedef struct {
//   real_T output;                       /* '<Root>/vref' */
//   real_T Vabc[3];                      /* '<Root>/Vabc' */
//   real_T theta;                        /* '<Root>/theta' */
// } ExtU;
//
// /* External outputs (root outports fed by signals with default storage) */
// typedef struct {
//   real_T abc[3];                       /* '<Root>/abc' */
// } ExtY;
//
// /* Real-time Model Data Structure */
// struct tag_RTM {
//   const char_T * volatile errorStatus;
// };
//
// /* Block signals and states (default storage) */
// // extern DW rtDW;
//
// /* External inputs (root inport signals with default storage) */
// extern ExtU rtU;
//
// /* External outputs (root outports fed by signals with default storage) */
// extern ExtY rtY;
//
// /* Model entry point functions */
// extern void dual_initialize(void);
// extern void dual_step(void);
//
// /* Real-time Model object */
// extern RT_MODEL *const rtM;
//
//
//
//
//
// // #endif //DUAL_LOOP_H

// #ifndef VCO_H
// #define VCO_H
//
// /*
//  * File: VCO0.h
//  *
//  * Code generated for Simulink model 'VCO0'.
//  *
//  * Model version                  : 9.0
//  * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
//  * C/C++ source code generated on : Fri Aug  1 11:00:26 2025
//  *
//  * Target selection: ert.tlc
//  * Embedded hardware selection: Intel->x86-32 (Windows32)
//  * Code generation objectives:
//  *    1. Execution efficiency
//  *    2. RAM efficiency
//  * Validation result: Not run
//  */
//
// #ifndef VCO0_COMMON_INCLUDES_
// #define VCO0_COMMON_INCLUDES_
// #include "rtwtypes.h"
// #include "math.h"
// #endif                                 /* VCO0_COMMON_INCLUDES_ */
//
// /* Macros for accessing real-time model data structure */
// #ifndef rtmGetErrorStatus
// #define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
// #endif
//
// #ifndef rtmSetErrorStatus
// #define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
// #endif
//
// /* Forward declaration for rtModel */
// typedef struct tag_RTM RT_MODEL;
//
// /* Block signals and states (default storage) for system '<Root>' */
//
//
// /* External inputs (root inport signals with default storage) */
// typedef struct {
//   real_T w0;                           /* '<Root>/w0' */
// } ExtU;
//
// /* External outputs (root outports fed by signals with default storage) */
// typedef struct {
//   real_T theta;                        /* '<Root>/theta' */
//   real_T f;                            /* '<Root>/f' */
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
// extern void VCO0_initialize(void);
// extern void VCO0_step(void);
//
// /* Real-time Model object */
// extern RT_MODEL *const rtM;
//
//
//
// #endif
//

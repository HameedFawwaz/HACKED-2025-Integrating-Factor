/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: WaijungTestLED.h
 *
 * Code generated with Waijung 2 ESP32 Target Blockset,
 * for Simulink model 'WaijungTestLED'.
 *
 * Model version                  : 1.6
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Sat Feb 15 23:00:03 2025
 *
 * Target selection: esp32.tlc
 * Embedded hardware selection: Cadence Design Systems (Tensilica)->Espressif Xtensa single-/dual-core 32-bit LX6 microprocessor
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef WaijungTestLED_h_
#define WaijungTestLED_h_
#ifndef WaijungTestLED_COMMON_INCLUDES_
#define WaijungTestLED_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                                 /* WaijungTestLED_COMMON_INCLUDES_ */

#include "waijung2_hwdrvlib.h"
#include "WaijungTestLED_types.h"
#include <stddef.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals (default storage) */
typedef struct {
  real_T PulseGenerator;               /* '<Root>/Pulse Generator' */
} B_WaijungTestLED_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  int32_T clockTickCounter;            /* '<Root>/Pulse Generator' */
} DW_WaijungTestLED_T;

/* Real-time Model Data Structure */
struct tag_RTM_WaijungTestLED_T {
  const char_T * volatile errorStatus;
};

/* Block signals (default storage) */
extern B_WaijungTestLED_T WaijungTestLED_B;

/* Block states (default storage) */
extern DW_WaijungTestLED_T WaijungTestLED_DW;

/* Model entry point functions */
extern void WaijungTestLED_initialize(void);
extern void WaijungTestLED_step(void);
extern void WaijungTestLED_terminate(void);

/* Real-time Model object */
extern RT_MODEL_WaijungTestLED_T *const WaijungTestLED_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'WaijungTestLED'
 */
#endif                                 /* WaijungTestLED_h_ */

/* [EOF] */

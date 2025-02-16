/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: WaijungTestLED.c
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

#include "WaijungTestLED.h"
#include "rtwtypes.h"

/* Block signals (default storage) */
B_WaijungTestLED_T WaijungTestLED_B;

/* Block states (default storage) */
DW_WaijungTestLED_T WaijungTestLED_DW;

/* Real-time model */
static RT_MODEL_WaijungTestLED_T WaijungTestLED_M_;
RT_MODEL_WaijungTestLED_T *const WaijungTestLED_M = &WaijungTestLED_M_;

/* Model step function */
void WaijungTestLED_step(void)
{
  /* DiscretePulseGenerator: '<Root>/Pulse Generator' */
  WaijungTestLED_B.PulseGenerator = ((WaijungTestLED_DW.clockTickCounter < 1) &&
    (WaijungTestLED_DW.clockTickCounter >= 0));

  /* DiscretePulseGenerator: '<Root>/Pulse Generator' */
  if (WaijungTestLED_DW.clockTickCounter >= 1) {
    WaijungTestLED_DW.clockTickCounter = 0;
  } else {
    WaijungTestLED_DW.clockTickCounter++;
  }

  /* S-Function (esp32_digital_output): '<Root>/Digital Output' */

  /* Updating the pins of DigitalOutput */
  gpio_set_level(2,(uint32_t) WaijungTestLED_B.PulseGenerator);/* GPIO2 */
}

/* Model initialize function */
void WaijungTestLED_initialize(void)
{
  /* Start for S-Function (esp32_digital_output): '<Root>/Digital Output' */
  GPIO_enable_DigitalOutput();
}

/* Model terminate function */
void WaijungTestLED_terminate(void)
{
  /* (no terminate code required) */
}

/* [EOF] */

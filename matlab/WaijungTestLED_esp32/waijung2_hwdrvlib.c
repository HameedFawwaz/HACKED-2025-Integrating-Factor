/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: waijung2_hwdrvlib.c
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

/** Model's header file **/
#include "WaijungTestLED.h"

void GPIO_enable_DigitalOutput()
{
  gpio_pad_select_gpio(2);
  gpio_set_direction(2, GPIO_MODE_OUTPUT);
  gpio_set_drive_capability(GPIO_NUM_2, GPIO_DRIVE_CAP_2);
}

/* [EOF] */

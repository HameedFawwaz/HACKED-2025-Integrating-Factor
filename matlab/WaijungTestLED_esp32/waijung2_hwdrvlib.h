/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: waijung2_hwdrvlib.h
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

#ifndef waijung2_hwdrvlib_h_
#define waijung2_hwdrvlib_h_
#include "driver/gpio.h"

/* FreeRTOS headers */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

/* ESP-IDF headers */
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"

/* ANSI C headers */
#include <string.h>
#define BASE_SAMPLE_TIME               0.5                       /* Base sample time in second */

void GPIO_enable_DigitalOutput();

#endif                                 /* waijung2_hwdrvlib_h_ */

/* [EOF] */

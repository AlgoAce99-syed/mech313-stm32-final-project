/**
 * @file    current_adc.c
 * @brief   3-phase current ADC reading and offset calibration module
 *
 * Students must fill in the body of the two functions below.
 * Function signatures and interfaces are fixed in current_adc.h (do not modify).
 *
 *   - ADC_MeasureOffset()   : 64-sample average for zero-current offset (once at boot)
 *   - ADC_ReadCurrents()    : direct DR register read → phase currents [A] (every ISR)
 *
 * Additional values students must set:
 *   - CURRENT_SCALE  ← compute 1 / (CSA_GAIN × shunt_R) yourself
 *                      (shunt_R = 2 mΩ fixed; CSA_GAIN comes from drv8323 reg 0x06)
 *
 * Cross-module consistency:
 *   - CSA_GAIN in drv8323.c  ↔  CURRENT_SCALE here — these MUST agree.
 *
 * ADC trigger architecture:
 *   - Normal operation:  TIM2 TRGO → ADC1 EOC → HAL_ADC_ConvCpltCallback() ISR
 *                        (ADC is converted *automatically* by external trigger.)
 *   - ADC_MeasureOffset: called before the ISR is enabled, so it needs *software*
 *                        triggering. Temporarily clear the EXTEN bit, measure,
 *                        then restore.
 */

#include "current_adc.h"
#include "main.h"

#define ADC_TO_VOLT    (3.3f / 4096.0f)   /* 12-bit ADC, 3.3V reference */
#define CURRENT_SCALE  25.0f              /* 1 / (20 V/V CSA_GAIN × 0.002 ohm) */

extern ADC_HandleTypeDef hadc1, hadc2, hadc3;

/* ── Current measurement variables ───────────────────────────────────────── */
volatile uint16_t current_raw_A = 0, current_raw_B = 0, current_raw_C = 0;
volatile float    current_A     = 0.0f, current_B = 0.0f, current_C = 0.0f;
         uint16_t adc_zero_A    = 2048, adc_zero_B = 2048, adc_zero_C = 2048;
         /* ↑ mid-rail of a 12-bit ADC (fallback before calibration) */


/* ════════════════════════════════════════════════════════════════════════════
 *  ── ADC Offset Calibration (64-sample average) ────────────────────────────
 * ════════════════════════════════════════════════════════════════════════════ */
void ADC_MeasureOffset(void)
{
// Inputs:
//   (none — motor must be stopped; called from main() BEFORE the ADC ISR is enabled)
//
// Outputs:
//   (void — updates module-level globals)
//   adc_zero_A/B/C  ← 64-sample average
//
// Used HAL APIs / Registers:
//   hadc1.Instance->CFGR             ← direct access to ADC1 CFGR register
//   ADC_CFGR_EXTEN                   ← bit mask for the external-trigger enable (HAL macro)
//   HAL_ADC_Start(&hadc)             ← start a software-triggered conversion
//   HAL_ADC_PollForConversion(&hadc, ms)  ← wait for conversion to finish (blocking)
//   HAL_ADC_GetValue(&hadc)          ← read the converted value
//   HAL_ADC_Stop(&hadc)              ← stop the ADC
//   HAL_Delay(ms)
//
// Goal:
//   Measure the zero-current offset of each ADC channel by averaging 64 samples.
//   Under normal operation the ADC is externally triggered by TIM2 TRGO, but this
//   function runs BEFORE the ISR is enabled, so it must use *software* triggering.
//
// Hint:
//   [Step 1] Save CFGR, then clear the EXTEN bit — switch trigger mode
//      - In normal mode, the EXTEN field of CFGR is set to enable external trigger.
//      - To use software trigger, clear EXTEN so HAL_ADC_Start() drives the conversion.
//      - Restore CFGR at the end so ISR-mode operation works after this call.
//
//   [Step 2] Loop: trigger and read all three ADCs together, accumulate sums.
//      - HAL_ADC_Start → HAL_ADC_PollForConversion → HAL_ADC_GetValue → HAL_ADC_Stop
//      - Channel mapping: ADC1 → phase C, ADC2 → phase B, ADC3 → phase A
//
//   [Step 3] Compute the average over 64 samples → store into adc_zero_A/B/C.
//
//   [Step 4] Restore the saved CFGR values.
//
// Pseudo Code:
//   (1) sum_A = sum_B = sum_C = 0;
//
//   (2) save the current CFGR of hadc1/2/3 into local variables (cfgr1, cfgr2, cfgr3)
//
//   (3) clear the ADC_CFGR_EXTEN bit in each ADC's CFGR
//        (e.g., hadc1.Instance->CFGR &= ~ADC_CFGR_EXTEN;  same for hadc2, hadc3)
//
//   (4) for i = 0 to 63:
//          HAL_ADC_Start  for hadc1, hadc2, hadc3
//          HAL_ADC_PollForConversion  for hadc1, hadc2, hadc3
//          sum_C += HAL_ADC_GetValue(&hadc1);    // ADC1 → phase C
//          sum_B += HAL_ADC_GetValue(&hadc2);    // ADC2 → phase B
//          sum_A += HAL_ADC_GetValue(&hadc3);    // ADC3 → phase A
//          HAL_ADC_Stop  for hadc1, hadc2, hadc3
//          HAL_Delay(1);
//
//   (5) compute the average of each sum (over 64 samples)
//        and store into adc_zero_A / adc_zero_B / adc_zero_C
//
//   (6) restore CFGR from cfgr1, cfgr2, cfgr3
    uint32_t sum_A = 0;
    uint32_t sum_B = 0;
    uint32_t sum_C = 0;

    uint32_t cfgr1 = hadc1.Instance->CFGR;
    uint32_t cfgr2 = hadc2.Instance->CFGR;
    uint32_t cfgr3 = hadc3.Instance->CFGR;

    hadc1.Instance->CFGR &= ~ADC_CFGR_EXTEN;
    hadc2.Instance->CFGR &= ~ADC_CFGR_EXTEN;
    hadc3.Instance->CFGR &= ~ADC_CFGR_EXTEN;

    for (uint32_t i = 0; i < 64; i++) {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_Start(&hadc2);
        HAL_ADC_Start(&hadc3);

        HAL_ADC_PollForConversion(&hadc1, 10);
        HAL_ADC_PollForConversion(&hadc2, 10);
        HAL_ADC_PollForConversion(&hadc3, 10);

        sum_C += HAL_ADC_GetValue(&hadc1);
        sum_B += HAL_ADC_GetValue(&hadc2);
        sum_A += HAL_ADC_GetValue(&hadc3);

        HAL_ADC_Stop(&hadc1);
        HAL_ADC_Stop(&hadc2);
        HAL_ADC_Stop(&hadc3);

        HAL_Delay(1);
    }

    adc_zero_A = (uint16_t)(sum_A / 64u);
    adc_zero_B = (uint16_t)(sum_B / 64u);
    adc_zero_C = (uint16_t)(sum_C / 64u);

    hadc1.Instance->CFGR = cfgr1;
    hadc2.Instance->CFGR = cfgr2;
    hadc3.Instance->CFGR = cfgr3;
}


/* ════════════════════════════════════════════════════════════════════════════
 *  ── Read ADC DR Registers → Phase Currents [A] ────────────────────────────
 * ════════════════════════════════════════════════════════════════════════════ */
void ADC_ReadCurrents(void)
{
// Inputs:
//   (none — the ADC has already finished a conversion via external trigger)
//
// Outputs:
//   (void — updates module-level globals)
//   current_raw_A/B/C  ← copied directly from ADC*->DR
//   current_A/B/C      ← raw → current [A] after offset removal and unit conversion
//
// Used Registers:
//   ADC1->DR, ADC2->DR, ADC3->DR   ← Data Registers, read directly (no HAL call)
//
// Goal:
//   The ISR is invoked at EOC, so the DR already holds fresh data — just read DR
//   and convert to amperes. Because this is called every 50 μs, we minimize
//   overhead by skipping HAL wrappers and reading the register directly.
//
// Hint:
//   - Channel mapping (hardware routing):
//        ADC1->DR  →  phase C
//        ADC2->DR  →  phase B
//        ADC3->DR  →  phase A
//
//   - Conversion chain (unit chain):
//        raw count  ──(ADC_TO_VOLT)──▶  voltage [V]  ──(CURRENT_SCALE)──▶  current [A]
//
//        Subtract the offset first, then run through the chain.
//
//   - Signed cast — careful:
//        raw and zero are uint16_t; if raw < zero, plain subtraction underflows.
//        Cast to (int32_t) before subtracting so a negative result is preserved.
//
// Pseudo Code:
//   (1) Read ADC DR registers directly (mind the channel mapping):
//        current_raw_C = (uint16_t)(ADC1->DR);
//        current_raw_B = (uint16_t)(ADC2->DR);
//        current_raw_A = (uint16_t)(ADC3->DR);
//
//   (2) For each phase, convert raw → amperes:
//        - compute the signed difference: (int32_t)raw_X − (int32_t)zero_X
//        - apply ADC_TO_VOLT to get voltage [V]
//        - apply CURRENT_SCALE to get current [A]
//        - store the result into current_A / current_B / current_C
    current_raw_C = (uint16_t)(ADC1->DR);
    current_raw_B = (uint16_t)(ADC2->DR);
    current_raw_A = (uint16_t)(ADC3->DR);

    current_A = ((float)((int32_t)current_raw_A - (int32_t)adc_zero_A)) *
                ADC_TO_VOLT * CURRENT_SCALE;
    current_B = ((float)((int32_t)current_raw_B - (int32_t)adc_zero_B)) *
                ADC_TO_VOLT * CURRENT_SCALE;
    current_C = ((float)((int32_t)current_raw_C - (int32_t)adc_zero_C)) *
                ADC_TO_VOLT * CURRENT_SCALE;
}

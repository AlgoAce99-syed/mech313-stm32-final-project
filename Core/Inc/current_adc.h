/**
 * @file    current_adc.h
 * @brief   3-phase current ADC reading and offset calibration module
 *
 * This header file is provided complete. Do not modify.
 * For implementation logic, algorithms, and pseudo code, refer to current_adc.c.
 *
 * Module overview:
 *   - ADC_MeasureOffset():  Once at boot — measure zero-current offset with motor stopped
 *   - ADC_ReadCurrents():   Every ISR (20 kHz) — ADC raw → phase current [A]
 *
 * Channel mapping (determined by hardware routing):
 *   - ADC1 → phase C
 *   - ADC2 → phase B
 *   - ADC3 → phase A
 */

#ifndef CURRENT_ADC_H
#define CURRENT_ADC_H

#include <stdint.h>

/* ── Current measurement variables ───────────────────────────────────────── */
extern volatile uint16_t current_raw_A, current_raw_B, current_raw_C;  /* ADC raw [0, 4095]    */
extern volatile float    current_A, current_B, current_C;              /* phase currents [A]   */
extern          uint16_t adc_zero_A, adc_zero_B, adc_zero_C;           /* zero-current offset (raw) */

/* ── Functions ───────────────────────────────────────────────────────────── */

/**
 * Measure 3-phase ADC zero-current offset.
 * Call ONCE at boot, with the motor stopped (before the ADC ISR is enabled).
 *
 * @post  adc_zero_A/B/C   ← updated to 64-sample averages
 * @post  ADC external-trigger configuration is preserved across this call
 */
void ADC_MeasureOffset(void);

/**
 * Convert ADC raw values to phase currents [A]. Called every ISR cycle.
 *
 * @post  current_raw_A/B/C  ← ADC*->DR (direct register read)
 * @post  current_A/B/C [A]  ← raw count → voltage → current
 */
void ADC_ReadCurrents(void);

#endif /* CURRENT_ADC_H */

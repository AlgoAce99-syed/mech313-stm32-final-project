/**
 * @file    svpwm.h
 * @brief   SVPWM (Space Vector PWM) output module
 *
 * This header file is provided complete. Do not modify.
 * For implementation logic, algorithms, and pseudo code, refer to svpwm.c.
 *
 * Module overview:
 *   - SVPWM_Output():  3-phase voltage command (va, vb, vc) [V] → TIM2 CCR1/2/3
 *                      Uses min-max zero-sequence injection to improve
 *                      DC bus utilization by ~15%.
 *
 * Notes:
 *   - ClampCCR() is a static helper inside .c.
 *   - SVPWM_Output() is the only externally visible function.
 */

#ifndef SVPWM_H
#define SVPWM_H

/* ── Functions ───────────────────────────────────────────────────────────── */

/**
 * Drive 3-phase voltages through center-aligned PWM with zero-sequence injection.
 *
 * @param   va, vb, vc  phase voltages [V], expected range ∈ [−VBUS/2, +VBUS/2]
 *
 * @post    TIM2 CCR1/2/3 updated; each CCR ∈ [0, pwm_period]
 *
 * @note    The motor only sees line-to-line voltages, so adding a common offset
 *          to all three phases does not change motor behaviour. We exploit this
 *          to inject a zero-sequence component that keeps each phase comfortably
 *          inside the PWM range.
 */
void SVPWM_Output(float va, float vb, float vc);

#endif /* SVPWM_H */

/**
 * @file    svpwm.c
 * @brief   SVPWM (Space Vector PWM) output module
 *
 * Students must fill in the body of the functions below.
 * Function signatures and interfaces are fixed in svpwm.h (do not modify).
 *
 *   - ClampCCR()       : Saturate CCR value into [0, pwm_period] (internal helper)
 *   - SVPWM_Output()   : 3-phase voltage → zero-sequence injection → TIM2 CCR
 *
 * Learning point — Zero-sequence injection (min-max method):
 *   - Pure sinusoidal PWM:      per-phase modulation range = ±VBUS/2
 *                               → max line-to-line voltage ≈ 0.866 × VBUS
 *   - With zero-seq injection:  add a common offset that symmetrizes
 *                               (v_max, v_min) about zero to all 3 phases
 *                               → max line-to-line voltage = VBUS
 *                               → ~15% better DC bus utilization
 *   - The motor sees only LINE-TO-LINE voltages; the common (zero-sequence) mode
 *     is invisible to it. We freely choose the common offset so each phase fits
 *     nicely inside the PWM range.
 *
 * Variables used:
 *   - VBUS         : DC bus voltage [V] (24 V on this board)
 *   - pwm_period   : TIM2 ARR value (≈ 4249, max count for center-aligned PWM)
 *   - pwm_center   : pwm_period / 2 (≈ 2125, the 50% duty reference)
 *   - htim2        : TIM2 handle (used via HAL macros to write CCR)
 */

#include "svpwm.h"
#include "main.h"

#define VBUS  24.0f   /* DC bus voltage [V] */

extern TIM_HandleTypeDef htim2;
extern uint32_t pwm_period;
extern uint32_t pwm_center;


/* ════════════════════════════════════════════════════════════════════════════
 *  ── ClampCCR — saturate CCR value (static helper) ─────────────────────────
 * ════════════════════════════════════════════════════════════════════════════ */
static uint32_t ClampCCR(int32_t v, uint32_t arr)
{
// Inputs:
//   int32_t  v    — computed CCR value (signed because it can be negative)
//   uint32_t arr  — upper bound (typically pwm_period)
//
// Outputs:
//   uint32_t  — v clamped into [0, arr]
//
// Goal:
//   Make sure the computed CCR value never falls outside the timer's
//   ARR range. Saturate between 0 (lower) and arr (upper).
//
// Hint:
//   - v negative   : return 0
//   - v > arr      : return arr
//   - otherwise    : return (uint32_t)v
//   - The signed → unsigned cast is safe *only after* clamping.
//
// Pseudo Code:
//   (1) if (v < 0)              return 0;
//   (2) if ((uint32_t)v > arr)  return arr;
//   (3) return (uint32_t)v;
    if (v < 0) {
        return 0;
    }
    if ((uint32_t)v > arr) {
        return arr;
    }
    return (uint32_t)v;
}


/* ════════════════════════════════════════════════════════════════════════════
 *  ── SVPWM_Output — 3-phase voltage → TIM2 CCR ─────────────────────────────
 * ════════════════════════════════════════════════════════════════════════════ */
void SVPWM_Output(float va, float vb, float vc)
{
// Inputs:
//   float va, vb, vc  — commanded phase voltages [V]
//                       expected range: ∈ [−VBUS/2, +VBUS/2]
//                       (Produced by the Inverse Clarke transform; under normal
//                        FOC operation they rarely leave this range.)
//
// Outputs:
//   (void — updates TIM2 CCR1/2/3 registers)
//
// Used HAL APIs / Macros:
//   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1/2/3, ccr_value)
//     → write ccr_value to TIM2's CCR1/2/3 register
//
// Goal:
//   3-phase voltage → inject zero-sequence offset → convert to timer counts
//   → write to CCR registers.
//
// Hint:
//   [Step 1] Find max (v_max) and min (v_min) among (va, vb, vc).
//            Use the ternary operator or plain comparisons.
//
//   [Step 2] Zero-sequence offset:
//            Compute a common offset that centers (v_max, v_min) symmetrically
//            about zero. Adding this offset to all three phases shifts the
//            common mode but leaves line-to-line voltages unchanged, so each
//            phase fits the ±VBUS/2 range better. (See lecture notes for the
//            exact formula.)
//
//   [Step 3] Compute the voltage-to-CCR scale factor:
//            Voltages of ±VBUS/2 should map to CCR ±pwm_center.
//            (0 V → CCR = pwm_center, i.e. 50% duty.)
//            Derive the scale from this mapping.
//
//   [Step 4] Per-phase CCR:
//            For each phase, convert (v_i + v_offset) into a CCR value that
//            swings up/down around the 50% duty reference (pwm_center).
//
//   [Step 5] Clamp into [0, pwm_period] (safety) and write to TIM2.
//            (Usually voltages stay in range, but extreme commands could
//             push the result outside — clamp to be safe.)
//
// Pseudo Code:
//   (1) v_max = max(va, vb, vc);
//       v_min = min(va, vb, vc);
//   (2) compute zero-sequence offset
//        (subtract the mean of v_max and v_min to symmetrize about zero)
//   (3) compute the voltage → CCR scale factor
//   (4) for each phase, compute the CCR value from (v_i + v_offset)
//        (each centered on pwm_center)
//   (5) clamp each CCR into [0, pwm_period] and write to TIM2:
//        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ClampCCR(ccr1, pwm_period));
//        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ClampCCR(ccr2, pwm_period));
//        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ClampCCR(ccr3, pwm_period));
    float v_max = va;
    if (vb > v_max) {
        v_max = vb;
    }
    if (vc > v_max) {
        v_max = vc;
    }

    float v_min = va;
    if (vb < v_min) {
        v_min = vb;
    }
    if (vc < v_min) {
        v_min = vc;
    }

    float v_offset = -0.5f * (v_max + v_min);
    float scale = (2.0f * (float)pwm_center) / VBUS;

    int32_t ccr1 = (int32_t)((float)pwm_center + (va + v_offset) * scale);
    int32_t ccr2 = (int32_t)((float)pwm_center + (vb + v_offset) * scale);
    int32_t ccr3 = (int32_t)((float)pwm_center + (vc + v_offset) * scale);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ClampCCR(ccr1, pwm_period));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ClampCCR(ccr2, pwm_period));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ClampCCR(ccr3, pwm_period));
}

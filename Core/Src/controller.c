/**
 * @file    controller.c
 * @brief   PI / PD controller module
 *
 * Students must fill in the body of the six functions below.
 * Function signatures and structs are fixed in controller.h (do not modify).
 *
 *   - PI_Init() / PI_Update() / PI_Reset()
 *   - PD_Init() / PD_Update() / PD_Reset()
 *
 * Core learning points:
 *   - PI:  accumulate the integral + anti-windup (clamp the integrator to
 *          ±limit BEFORE summing with the proportional term)
 *   - PD:  finite-difference derivative (divide the change in error by dt)
 *   - Both clamp the final output to ±limit (output saturation)
 *   - Reset:  called by cascade_foc on control-mode change
 *             → prevents stale integrator / derivative state from a prior mode
 *               (suppresses windup transients)
 */

#include "controller.h"


/* ════════════════════════════════════════════════════════════════════════════
 *  ── PI Controller ─────────────────────────────────────────────────────────
 * ════════════════════════════════════════════════════════════════════════════ */

/* ── PI_Init ─────────────────────────────────────────────────────────────── */
void PI_Init(SimplePI *pi, float kp, float ki, float dt, float limit)
{
// Inputs:
//   SimplePI *pi    — controller instance
//   float kp, ki    — proportional and integral gains
//   float dt        — sample period [s]
//   float limit     — symmetric saturation
//
// Goal:
//   Store the gain / dt / limit fields from the arguments.
//   Initialize the integrator to 0.
//
// Pseudo Code:
//   (1) store kp, ki, dt, limit into pi's fields
//   (2) pi->integral = 0
    pi->kp = kp;
    pi->ki = ki;
    pi->dt = dt;
    pi->limit = limit;
    pi->integral = 0.0f;
}


/* ── PI_Update ───────────────────────────────────────────────────────────── */
float PI_Update(SimplePI *pi, float error)
{
// Inputs:
//   SimplePI *pi    — controller instance
//   float error     — setpoint − measurement
//
// Outputs:
//   float — saturated output ∈ [−pi->limit, +pi->limit]
//
// Goal:
//   Compute one PI step.
//   Accumulate the error contribution into the integrator → anti-windup clamp
//   → sum proportional + integral → saturate the output → return.
//
// Hint:
//   - Integration step:
//        Update the integrator using (ki, dt, error). Refer to lecture notes
//        for the integration update.
//   - Anti-windup:
//        AFTER updating the integrator, clamp its value to ±limit.
//        This prevents the integrator from growing unbounded while the output
//        is already saturated.
//   - Output sum:
//        Sum the proportional term with the integrator.
//   - Output saturation:
//        Clamp the final output to ±limit before returning.
//
// Pseudo Code:
//   (1) accumulate the new error contribution into integral   (uses ki, dt, error)
//   (2) clamp integral to ±pi->limit                          // anti-windup
//   (3) out = proportional term + integral
//   (4) clamp out to ±pi->limit                               // output saturation
//   (5) return out;
    pi->integral += pi->ki * error * pi->dt;

    if (pi->integral > pi->limit) {
        pi->integral = pi->limit;
    } else if (pi->integral < -pi->limit) {
        pi->integral = -pi->limit;
    }

    float out = pi->kp * error + pi->integral;

    if (out > pi->limit) {
        out = pi->limit;
    } else if (out < -pi->limit) {
        out = -pi->limit;
    }

    return out;
}


/* ── PI_Reset ────────────────────────────────────────────────────────────── */
void PI_Reset(SimplePI *pi)
{
// Inputs:
//   SimplePI *pi    — controller instance
//
// Goal:
//   Zero the integrator. Called by cascade_foc on control-mode change.
//
// Pseudo Code:
//   (1) pi->integral = 0
    pi->integral = 0.0f;
}


/* ════════════════════════════════════════════════════════════════════════════
 *  ── PD Controller ─────────────────────────────────────────────────────────
 * ════════════════════════════════════════════════════════════════════════════ */

/* ── PD_Init ─────────────────────────────────────────────────────────────── */
void PD_Init(SimplePD *pd, float kp, float kd, float limit)
{
// Inputs:
//   SimplePD *pd    — controller instance
//   float kp, kd    — proportional and derivative gains
//   float limit     — symmetric saturation
//
// Goal:
//   Store the gain / limit fields from the arguments.
//   Initialize prev_error to 0.
//
// Pseudo Code:
//   (1) store kp, kd, limit into pd's fields
//   (2) pd->prev_error = 0
    pd->kp = kp;
    pd->kd = kd;
    pd->limit = limit;
    pd->prev_error = 0.0f;
}


/* ── PD_Update ───────────────────────────────────────────────────────────── */
float PD_Update(SimplePD *pd, float error, float dt)
{
// Inputs:
//   SimplePD *pd    — controller instance
//   float error     — setpoint − measurement
//   float dt        — sample period [s]
//
// Outputs:
//   float — saturated output ∈ [−pd->limit, +pd->limit]
//
// Goal:
//   Compute one PD step.
//   Estimate derivative from (error − prev_error) → update prev_error →
//   sum proportional + derivative → saturate the output → return.
//
// Hint:
//   - Derivative estimate:
//        Compute a finite-difference derivative from the change in error,
//        divided by dt.
//        (※ This amplifies noise on noisy signals; for this project the
//         basic form is acceptable.)
//   - prev_error update:
//        Overwrite prev_error with the current error AFTER computing the
//        derivative.  (Order matters — if you overwrite BEFORE the difference,
//        the derivative is always 0.)
//   - Output sum:
//        Sum the proportional term with the derivative term.
//   - Output saturation:
//        Clamp the final output to ±limit before returning.
//
// Pseudo Code:
//   (1) deriv = (difference between current error and prev_error) / dt
//   (2) save current error into prev_error                    // for next call
//   (3) out = proportional term + derivative term
//   (4) clamp out to ±pd->limit                               // output saturation
//   (5) return out;
    float deriv = (error - pd->prev_error) / dt;
    pd->prev_error = error;

    float out = pd->kp * error + pd->kd * deriv;

    if (out > pd->limit) {
        out = pd->limit;
    } else if (out < -pd->limit) {
        out = -pd->limit;
    }

    return out;
}


/* ── PD_Reset ────────────────────────────────────────────────────────────── */
void PD_Reset(SimplePD *pd)
{
// Inputs:
//   SimplePD *pd    — controller instance
//
// Goal:
//   Zero prev_error. Called by cascade_foc on control-mode change.
//
// Pseudo Code:
//   (1) pd->prev_error = 0
    pd->prev_error = 0.0f;
}

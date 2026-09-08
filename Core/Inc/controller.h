/**
 * @file    controller.h
 * @brief   PI / PD controller module — for current, velocity, and position loops
 *
 * This header file is provided complete. Do not modify.
 * For implementation logic, algorithms, and pseudo code, refer to controller.c.
 *
 * Module overview:
 *   - SimplePI struct + Init / Update / Reset     : used for current / velocity loops
 *   - SimplePD struct + Init / Update / Reset     : used for the position loop
 *
 * Where used:
 *   - cascade_foc.c calls 4 instances of these controllers:
 *       pi_d, pi_q  : d/q-axis current PI
 *       pi_vel      : velocity PI
 *       pd_pos      : position PD
 *   - On control-mode change, Reset() must be called on the now-inactive
 *     controllers to prevent windup transients.
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

/* ── PI controller ─────────────────────────────────────────────────────── */
typedef struct {
    float kp, ki, dt;
    float integral;     /* accumulator; kept within ±limit via anti-windup */
    float limit;        /* output saturation and integrator clamp bound    */
} SimplePI;

/**
 * Initialize a PI controller instance.
 *
 * @param   pi      controller instance
 * @param   kp,ki   proportional and integral gains
 * @param   dt      sample period [s]
 * @param   limit   symmetric saturation (used for both output and anti-windup clamp)
 *
 * @post    pi->integral = 0
 */
void  PI_Init  (SimplePI *pi, float kp, float ki, float dt, float limit);

/**
 * Compute one PI step (with anti-windup).
 *
 * @param   error   setpoint − measurement
 * @return  saturated output ∈ [−limit, +limit]
 */
float PI_Update(SimplePI *pi, float error);

/**
 * Zero the integrator. Call on control-mode change.
 */
void  PI_Reset (SimplePI *pi);


/* ── PD controller (used for the position loop) ────────────────────────── */
typedef struct {
    float kp, kd;
    float limit;        /* output saturation                              */
    float prev_error;   /* previous error, needed for derivative estimate */
} SimplePD;

/**
 * Initialize a PD controller instance.
 *
 * @param   pd      controller instance
 * @param   kp,kd   proportional and derivative gains
 * @param   limit   symmetric saturation (output bound)
 *
 * @post    pd->prev_error = 0
 */
void  PD_Init  (SimplePD *pd, float kp, float kd, float limit);

/**
 * Compute one PD step.
 *
 * @param   error   setpoint − measurement
 * @param   dt      sample period [s]
 * @return  saturated output ∈ [−limit, +limit]
 */
float PD_Update(SimplePD *pd, float error, float dt);

/**
 * Zero prev_error. Call on control-mode change.
 */
void  PD_Reset (SimplePD *pd);

#endif /* CONTROLLER_H */

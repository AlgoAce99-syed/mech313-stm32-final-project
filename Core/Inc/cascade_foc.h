/**
 * @file    cascade_foc.h
 * @brief   Cascade FOC position control module
 *
 * This header file is provided complete. Do not modify.
 * For implementation logic, algorithms, and pseudo code, refer to cascade_foc.c.
 *
 * Module overview:
 *   - CtrlMode enum                  : position / velocity / current mode
 *   - id_meas / iq_meas              : measured d/q-axis currents [A] (telemetry)
 *   - vd_out  / vq_out               : output d/q-axis voltages [V] (telemetry)
 *   - UpdateCascadeFOC()             : called every ISR — full FOC chain in one call
 *
 * Active loops per control mode:
 *   - CTRL_POSITION:  PD_pos → PI_vel → PI_id/iq → SVPWM
 *   - CTRL_VELOCITY:           PI_vel → PI_id/iq → SVPWM   (PD_pos is reset)
 *   - CTRL_CURRENT:                     PI_id/iq → SVPWM   (PD_pos, PI_vel reset)
 */

#ifndef CASCADE_FOC_H
#define CASCADE_FOC_H

#include "controller.h"

/* ── Control mode ────────────────────────────────────────────────── */
typedef enum {
    CTRL_CURRENT  = 0,   /* iq_ref_cmd used directly */
    CTRL_VELOCITY = 1,   /* vel_ref_cmd used */
    CTRL_POSITION = 2,   /* pos_ref used [rad] */
} CtrlMode;

/* ── FOC output (telemetry — for debugging / plotting) ───────────── */
extern volatile float id_meas, iq_meas;   /* measured dq-axis currents [A] */
extern volatile float vd_out,  vq_out;    /* output dq-axis voltages  [V]  */

/* ── Functions ───────────────────────────────────────────────────── */

/**
 * One step of cascade FOC (control period 1/dt = 20 kHz).
 *
 * @param   current_A, current_B   measured phase currents [A]
 * @param   elec_theta              current electrical angle [rad]
 * @param   pos_mech                current mechanical angle [rad] (unwrapped)
 * @param   vel_mech                current mechanical angular velocity [rad/s] (IIR-filtered)
 * @param   pos_ref                 position command [rad]       (CTRL_POSITION mode)
 * @param   vel_ref_cmd             velocity command [rad/s]     (CTRL_VELOCITY mode)
 * @param   iq_ref_cmd              q-axis current command [A]   (CTRL_CURRENT mode)
 * @param   id_ref                  d-axis current command [A]   (typically 0 for SPMSM)
 * @param   ctrl_mode               POSITION / VELOCITY / CURRENT
 * @param   pi_d, pi_q              inner current PI instances
 * @param   pi_vel                  outer velocity PI instance
 * @param   pd_pos                  outer position PD instance
 * @param   dt                      sample period [s]
 *
 * @post    id_meas, iq_meas, vd_out, vq_out updated (telemetry)
 * @post    SVPWM_Output called → TIM2 CCR updated
 *
 * @note    On mode change, call Reset() on the now-inactive controllers
 *          to prevent windup transients.
 */
void UpdateCascadeFOC(float current_A, float current_B,
                      float elec_theta,
                      float pos_mech,  float vel_mech,
                      float pos_ref,   float vel_ref_cmd, float iq_ref_cmd,
                      float id_ref,    CtrlMode ctrl_mode,
                      SimplePI *pi_d,  SimplePI *pi_q,
                      SimplePI *pi_vel, SimplePD *pd_pos,
                      float dt);

#endif /* CASCADE_FOC_H */

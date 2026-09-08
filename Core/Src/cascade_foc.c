/**
 * @file    cascade_foc.c
 * @brief   Cascade FOC position control module
 *
 * Students must fill in the body of UpdateCascadeFOC().
 * Function signature and interface are fixed in cascade_foc.h (do not modify).
 *
 * 8-stage composition:
 *   [1] Clarke transform         : (i_A, i_B) → (i_α, i_β)
 *   [2] Park transform            : (i_α, i_β, θ) → (id, iq)
 *   [3] Outer loop — mode branch
 *        POSITION:  PD_pos produces vel_ref*
 *        VELOCITY:  vel_ref* = vel_ref_cmd  (PD_pos is reset)
 *        CURRENT:   PD_pos and PI_vel both reset → iq_ref* = iq_ref_cmd
 *   [4] Velocity loop — only when ctrl_mode != CTRL_CURRENT
 *        PI_vel produces iq_ref*
 *   [5] Current loop
 *        PI_d  → vd
 *        PI_q  → vq
 *   [6] Inverse Park              : (vd, vq, θ) → (v_α, v_β)
 *   [7] Inverse Clarke            : (v_α, v_β) → (v_a, v_b, v_c)
 *   [8] SVPWM_Output(va, vb, vc)
 *
 * Core learning points:
 *   - 4 coordinate transforms (Clarke / Park / inverse Park / inverse Clarke)
 *     — refer to lecture notes for the formulas.
 *   - Mode branching + Reset semantics — prevents stale state in inactive loops.
 *   - Inter-loop reference passing — static variables (vel_ref_inner, iq_ref_inner).
 *   - cos(θ), sin(θ) are used by both Park and inverse Park
 *     — compute once and reuse.
 */

#include "cascade_foc.h"
#include "svpwm.h"
#include <math.h>

/* ── FOC output variables (telemetry, observed by the caller for debugging) ─ */
volatile float id_meas = 0.0f, iq_meas = 0.0f;
volatile float vd_out  = 0.0f, vq_out  = 0.0f;

/* ── Inter-loop references (static, kept across ISR calls) ─────── */
static float vel_ref_inner = 0.0f;   /* outer loop → velocity loop */
static float iq_ref_inner  = 0.0f;   /* outer loop → current loop  */


/* ════════════════════════════════════════════════════════════════════════════
 *  ── UpdateCascadeFOC — 8-stage composition ────────────────────────────────
 * ════════════════════════════════════════════════════════════════════════════ */
void UpdateCascadeFOC(float current_A, float current_B,
                      float elec_theta,
                      float pos_mech,  float vel_mech,
                      float pos_ref,   float vel_ref_cmd, float iq_ref_cmd,
                      float id_ref,    CtrlMode ctrl_mode,
                      SimplePI *pi_d,  SimplePI *pi_q,
                      SimplePI *pi_vel, SimplePD *pd_pos,
                      float dt)
{
// Inputs:
//   (see signature — measurements, commands, 4 controller pointers, mode, dt)
//
// Outputs:
//   (void — updates telemetry globals + calls SVPWM_Output → TIM2 CCR)
//
// Used functions:
//   cosf(x), sinf(x)                ← <math.h>
//   PI_Update, PI_Reset             ← controller.h
//   PD_Update, PD_Reset             ← controller.h
//   SVPWM_Output(va, vb, vc)        ← svpwm.h
//
// Goal:
//   Execute the 8-stage chain every ISR cycle (50 μs):
//       Clarke → Park → outer loop (by mode) → velocity loop → current loop →
//       inverse Park → inverse Clarke → SVPWM
//
// Hint:
//
//   [Step 1-2] Clarke + Park transform
//     - Compute cos(elec_theta), sin(elec_theta) ONCE and share with both
//       transforms.
//     - Clarke:  (i_A, i_B) → (i_α, i_β)
//                 i_α equals i_A.
//                 i_β is a linear combination of (i_A, i_B) — see lecture notes.
//                 (Kirchhoff: i_A + i_B + i_C = 0 → no need to measure i_C.)
//     - Park:    (i_α, i_β, θ) → (id, iq)
//                Rotation transform (rotation matrix) — see lecture notes.
//     - Save id, iq into id_meas, iq_meas (telemetry for debugging / plotting).
//
//   [Step 3] Outer loop — branch on ctrl_mode
//     - CTRL_POSITION:
//         pos_err = pos_ref − pos_mech
//         vel_ref_inner = PD_Update(pd_pos, pos_err, dt)
//     - CTRL_VELOCITY:
//         vel_ref_inner = vel_ref_cmd
//         PD_Reset(pd_pos)                  // position loop deactivated
//     - CTRL_CURRENT:
//         PD_Reset(pd_pos)
//         PI_Reset(pi_vel)                  // velocity loop also deactivated
//         iq_ref_inner = iq_ref_cmd
//
//   [Step 4] Velocity loop — only when ctrl_mode != CTRL_CURRENT
//     - vel_err = vel_ref_inner − vel_mech
//     - iq_ref_inner = PI_Update(pi_vel, vel_err)
//
//   [Step 5] Current loop
//     - vd = PI_Update(pi_d, id_ref − id)
//     - vq = PI_Update(pi_q, iq_ref_inner − iq)
//     - Save vd, vq into vd_out, vq_out (telemetry).
//
//   [Step 6] Inverse Park
//     - (vd, vq, θ) → (v_α, v_β).
//     - Inverse rotation of Park — see lecture notes.
//
//   [Step 7] Inverse Clarke
//     - (v_α, v_β) → (v_a, v_b, v_c).
//     - Recover 3-phase voltages — see lecture notes.
//     - v_a equals v_α. v_b and v_c are linear combinations of (v_α, v_β).
//
//   [Step 8] SVPWM output
//     - Call SVPWM_Output(va, vb, vc) — the svpwm module injects zero-sequence
//       and writes the CCR registers.
//
// Pseudo Code:
//
//   (1) cos_t = cosf(elec_theta);
//       sin_t = sinf(elec_theta);
//
//   (2) [Clarke + Park transform]
//       compute i_alpha from current_A
//       compute i_beta  from (current_A, current_B)        ← see lecture notes
//       compute id, iq  by rotating (i_alpha, i_beta) with (cos_t, sin_t)
//       id_meas = id;  iq_meas = iq;
//
//   (3) [Outer loop — branch by ctrl_mode]
//       if      (ctrl_mode == CTRL_POSITION):
//          vel_ref_inner = PD_Update(pd_pos, pos_ref − pos_mech, dt);
//       else if (ctrl_mode == CTRL_VELOCITY):
//          vel_ref_inner = vel_ref_cmd;
//          PD_Reset(pd_pos);
//       else:   // CTRL_CURRENT
//          PD_Reset(pd_pos);
//          PI_Reset(pi_vel);
//          iq_ref_inner = iq_ref_cmd;
//
//   (4) [Velocity loop — skip if CTRL_CURRENT]
//       if (ctrl_mode != CTRL_CURRENT):
//          iq_ref_inner = PI_Update(pi_vel, vel_ref_inner − vel_mech);
//
//   (5) [Inner current loop]
//       vd = PI_Update(pi_d, id_ref − id);
//       vq = PI_Update(pi_q, iq_ref_inner − iq);
//       vd_out = vd;  vq_out = vq;
//
//   (6) [Inverse Park]
//       compute v_alpha, v_beta by inverse-rotating (vd, vq) with (cos_t, sin_t)
//
//   (7) [Inverse Clarke]
//       va = v_alpha;
//       compute vb, vc from (v_alpha, v_beta)              ← see lecture notes
//
//   (8) [SVPWM output]
//       SVPWM_Output(va, vb, vc);
    float cos_t = cosf(elec_theta);
    float sin_t = sinf(elec_theta);

    float i_alpha = current_A;
    float i_beta = (current_A + 2.0f * current_B) * 0.57735026919f;

    float id = i_alpha * cos_t + i_beta * sin_t;
    float iq = -i_alpha * sin_t + i_beta * cos_t;
    id_meas = id;
    iq_meas = iq;

    if (ctrl_mode == CTRL_POSITION) {
        vel_ref_inner = PD_Update(pd_pos, pos_ref - pos_mech, dt);
    } else if (ctrl_mode == CTRL_VELOCITY) {
        vel_ref_inner = vel_ref_cmd;
        PD_Reset(pd_pos);
    } else {
        PD_Reset(pd_pos);
        PI_Reset(pi_vel);
        iq_ref_inner = iq_ref_cmd;
    }

    if (ctrl_mode != CTRL_CURRENT) {
        iq_ref_inner = PI_Update(pi_vel, vel_ref_inner - vel_mech);
    }

    float vd = PI_Update(pi_d, id_ref - id);
    float vq = PI_Update(pi_q, iq_ref_inner - iq);
    vd_out = vd;
    vq_out = vq;

    float v_alpha = vd * cos_t - vq * sin_t;
    float v_beta = vd * sin_t + vq * cos_t;

    float va = v_alpha;
    float vb = -0.5f * v_alpha + 0.86602540378f * v_beta;
    float vc = -0.5f * v_alpha - 0.86602540378f * v_beta;

    SVPWM_Output(va, vb, vc);
}

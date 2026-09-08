/**
 * @file    encoder.h
 * @brief   AS5047P magnetic encoder reading and angle/velocity estimation module
 *
 * This header file is provided complete. Do not modify.
 * For implementation logic, algorithms, and pseudo code, refer to encoder.c.
 *
 * Module overview:
 *   - Encoder_ReadAngle():           Read 14-bit raw angle from AS5047P
 *   - ElecThetaFromEncoder():        raw count → electrical angle [rad]
 *   - UpdateMechAngleAndVelocity():  Unwrap mechanical angle + IIR-filtered velocity
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* ── Encoder variables ───────────────────────────────────────────────────── */
extern volatile uint16_t encoder_raw;       /* 14-bit raw count [0, 16383]                    */
extern const    uint16_t encoder_elec_zero; /* raw count at electrical angle 0° (calibration) */
extern volatile uint16_t enc_rx_frame1;     /* SPI 1st frame (for debugging)                  */
extern volatile uint16_t enc_rx_frame2;     /* SPI 2nd frame (actual data, for debugging)     */
extern volatile float    elec_theta_enc;    /* electrical angle [rad] ∈ [0, 2π)               */
extern volatile float    pos_mech_rad;      /* unwrapped mechanical angle [rad]               */
extern volatile float    vel_mech_rads;     /* filtered mechanical angular velocity [rad/s]   */

/* ── Functions ───────────────────────────────────────────────────────────── */

/**
 * Read 14-bit raw angle from the AS5047P magnetic encoder.
 *
 * @return  14-bit raw encoder count ∈ [0, 16383]
 *          (one mechanical revolution = 16384 counts; wraps on rotation)
 */
uint16_t Encoder_ReadAngle(void);

/**
 * Convert raw encoder count → electrical angle [rad].
 * Used directly by the Park transform (cosf(θ_elec), sinf(θ_elec)) in FOC.
 *
 * @param   enc_raw     current raw count ∈ [0, 16383]
 * @param   elec_zero   raw count at electrical angle 0° (calibration value)
 * @return  electrical angle [rad] ∈ [0, 2π)
 */
float    ElecThetaFromEncoder(uint16_t enc_raw, uint16_t elec_zero);

/**
 * Unwrap mechanical angle and update IIR-filtered velocity.
 * Called every cycle inside the 20 kHz ADC ISR.
 *
 * @param   enc_now     current raw count from this cycle
 *
 * @post    pos_mech_rad   accumulated by this cycle's increment (units: rad)
 * @post    vel_mech_rads  updated via 1st-order IIR (units: rad/s)
 */
void     UpdateMechAngleAndVelocity(uint16_t enc_now);

#endif /* ENCODER_H */

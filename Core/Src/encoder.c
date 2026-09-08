/**
 * @file    encoder.c
 * @brief   AS5047P magnetic encoder reading and angle/velocity estimation module
 *
 * Students must fill in the body of the three functions below.
 * Function signatures and interfaces are fixed in encoder.h (do not modify).
 *
 *   - Encoder_ReadAngle()            : Read 14-bit raw angle via two SPI transfers
 *   - ElecThetaFromEncoder()         : raw count → electrical angle [rad]
 *   - UpdateMechAngleAndVelocity()   : Unwrap mechanical angle + IIR velocity estimate
 *
 * Additional values students must set:
 *   - encoder_elec_zero  ← Replace with the measured value after running
 *                          encoder_calibration.c
 *   - VEL_FILT_ALPHA     ← Tune in the range 0.01~0.1
 */

#include "encoder.h"
#include "main.h"
#include <math.h>

#define POLE_PAIRS      7
#define ENC_TO_MECH_RAD (2.0f * (float)M_PI / 16384.0f)
#define CTRL_DT         (1.0f / 20000.0f)
#define VEL_FILT_ALPHA  0.03f   /* IIR velocity filter coefficient */

extern SPI_HandleTypeDef hspi1;

/* ── Encoder variables ───────────────────────────────────────────── */
volatile uint16_t encoder_raw       = 0;
const    uint16_t encoder_elec_zero = 9255;    /* Measured with encoder_calibration.c */
volatile uint16_t enc_rx_frame1     = 0;
volatile uint16_t enc_rx_frame2     = 0;
volatile float    elec_theta_enc    = 0.0f;
volatile float    pos_mech_rad      = 0.0f;    /* unwrapped mechanical angle [rad] */
volatile float    vel_mech_rads     = 0.0f;    /* mechanical angular velocity [rad/s], 1st-order IIR */

static   uint16_t enc_prev          = 0;       /* previous encoder raw value */
static   uint8_t  enc_initialized   = 0;       /* first update sync flag */


/* ════════════════════════════════════════════════════════════════════════════
 *  ── Read 14-bit Raw Angle from AS5047P ────────────────────────────────────
 * ════════════════════════════════════════════════════════════════════════════ */
uint16_t Encoder_ReadAngle(void)
{
// Inputs:
//   (none — communicates with AS5047P over SPI1)
//
// Outputs:
//   uint16_t — 14-bit absolute angle raw count, range [0, 16383]
//              (0 = 0°, 16383 ≈ 359.978°, wraps once per revolution)
//
// Used HAL APIs:
//   HAL_GPIO_WritePin(enc_cs_GPIO_Port, enc_cs_Pin, GPIO_PIN_RESET / GPIO_PIN_SET)
//     → Toggle CS pin LOW/HIGH (SPI slave select)
//   HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&tx, (uint8_t*)&rx, 1, 10)
//     → Transmit/receive one 16-bit word, timeout 10 ms
//
// Goal:
//   Read the 14-bit absolute angle from the AS5047P magnetic encoder over SPI.
//
// Hint:
//   - AS5047P uses a "delayed response" protocol:
//        1st transfer: send the command (the value received back is from the
//                      *previous* command → discard).
//        2nd transfer: receive the actual angle data.
//   - tx can simply be 0xFFFF
//     (or use the READ NOP command from the datasheet).
//   - CS pin: pull LOW (assert) before each transfer, HIGH (release) after.
//     **Between the two transfers, CS MUST go HIGH** (two separate transactions).
//   - Of the 16 bits received, the top 2 bits are parity/error flags;
//     the bottom 14 bits are the angle. Mask with 0x3FFF before returning.
//
// Pseudo Code:
//   (1) tx = 0xFFFF; rx = 0;
//   (2) [Transfer #1 — dummy / command]
//        CS LOW  →  HAL_SPI_TransmitReceive(tx, &rx, 1)  →  CS HIGH
//        (optional: enc_rx_frame1 = rx for debugging)
//   (3) [Transfer #2 — real data]
//        CS LOW  →  HAL_SPI_TransmitReceive(tx, &rx, 1)  →  CS HIGH
//        (optional: enc_rx_frame2 = rx for debugging)
//   (4) return (rx & 0x3FFF);
    uint16_t tx = 0xFFFFu;
    uint16_t rx = 0;

    HAL_GPIO_WritePin(enc_cs_GPIO_Port, enc_cs_Pin, GPIO_PIN_RESET);
    (void)HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&tx, (uint8_t *)&rx, 1, 10);
    HAL_GPIO_WritePin(enc_cs_GPIO_Port, enc_cs_Pin, GPIO_PIN_SET);
    enc_rx_frame1 = rx;

    rx = 0;
    HAL_GPIO_WritePin(enc_cs_GPIO_Port, enc_cs_Pin, GPIO_PIN_RESET);
    (void)HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&tx, (uint8_t *)&rx, 1, 10);
    HAL_GPIO_WritePin(enc_cs_GPIO_Port, enc_cs_Pin, GPIO_PIN_SET);
    enc_rx_frame2 = rx;

    return (uint16_t)(rx & 0x3FFFu);
}


/* ════════════════════════════════════════════════════════════════════════════
 *  ── Electrical Angle Calculation ──────────────────────────────────────────
 * ════════════════════════════════════════════════════════════════════════════ */
float ElecThetaFromEncoder(uint16_t enc_raw, uint16_t elec_zero)
{
// Inputs:
//   uint16_t enc_raw    — current raw encoder count ∈ [0, 16383]
//   uint16_t elec_zero  — raw count at electrical angle 0° (measured via
//                         encoder_calibration.c, read by debugger)
//
// Outputs:
//   float — electrical angle [rad], range [0, 2π)
//           Fed directly into the Park transform cosf(θ_elec), sinf(θ_elec) in FOC
//
// Parameters used:
//   POLE_PAIRS = 7  (number of pole pairs of this motor)
//
// Goal:
//   Convert raw mechanical count → electrical angle [rad].
//
// Hint:
//   - 1 mechanical revolution = POLE_PAIRS electrical revolutions (= 7 here).
//     Use this to figure out how many encoder counts make up one electrical cycle.
//   - First compute how far enc_raw is from elec_zero (delta).
//     Delta may be negative; wrap it into the positive range
//     (one mechanical revolution = 16384 counts).
//   - Take the remainder of delta within one electrical cycle.
//     (hint: <math.h> has a floating-point modulo function.)
//   - Convert that fraction of an electrical cycle into radians.
//
// Pseudo Code:
//   (1) compute the encoder counts per one electrical cycle
//   (2) delta = enc_raw − elec_zero      (signed)
//   (3) if delta is negative, wrap it into the positive range
//   (4) take the remainder of delta within one electrical cycle
//        (use the floating-point modulo from <math.h>)
//   (5) convert that fraction to radians and return
    float counts_per_elec_cycle = 16384.0f / (float)POLE_PAIRS;
    int32_t delta = (int32_t)(enc_raw & 0x3FFFu) - (int32_t)(elec_zero & 0x3FFFu);

    if (delta < 0) {
        delta += 16384;
    }

    float elec_counts = fmodf((float)delta, counts_per_elec_cycle);
    return (elec_counts / counts_per_elec_cycle) * 2.0f * (float)M_PI;
}


/* ════════════════════════════════════════════════════════════════════════════
 *  ── Mechanical Angle and Velocity Update ──────────────────────────────────
 * ════════════════════════════════════════════════════════════════════════════ */
void UpdateMechAngleAndVelocity(uint16_t enc_now)
{
// Inputs:
//   uint16_t enc_now    — raw encoder count from this ISR cycle ∈ [0, 16383]
//
// Outputs:
//   (void — updates module-level globals)
//   pos_mech_rad  ← unwrapped accumulated mechanical angle [rad]
//                   (accumulates freely in (−∞, +∞) over time)
//   vel_mech_rads ← mechanical angular velocity [rad/s] through 1st-order IIR LPF
//
// Static state:
//   static uint16_t enc_prev — previous ISR's raw count (kept inside this module)
//
// Parameters used:
//   ENC_TO_MECH_RAD = 2π / 16384   (count → radians)
//   CTRL_DT         = 1 / 20000    (control period [s])
//   VEL_FILT_ALPHA                 (IIR coefficient — student tunes)
//
// Goal:
//   (1) Unwrapping:
//       The encoder wraps in [0, 16383]. After one revolution it jumps 16383 → 0.
//       Correct this so the mechanical angle accumulates freely over time.
//   (2) Velocity estimation:
//       (position increment this cycle) / (control period) = instantaneous velocity [rad/s].
//       Raw velocity is noisy → smooth with a 1st-order IIR low-pass filter.
//
// Hint:
//   - d_enc = enc_now − enc_prev is normally small under continuous rotation,
//     but jumps to nearly ±16383 when it crosses the wrap boundary.
//   - We assume the motor cannot rotate more than ±8192 counts (half revolution)
//     within 50 μs:
//        d_enc >  +8192  →  forward wrap (16383 → 0).   d_enc -= 16384.
//        d_enc <  −8192  →  reverse wrap (0 → 16383).   d_enc += 16384.
//   - 1st-order IIR LPF: blend the new raw velocity with the previous filtered
//     value, weighted by VEL_FILT_ALPHA.  (See lecture notes for the formula.)
//     Smaller α → smoother but more lag.  α = 1 → no filtering (raw value).
//     Tune VEL_FILT_ALPHA in 0.01~0.1.
//
// Pseudo Code:
//   (1) compute d_enc = enc_now − enc_prev    (signed)
//   (2) unwrap: if |d_enc| exceeds half of one mechanical revolution,
//        correct by adding or subtracting one full revolution
//   (3) save enc_now into enc_prev (for the next ISR)
//   (4) convert d_enc into radians using ENC_TO_MECH_RAD
//   (5) accumulate into pos_mech_rad
//   (6) compute the raw velocity from the position increment over CTRL_DT
//   (7) update vel_mech_rads via 1st-order IIR using VEL_FILT_ALPHA
    enc_now &= 0x3FFFu;

    if (!enc_initialized) {
        enc_prev = enc_now;
        enc_initialized = 1;
        vel_mech_rads = 0.0f;
        return;
    }

    int32_t d_enc = (int32_t)enc_now - (int32_t)enc_prev;

    if (d_enc > 8192) {
        d_enc -= 16384;
    } else if (d_enc < -8192) {
        d_enc += 16384;
    }

    enc_prev = enc_now;

    float d_pos = (float)d_enc * ENC_TO_MECH_RAD;
    pos_mech_rad += d_pos;

    float vel_raw = d_pos / CTRL_DT;
    vel_mech_rads = VEL_FILT_ALPHA * vel_raw +
                    (1.0f - VEL_FILT_ALPHA) * vel_mech_rads;
}

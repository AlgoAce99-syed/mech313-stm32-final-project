/**
 * @file    drv8323.c
 * @brief   DRV8323 gate driver SPI communication and initialization module
 *
 * Students must fill in the body of the functions below.
 * Function signatures and interfaces are fixed in drv8323.h (do not modify).
 *
 *   - DRV8323_SPI_Transfer()    : 16-bit bit-banging SPI transfer (internal helper)
 *   - DRV8323_Write()           : Construct write frame + call Transfer
 *   - DRV8323_Read()            : Construct read  frame + call Transfer
 *   - DRV8323_Init()            : Boot sequence — 5 register magic numbers + CSA cal
 *
 * Datasheet references:
 *   - DRV8323 datasheet section 8.5 (Register Maps) — bit fields of 0x02~0x06
 *   - section 7.6.1 (SPI Communication Protocol)    — 16-bit frame layout
 *
 * Cross-module consistency:
 *   - The CSA_GAIN choice (10 / 20 / 40 / 80) in register 0x06 MUST agree with
 *     CURRENT_SCALE = 1 / (CSA_GAIN × 0.002) in current_adc.c.
 */

#include "drv8323.h"
#include "main.h"


/* ── SPI bit-banging helper — provided (do not modify) ───────────────────── */
/* Short delay (≈ 10 NOPs) between GPIO toggles, to satisfy DRV8323 SPI timing. */
static void DRV_SPI_Delay(void) { for (volatile int i = 0; i < 10; i++) __NOP(); }


/* ════════════════════════════════════════════════════════════════════════════
 *  ── 16-bit Bit-Banging SPI Transfer (static helper) ───────────────────────
 * ════════════════════════════════════════════════════════════════════════════ */
static uint16_t DRV8323_SPI_Transfer(uint16_t tx)
{
// Inputs:
//   uint16_t tx  — 16-bit frame to transmit to the DRV8323
//
// Outputs:
//   uint16_t  — lower 11 bits of the received 16-bit data (mask upper 5 bits)
//               DRV8323 registers are 11 bits wide.
//
// Used HAL APIs:
//   HAL_GPIO_WritePin(drv_cs_GPIO_Port,   drv_cs_Pin,   SET/RESET)   ← CS
//   HAL_GPIO_WritePin(drv_mosi_GPIO_Port, drv_mosi_Pin, SET/RESET)   ← MOSI out
//   HAL_GPIO_WritePin(drv_sclk_GPIO_Port, drv_sclk_Pin, SET/RESET)   ← SCLK toggle
//   HAL_GPIO_ReadPin (drv_miso_GPIO_Port, drv_miso_Pin)              ← MISO in
//   DRV_SPI_Delay()                                                  ← signal settle
//
// Goal:
//   Send and receive one 16-bit SPI frame over 4 GPIO pins (CS, MOSI, MISO, SCLK).
//   Implemented as software bit-banging, NOT using the hardware SPI peripheral.
//
// Hint:
//   - SPI mode: SCLK idle = LOW, data sampled on the SCLK rising edge.
//     Master sets MOSI just before SCLK rises, and samples MISO while SCLK is
//     HIGH (or right after the rising edge).
//   - Transmit order: MSB-first (bit 15 → bit 0).
//   - Insert DRV_SPI_Delay() between GPIO toggles so signals can settle.
//   - CS goes LOW before the transfer and HIGH after it completes.
//
// Pseudo Code:
//   (1) rx = 0;
//   (2) CS LOW;  delay;
//   (3) for i = 15 down to 0:           // MSB first
//          set MOSI according to bit i of tx
//          delay
//          SCLK HIGH
//          delay
//          if MISO is HIGH, set bit i of rx
//          SCLK LOW
//          delay
//   (4) delay;  CS HIGH;  delay;
//   (5) return rx masked to its lower 11 bits;
    uint16_t rx = 0;

    HAL_GPIO_WritePin(drv_cs_GPIO_Port, drv_cs_Pin, GPIO_PIN_RESET);
    DRV_SPI_Delay();

    for (int i = 15; i >= 0; i--) {
        HAL_GPIO_WritePin(drv_mosi_GPIO_Port, drv_mosi_Pin,
                          (tx & (1u << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        DRV_SPI_Delay();

        HAL_GPIO_WritePin(drv_sclk_GPIO_Port, drv_sclk_Pin, GPIO_PIN_SET);
        DRV_SPI_Delay();

        if (HAL_GPIO_ReadPin(drv_miso_GPIO_Port, drv_miso_Pin) == GPIO_PIN_SET) {
            rx |= (uint16_t)(1u << i);
        }

        HAL_GPIO_WritePin(drv_sclk_GPIO_Port, drv_sclk_Pin, GPIO_PIN_RESET);
        DRV_SPI_Delay();
    }

    DRV_SPI_Delay();
    HAL_GPIO_WritePin(drv_cs_GPIO_Port, drv_cs_Pin, GPIO_PIN_SET);
    DRV_SPI_Delay();

    return rx & 0x07FFu;
}


/* ════════════════════════════════════════════════════════════════════════════
 *  ── DRV8323_Write — register write ────────────────────────────────────────
 * ════════════════════════════════════════════════════════════════════════════ */
void DRV8323_Write(uint8_t reg, uint16_t data)
{
// Inputs:
//   uint8_t  reg   — register address ∈ [0x00, 0x0F]   (4-bit used)
//   uint16_t data  — payload to write   ∈ [0x000, 0x7FF] (11-bit used)
//
// Outputs:
//   (none — write only)
//
// Goal:
//   Construct a 16-bit write frame and send it via DRV8323_SPI_Transfer.
//
// Hint:
//   - DRV8323 SPI frame layout (datasheet section 7.6.1):
//
//        bit  15   14  13  12  11   10  9  8  7  6  5  4  3  2  1  0
//             ┌──┬─────────────┬─────────────────────────────────────┐
//             │RW│  REG (4b)   │            DATA (11b)               │
//             └──┴─────────────┴─────────────────────────────────────┘
//
//     RW = 0  : write
//     RW = 1  : read
//
//   - Use the layout above to position reg and data within the 16-bit frame.
//   - Mask reg to its 4-bit field and data to its 11-bit field for safety.
//   - Add HAL_Delay(1) after the transfer so the DRV8323 has time to process.
//
// Pseudo Code:
//   (1) construct the 16-bit write frame from reg and data
//        (RW = 0; use the layout diagram in the Hint)
//   (2) DRV8323_SPI_Transfer(frame);
//   (3) HAL_Delay(1);
    uint16_t frame = (uint16_t)(((uint16_t)(reg & 0x0Fu) << 11) |
                                (data & 0x07FFu));
    (void)DRV8323_SPI_Transfer(frame);
    HAL_Delay(1);
}


/* ════════════════════════════════════════════════════════════════════════════
 *  ── DRV8323_Read — register read ──────────────────────────────────────────
 * ════════════════════════════════════════════════════════════════════════════ */
uint16_t DRV8323_Read(uint8_t reg)
{
// Inputs:
//   uint8_t reg  — register address ∈ [0x00, 0x0F]
//
// Outputs:
//   uint16_t  — 11-bit contents of the requested register
//
// Goal:
//   Construct a 16-bit read frame, call DRV8323_SPI_Transfer, and return the value.
//
// Hint:
//   - Same frame layout as Write, but RW = 1.
//   - Construct the read frame by setting RW (bit 15) and placing reg at
//     bits 14-11. The DATA field is don't-care (use 0).
//   - DRV8323 returns the response *in the same transfer* (no delayed response,
//     unlike the AS5047P encoder).
//
// Pseudo Code:
//   (1) construct the 16-bit read frame from reg
//        (RW = 1; use the layout diagram in the Hint)
//   (2) return DRV8323_SPI_Transfer(frame);
    uint16_t frame = (uint16_t)(0x8000u | ((uint16_t)(reg & 0x0Fu) << 11));
    return DRV8323_SPI_Transfer(frame);
}


/* ════════════════════════════════════════════════════════════════════════════
 *  ── DRV8323_Init — boot sequence ──────────────────────────────────────────
 * ════════════════════════════════════════════════════════════════════════════ */
void DRV8323_Init(void)
{
// Inputs:
//   (none)
//
// Outputs:
//   (none — configures DRV8323 internal registers + enables output)
//
// Used HAL APIs:
//   HAL_GPIO_WritePin(motor_enable_GPIO_Port, motor_enable_Pin, SET/RESET)
//     → ENABLE pin (LOW = sleep, HIGH = active)
//   HAL_GPIO_WritePin(motor_hiz_GPIO_Port, motor_hiz_Pin, SET)
//     → HIZ pin (LOW = output high-impedance, HIGH = enable PWM outputs)
//   HAL_Delay(ms)
//   DRV8323_Write(reg, data) / DRV8323_Read(reg)   ← defined in this module
//
// Goal:
//   Wake the DRV8323 from sleep, configure operating mode, run CSA calibration,
//   clear faults, and enable outputs. Call once from main().
//
// Register configuration — refer to datasheet (section 8.5) for each bit field:
//
//   ┌─────┬──────────────────────────────────────────────────────────────────┐
//   │ Reg │ What to set                                                       │
//   ├─────┼──────────────────────────────────────────────────────────────────┤
//   │ 0x02│ Driver Control          → PWM_MODE = 3x PWM                      │
//   │ 0x03│ Gate Drive HS           → choose IDRIVEP_HS / IDRIVEN_HS         │
//   │ 0x04│ Gate Drive LS           → TDRIVE, IDRIVEP_LS / IDRIVEN_LS        │
//   │ 0x05│ OCP Control             → DEAD_TIME, OCP_MODE, VDS_LVL           │
//   │ 0x06│ CSA Control             → CSA_GAIN (★), SEN_LVL, VREF_DIV         │
//   └─────┴──────────────────────────────────────────────────────────────────┘
//
//   ★ The CSA_GAIN choice MUST match CURRENT_SCALE in current_adc.c:
//        CSA_GAIN = 20 V/V  →  CURRENT_SCALE = 1/(20 × 0.002) = 25 A/V
//        CSA_GAIN = 40 V/V  →  CURRENT_SCALE = 12.5 A/V
//
// CSA offset calibration — CAL bits in 0x06 (bit4: A, bit3: B, bit2: C):
//   r = Read(0x06)
//   Write(0x06, r | 0x001C);    // CAL on  (CSA1+CSA2+CSA3)
//   HAL_Delay(1);                // wait
//   Write(0x06, r & ~0x001C);   // CAL off
//
// Pseudo Code:
//   (1) Toggle ENABLE for hardware reset:
//        motor_enable = LOW;  HAL_Delay(5);
//        motor_enable = HIGH; HAL_Delay(5);
//   (2) Write 5 configuration registers (values from datasheet):
//        DRV8323_Write(0x02,  <PWM_MODE = 3x PWM>      );
//        DRV8323_Write(0x03,  <HS gate drive setting>  );
//        DRV8323_Write(0x04,  <LS gate, TDRIVE setting>);
//        DRV8323_Write(0x05,  <OCP, VDS_LVL setting>   );
//        DRV8323_Write(0x06,  <CSA_GAIN, SEN_LVL setting>);
//   (3) CSA offset calibration:
//        reg6 = DRV8323_Read(0x06);
//        DRV8323_Write(0x06, reg6 | 0x001C);
//        HAL_Delay(1);
//        DRV8323_Write(0x06, reg6 & ~0x001C);
//   (4) Read fault registers to clear them:
//        (void)DRV8323_Read(0x00);   // Fault Status 1
//        (void)DRV8323_Read(0x01);   // Fault Status 2
//   (5) Drive HIZ HIGH to enable gate driver outputs:
//        motor_hiz = HIGH;
//        HAL_Delay(1);
    const uint16_t reg_driver_control = 0x0020u; /* 3x PWM mode */
    const uint16_t reg_gate_hs        = 0x0322u; /* unlocked, 60mA source, 120mA sink */
    const uint16_t reg_gate_ls        = 0x0722u; /* CBC, 4us TDRIVE, 60mA source, 120mA sink */
    const uint16_t reg_ocp            = 0x0159u; /* 100ns dead time, auto retry, 4us deg, 0.75V */
    const uint16_t reg_csa            = 0x0283u; /* VREF/2, CSA_GAIN=20V/V, SEN_LVL=1V */

    HAL_GPIO_WritePin(motor_hiz_GPIO_Port, motor_hiz_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(drv_cs_GPIO_Port, drv_cs_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(drv_sclk_GPIO_Port, drv_sclk_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(motor_enable_GPIO_Port, motor_enable_Pin, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(motor_enable_GPIO_Port, motor_enable_Pin, GPIO_PIN_SET);
    HAL_Delay(5);

    DRV8323_Write(0x02, reg_driver_control);
    DRV8323_Write(0x03, reg_gate_hs);
    DRV8323_Write(0x04, reg_gate_ls);
    DRV8323_Write(0x05, reg_ocp);
    DRV8323_Write(0x06, reg_csa);

    uint16_t reg6 = DRV8323_Read(0x06);
    DRV8323_Write(0x06, reg6 | 0x001Cu);
    HAL_Delay(1);
    DRV8323_Write(0x06, (uint16_t)(reg6 & ~0x001Cu));

    (void)DRV8323_Read(0x00);
    (void)DRV8323_Read(0x01);

    HAL_GPIO_WritePin(motor_hiz_GPIO_Port, motor_hiz_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
}

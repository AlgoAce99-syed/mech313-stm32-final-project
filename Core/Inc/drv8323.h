/**
 * @file    drv8323.h
 * @brief   DRV8323 gate driver SPI communication and initialization module
 *
 * This header file is provided complete. Do not modify.
 * For implementation logic, algorithms, and pseudo code, refer to drv8323.c.
 *
 * Module overview:
 *   - DRV8323_Write():   16-bit bit-banging SPI write (register, data)
 *   - DRV8323_Read():    16-bit bit-banging SPI read  (register → value)
 *   - DRV8323_Init():    Toggle ENABLE → write registers 0x02–0x06 →
 *                        CSA offset calibration → clear faults → release HIZ
 *
 * Notes:
 *   - SPI is bit-banged over GPIO (not the hardware SPI peripheral).
 *     SPI1 is reserved for the encoder; DRV8323 uses 4 separate GPIO pins.
 *   - DRV8323_SPI_Transfer / DRV_SPI_Delay are static helpers inside .c.
 */

#ifndef DRV8323_H
#define DRV8323_H

#include <stdint.h>

/* ── Functions ───────────────────────────────────────────────────────────── */

/**
 * Write 16-bit SPI frame to a DRV8323 register.
 *
 * @param   reg    register address ∈ [0x00, 0x0F]
 * @param   data   11-bit payload (upper 5 bits are ignored)
 */
void     DRV8323_Write(uint8_t reg, uint16_t data);

/**
 * Read 16-bit SPI frame from a DRV8323 register.
 *
 * @param   reg    register address ∈ [0x00, 0x0F]
 * @return  11-bit register contents (upper 5 bits zero)
 */
uint16_t DRV8323_Read (uint8_t reg);

/**
 * DRV8323 boot sequence — call once from main().
 *
 *  1. Toggle ENABLE pin for hardware reset
 *  2. Write operating mode / gate / OCP / CSA settings to registers 0x02–0x06
 *  3. CSA offset calibration (set→wait→clear CAL bits in 0x06)
 *  4. Read fault status registers 0x00, 0x01 to clear
 *  5. Drive HIZ pin HIGH to enable gate driver output
 *
 * @note  The CSA_GAIN setting in register 0x06 MUST match
 *        CURRENT_SCALE = 1 / (CSA_GAIN × shunt) in current_adc.c.
 */
void     DRV8323_Init (void);

#endif /* DRV8323_H */

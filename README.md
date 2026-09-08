# MECH313 Final Project — BLDC Motor Cascade FOC

Implement field-oriented control (FOC) for a 3-phase brushless DC motor on an
STM32G474 + DRV8323 board. You will write the algorithms inside six C modules;
the framework (HAL init, ISR scaffolding, controller gains) is provided.

---

## 1. What you'll build

A position-controllable BLDC motor that runs three cascaded control loops at
**20 kHz** inside an ADC interrupt:

```
                              ADC ISR — every 50 μs
                                       │
        ┌──────────────────────────────┼──────────────────────────────┐
        ▼                              ▼                              ▼
   current_adc                     encoder                       cascade_foc
   (read 3 ADCs)            (SPI, angle, velocity)        (Clarke → Park → PD →
                                                          PI → PI → invPark →
                                                          invClarke → SVPWM)
                                                                      │
                                                                      ▼
                                                                  svpwm
                                                            (write TIM2 CCRs)
                                                                      │
                                                                      ▼
                                                                  DRV8323
                                                              (3-phase output)
                                                                      │
                                                                      ▼
                                                                    Motor
```

Three control modes selected by the runtime variable `ctrl_mode`:

| Mode | Active loops | Command variable |
|---|---|---|
| `CTRL_CURRENT`  | current PI only          | `iq_ref_cmd` |
| `CTRL_VELOCITY` | velocity PI → current PI | `vel_ref_cmd` |
| `CTRL_POSITION` | position PD → velocity PI → current PI | `pos_ref_cmd` |

---

## 2. Hardware

- **MCU**: STM32G474 (Cortex-M4 @ 170 MHz, FPU)
- **Gate driver**: DRV8323 (3-phase, 2 mΩ shunts, integrated CSA)
- **Motor**: 3-phase BLDC, **7 pole pairs** (SPMSM, Ld ≈ Lq)
- **Encoder**: AS5047P magnetic, 14-bit absolute, SPI
- **Bus**: 24 V DC
- **Debugger**: ST-Link (provides STM32CubeIDE debugging + Live Expressions)

---

## 3. Files you must implement

**Headers (`Core/Inc/*.h`) are provided complete — DO NOT modify them.**

You will write the bodies of functions in **six C files** under `Core/Src/`:

| File | Functions to implement | Constants to set |
|---|---|---|
| `controller.c`  | `PI_Init`, `PI_Update`, `PI_Reset`, `PD_Init`, `PD_Update`, `PD_Reset` | — |
| `svpwm.c`       | `ClampCCR`, `SVPWM_Output`                                            | — |
| `encoder.c`     | `Encoder_ReadAngle`, `ElecThetaFromEncoder`, `UpdateMechAngleAndVelocity` | `encoder_elec_zero`, `VEL_FILT_ALPHA` |
| `current_adc.c` | `ADC_MeasureOffset`, `ADC_ReadCurrents`                               | `CURRENT_SCALE` |
| `drv8323.c`     | `DRV8323_SPI_Transfer`, `DRV8323_Write`, `DRV8323_Read`, `DRV8323_Init` | 5 register values (0x02–0x06) |
| `cascade_foc.c` | `UpdateCascadeFOC`                                                    | — |

**Files you must NOT modify** (framework — touching them breaks the project):

```
Core/Inc/*.h                              ← all headers
Core/Src/main.h, stm32g4xx_it.c, etc.     ← CubeMX boilerplate
Core/Src/encoder_calibration.c            ← calibration helper, already complete
CMakeLists.txt                            ← except the documented swap
moteus_test.ioc                           ← modify the code directly instead of using CubeMX to make the hardware configuration changes.
```

**Files you MAY selectively modify** (only where noted):

```
Core/Src/main.c                           ← integration code, gains already set
```

You can tune gains, add monitoring (UART printf, debug output), or add peripheral features (e.g. LED blink) and other custom functionality.
DO NOT modify control flow, peripheral init, or ISR structure.

---

## 4. Recommended work order (≈ 3 effective weeks)

The work order matches the **5 validation steps** in section 7. The project
window spans more calendar weeks, but conference / exam weeks are excluded
from the working schedule below.

| Week | Steps | Modules | What you should have working |
|---|---|---|---|
| **W1** | Step 1 + Step 2 | `drv8323.c`, `svpwm.c`, `current_adc.c` | Gate driver wakes up, PWM outputs correct duty, current readings sane |
| **W2** | Step 3 + Step 4 | `encoder.c` (+ calibration), `controller.c`, `cascade_foc.c` (CURRENT mode) | Angle reads correctly, current loop tracks `iq_ref_cmd` |
| **W3** | Step 5 + Report | `cascade_foc.c` (POSITION mode) | Motor holds and tracks position commands; report drafted |

> Office-hours support is available during the working weeks. The
> conference and exam weeks in between are **self-paced** — finish as much
> of Steps 1–4 as you can before those weeks.

---

## 5. Per-file work guide

Every `.c` file you edit has the same comment layout inside each function:

```c
// Inputs:                ← what the function receives
// Outputs:               ← what it returns / what globals it updates
// Used HAL APIs:         ← which library calls you'll likely need
// Goal:                  ← what the function must accomplish
// Hint:                  ← directional advice
// Pseudo Code:           ← numbered, language-neutral steps
```

Pseudo code is **language-neutral** — it tells you the steps, not the math.
**Math formulas live in the lecture notes (PPT).** You translate the steps
into C using the formulas you learned.

---

### 5.1 `controller.c` — PI / PD primitives

**Purpose**: generic single-axis PI and PD controllers, reused by the
cascade as four instances (id PI, iq PI, velocity PI, position PD).

| Function | Lines | Difficulty |
|---|---|---|
| `PI_Init`   | ~ line 29  | Trivial — store fields, zero integrator |
| `PI_Update` | ~ line 48  | **Core** — integrate, anti-windup clamp, output saturation |
| `PI_Reset`  | ~ line 85  | Trivial — zero the integrator |
| `PD_Init`   | ~ line 103 | Trivial |
| `PD_Update` | ~ line 121 | **Core** — finite-difference derivative, output saturation |
| `PD_Reset`  | ~ line 161 | Trivial |

**Key concepts**: anti-windup (clamp integrator BEFORE summing with P term),
order of `prev_error` update in PD (compute derivative first, THEN save).

---

### 5.2 `svpwm.c` — 3-phase voltage → TIM2 CCR

**Purpose**: convert commanded 3-phase voltages (from the FOC chain) into
TIM2 capture/compare register values, with zero-sequence injection so the
PWM uses the DC bus efficiently.

| Function | Lines | Difficulty |
|---|---|---|
| `ClampCCR`     | ~ line 42 | Easy — saturate `int32_t` into `[0, arr]` |
| `SVPWM_Output` | ~ line 71 | **Core** — min/max → zero-seq offset → CCR scaling → write timer |

**Key concepts**: SVPWM via zero-sequence injection — the motor sees only
line-to-line voltage, so adding a common offset to all three phases is free.
The formula for the offset is in the lecture notes.

---

### 5.3 `encoder.c` — AS5047P + angle/velocity estimation

**Purpose**: read the 14-bit magnetic encoder over SPI, convert raw counts
to electrical angle (used by Park transform), and accumulate the unwrapped
mechanical angle + IIR-filtered velocity.

| Item | Lines | Difficulty |
|---|---|---|
| `VEL_FILT_ALPHA` macro     | line 25  | Tune in 0.01~0.1 (start at 0.03) |
| `encoder_elec_zero` const  | line 31  | **Measure via calibration** (see section 6) |
| `Encoder_ReadAngle`        | ~ line 44  | SPI 2-transfer protocol (AS5047P quirk) |
| `ElecThetaFromEncoder`     | ~ line 89  | Modulo math over one electrical cycle |
| `UpdateMechAngleAndVelocity` | ~ line 129 | **Core** — wrap detection + IIR LPF |

**Key concepts**: AS5047P uses *delayed response* (first transfer is the
command, second carries the data). Mechanical angle must be unwrapped at
the 16383 → 0 boundary so it accumulates freely.

---

### 5.4 `current_adc.c` — 3-phase current measurement

**Purpose**: at boot, measure each ADC channel's zero-current offset; every
ISR cycle, read the data registers and convert raw counts to amperes.

| Item | Lines | Difficulty |
|---|---|---|
| `CURRENT_SCALE` macro | line 30 | **Compute** from CSA_GAIN × 2 mΩ shunt |
| `ADC_MeasureOffset`   | ~ line 44 | EXTEN-toggle trick + 64-sample average |
| `ADC_ReadCurrents`    | ~ line 108 | Direct DR read + (raw − zero) × scale chain |

**Key concepts**: ADC is normally externally triggered by TIM2 (for the ISR
mode), but offset measurement must use software triggering — temporarily
clear EXTEN and restore it. Channel mapping is `ADC1 → C`, `ADC2 → B`,
`ADC3 → A`.

> ⚠ `CURRENT_SCALE` here MUST match the CSA_GAIN value you choose in
> `drv8323.c`'s register 0x06. They're two sides of the same physical chain.

---

### 5.5 `drv8323.c` — gate driver bring-up

**Purpose**: bit-bang 16-bit SPI to configure the DRV8323 at boot —
PWM mode, gate drive strength, OCP, and the current-sense amplifier (CSA).

| Function | Lines | Difficulty |
|---|---|---|
| `DRV8323_SPI_Transfer` | ~ line 34 | **Core** — GPIO bit-bang, MSB-first |
| `DRV8323_Write`        | ~ line 81 | Frame construction (RW=0) |
| `DRV8323_Read`         | ~ line 119 | Frame construction (RW=1) |
| `DRV8323_Init`         | ~ line 147 | **Core** — 5 register values + CSA cal sequence |

**Key concepts**: bit-bang SPI requires careful toggling of CS / MOSI / SCLK
with small delays. The five magic-number register values must be derived
from the DRV8323 datasheet (section 8.5 — Register Maps).

> ⚠ Your choice of `CSA_GAIN` in register `0x06` determines `CURRENT_SCALE`
> in `current_adc.c`. They MUST match.

---

### 5.6 `cascade_foc.c` — the FOC composition

**Purpose**: one function that, every 50 μs, transforms measurements into
the dq frame, runs the active control loops, transforms back to 3-phase,
and calls `SVPWM_Output`. This is the crown of the project.

| Function | Lines | Difficulty |
|---|---|---|
| `UpdateCascadeFOC` | ~ line 49 | **Hard** — 14 args, 8 stages, 3 control modes |

The 8 stages (see Pseudo Code in the file):

```
[1] Clarke           : (i_A, i_B)           → (i_α, i_β)
[2] Park             : (i_α, i_β, θ)         → (id, iq)
[3] Outer loop branch by ctrl_mode:
      POSITION   : PD_pos produces vel_ref*
      VELOCITY   : vel_ref* = vel_ref_cmd  (reset PD_pos)
      CURRENT    : reset PD_pos & PI_vel; iq_ref* = iq_ref_cmd
[4] Velocity loop (skip if CURRENT) : PI_vel produces iq_ref*
[5] Current loop     : PI_d → vd, PI_q → vq
[6] Inverse Park     : (vd, vq, θ)           → (v_α, v_β)
[7] Inverse Clarke   : (v_α, v_β)            → (v_a, v_b, v_c)
[8] SVPWM_Output(va, vb, vc)
```

**Key concepts**: coordinate transforms (Clarke/Park and their inverses
— lecture notes), mode-dependent loop activation, *resetting* the
controllers of inactive loops to avoid windup transients when modes change.

---

## 6. Encoder calibration (one-time, before Step 3)

The encoder needs to know "what raw count is electrical angle 0°".
`encoder_calibration.c` measures this for you — but you must build it
as a separate program.

**Prerequisites**: Step 1 passes (DRV8323 + SVPWM working), and
`Encoder_ReadAngle()` is implemented.

**Procedure**:

1. Open `CMakeLists.txt`. In the `target_sources(...)` block:
   ```cmake
   # Core/Src/main.c                 ← comment OUT
   Core/Src/encoder_calibration.c   ← uncomment
   ```
2. Build → flash → reset the board.
3. The rotor will twitch and lock into its d-axis equilibrium position
   (a soft "thunk" sound is normal).
4. Open the debugger (STM32CubeIDE):
   - Set a breakpoint at the `while(1) {}` in `encoder_calibration.c`
   - Read the value of `elec_zero_result` (e.g., `2289`)
5. Open `encoder.c` line 31 and paste that value:
   ```c
   const uint16_t encoder_elec_zero = 2289;   // your measured value
   ```
6. Reverse step 1 — comment out `encoder_calibration.c`, uncomment `main.c`.
7. Build → flash. Normal FOC operation resumes.

**Sanity check** after calibration: slowly turn the rotor by hand and watch
`elec_theta_enc` in Live Expressions. It should sweep 0 → 2π **seven
times** per full mechanical revolution (7 pole pairs).

---

## 7. Safety

- **Bus voltage is 24 V** — won't shock you, but a 5 A short can burn the
  shunts or melt cabling. Always pause and check before powering up.
- **If the motor screams, vibrates violently, or smells/smokes:
  immediately disconnect VBUS.** Common causes:
    - Wrong `CURRENT_SCALE` (current-loop gain effectively too high)
    - Wrong `encoder_elec_zero` (torque applied at q-axis instead of d-axis)
    - Wrong DRV8323 register value (gate drive timing too aggressive)
- **Never hold the motor by the shaft** while it's energized in
  CTRL_POSITION mode — `POS_VEL_LIMIT = 30 rad/s` is fast enough to
  injure fingers.
- **Check `drv_fault` pin** before assuming the controller is the
  problem — if `drv_fault` is LOW, the gate driver has tripped and
  needs a power cycle.

---

## 8. Build environment

- STM32CubeIDE 1.19.0 (Recommended for use with VSCode extensions)
- Build with CMake (`CMakePresets.json` provided)
- Toolchain: `arm-none-eabi-gcc` (bundled with CubeIDE)
- Debugger: ST-Link

---

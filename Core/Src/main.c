/* USER CODE BEGIN Header */
/**
 * @file    main.c
 * @brief   Integration code — top-level orchestration of the FOC system
 *
 * This file is provided MOSTLY COMPLETE. Students should not need to modify
 * it under normal use. The peripheral init, controller wiring, and ISR
 * scaffolding are all set up.
 *
 * What students may want to do here (optional):
 *   - Change ctrl_mode and the reference commands (pos_ref_cmd, vel_ref_cmd,
 *     iq_ref_cmd) — either via the debugger (Live Expressions) or by writing
 *     a sequence in the main loop (see commented-out demo in while(1)).
 *   - Slightly adjust the safety limits VEL_IQ_LIMIT / POS_VEL_LIMIT if needed.
 *
 * The PI / PD gains below have been pre-tuned for the BLDC on this board.
 * You do NOT need to retune them. (Analysis of the step response and the
 * meaning of each gain belongs in the project report.)
 *
 * ISR architecture — every 50 μs:
 *   TIM2 TRGO  →  ADC1 EOC  →  HAL_ADC_ConvCpltCallback()
 *      [1] ADC_ReadCurrents()
 *      [2] Encoder_ReadAngle() → ElecThetaFromEncoder() → UpdateMechAngleAndVelocity()
 *      [3] UpdateCascadeFOC()
 *
 * Safety:
 *   If the motor screams, shakes violently, or smells/smokes — IMMEDIATELY
 *   disconnect VBUS (24 V supply). Common causes: wrong CURRENT_SCALE, wrong
 *   encoder_elec_zero, wrong DRV8323 register values.
 */
/* USER CODE END Header */

#include "main.h"
#include "controller.h"
#include "svpwm.h"
#include "encoder.h"
#include "current_adc.h"
#include "drv8323.h"
#include "cascade_foc.h"
#include <math.h>

/* ── Macros ─────────────────────────────────────────────────────────────── */
#define CTRL_FREQ       20000.0f
#define CTRL_DT         (1.0f / CTRL_FREQ)    /* 50 μs */

#define VBUS            24.0f                 /* External DC bus voltage [V]  */
#define VMAX_INV        (VBUS * 0.57735f)     /* VBUS / √3, prevents overvoltage */

#define ENC_TO_MECH_RAD (2.0f * (float)M_PI / 16384.0f)

#define CURRENT_POLARITY_TEST_ENABLE  0u
#define VELOCITY_TEST_ENABLE          0u
#define POSITION_TEST_ENABLE          0u

/* ── Current PI gain ─────────────────────────────────────────────────────── */
/* Pre-tuned for the motor on this board. Same gain on d/q axes
   (SPMSM with Ld ≈ Lq). */
#if CURRENT_POLARITY_TEST_ENABLE || VELOCITY_TEST_ENABLE || POSITION_TEST_ENABLE
#define CUR_KP          1.0f
#define CUR_KI          50.0f
#define CUR_V_LIMIT     1.0f
#else
#define CUR_KP          0.025f
#define CUR_KI          40.0f
#define CUR_V_LIMIT     VMAX_INV
#endif

/* ── Velocity PI gain ────────────────────────────────────────────────────── */
/* I-gain is disabled (= 0): the velocity loop relies on the P term only. */
#define VEL_KP          0.4f
#define VEL_KI          0.0f
#define VEL_IQ_LIMIT    5.0f                  /* iq saturation [A] */

/* ── Position PD gain ────────────────────────────────────────────────────── */
#if POSITION_TEST_ENABLE
#define POS_KP          8.0f
#define POS_KD          0.05f
#define POS_VEL_LIMIT   8.0f                  /* velocity saturation [rad/s] */
#else
#define POS_KP          70.0f
#define POS_KD          0.01f
#define POS_VEL_LIMIT   30.0f                 /* velocity saturation [rad/s] */
#endif

/* One-shot open-loop power-stage smoke test. */
#define POWER_STAGE_TEST_ENABLE       0u
#define POWER_STAGE_TEST_ARM_DELAY_MS 8000u
#define POWER_STAGE_TEST_PULSE_MS     5000u
#define POWER_STAGE_TEST_VD           1.0f

/* Known-good open-loop pulse used as a proof that the flashed image is running. */
#define PREFLIGHT_TEST_ENABLE         0u
#define PREFLIGHT_PULSE_MS            2000u
#define PREFLIGHT_GAP_MS              2000u

/* Hold zero PWM forever after initialization; useful after any suspicious power event. */
#define NO_DRIVE_SAFETY_ENABLE        1u

/* One-direction velocity-mode smoke test. */
#define VELOCITY_TEST_REF_RAD_S       8.0f
#define VELOCITY_TEST_ARM_MS          500u
#define VELOCITY_TEST_GAP_MS          500u
#define VELOCITY_TEST_SAMPLES         100000u

/* One-direction position-mode hold test. */
#define POSITION_TEST_STEP_RAD        15.0f
#define POSITION_TEST_ARM_MS          500u
#define POSITION_TEST_SETTLE_MS       8000u
#define POSITION_TEST_SAMPLES         20000u

/* Long-duration q-axis current hold test. */
#define CURRENT_POLARITY_IQ_A         0.20f
#define CURRENT_POLARITY_ARM_MS       500u
#define CURRENT_POLARITY_GAP_MS       300u
#define CURRENT_POLARITY_SAMPLES      100000u

/* One-shot current-loop smoke test. */
#define CURRENT_TEST_ENABLE           0u
#define CURRENT_TEST_ARM_DELAY_MS     3000u
#define CURRENT_TEST_PULSE_MS         5000u
#define CURRENT_TEST_GAP_MS           2000u
#define CURRENT_TEST_IQ_A             0.20f
#define ISR_ALIVE_MIN_COUNT           1000u

/* ── Peripheral handles ──────────────────────────────────────────────────── */
ADC_HandleTypeDef   hadc1, hadc2, hadc3;
SPI_HandleTypeDef   hspi1;
TIM_HandleTypeDef   htim2;
#ifdef HAL_UART_MODULE_ENABLED
UART_HandleTypeDef  huart1;
#endif

/* USER CODE BEGIN PV */

/* ── PWM timer parameters ────────────────────────────────────────────────── */
uint32_t pwm_period = 4249;   /* ARR value (TIM2 auto-reload register)              */
uint32_t pwm_center = 2125;   /* ARR/2, 50% duty reference for center-aligned PWM */

/* ── User command interface ──────────────────────────────────────────────── */
/* Change these at runtime (via debugger Live Expressions) to drive the motor. */
volatile CtrlMode ctrl_mode   = CTRL_CURRENT;  /* default: safest mode at boot */
volatile float    iq_ref_cmd  = 0.0f;          /* q-axis current command [A]   */
volatile float    id_ref      = 0.0f;          /* d-axis current command [A] (SPMSM: 0) */
volatile float    vel_ref_cmd = 0.0f;          /* velocity command [rad/s]     */
volatile float    pos_ref_cmd = 0.0f;          /* position command [rad]       */

/* Debug breadcrumbs for bring-up. */
volatile uint32_t debug_isr_count = 0u;
volatile uint32_t debug_ccr1 = 0u, debug_ccr2 = 0u, debug_ccr3 = 0u;
volatile uint8_t  debug_current_test_stage = 0u;
volatile uint8_t  debug_drv_fault_pin = 1u;
volatile uint8_t  debug_force_zero_current_feedback = 0u;

#if CURRENT_POLARITY_TEST_ENABLE
volatile uint8_t  debug_polarity_active = 0u;
volatile uint8_t  debug_polarity_phase = 0u;
volatile uint32_t debug_polarity_count = 0u;
volatile uint32_t debug_polarity_fault_count = 0u;
volatile float    debug_polarity_id_sum = 0.0f;
volatile float    debug_polarity_iq_sum = 0.0f;
volatile float    debug_polarity_vd_sum = 0.0f;
volatile float    debug_polarity_vq_sum = 0.0f;
volatile float    debug_polarity_plus_id_avg = 0.0f;
volatile float    debug_polarity_plus_iq_avg = 0.0f;
volatile float    debug_polarity_plus_vd_avg = 0.0f;
volatile float    debug_polarity_plus_vq_avg = 0.0f;
volatile float    debug_polarity_minus_id_avg = 0.0f;
volatile float    debug_polarity_minus_iq_avg = 0.0f;
volatile float    debug_polarity_minus_vd_avg = 0.0f;
volatile float    debug_polarity_minus_vq_avg = 0.0f;
volatile uint32_t debug_polarity_plus_faults = 0u;
volatile uint32_t debug_polarity_minus_faults = 0u;
#endif

#if VELOCITY_TEST_ENABLE
volatile uint8_t  debug_velocity_active = 0u;
volatile uint8_t  debug_velocity_phase = 0u;
volatile uint32_t debug_velocity_count = 0u;
volatile uint32_t debug_velocity_fault_count = 0u;
volatile float    debug_velocity_vel_sum = 0.0f;
volatile float    debug_velocity_iq_sum = 0.0f;
volatile float    debug_velocity_id_sum = 0.0f;
volatile float    debug_velocity_vq_sum = 0.0f;
volatile float    debug_velocity_plus_vel_avg = 0.0f;
volatile float    debug_velocity_plus_iq_avg = 0.0f;
volatile float    debug_velocity_plus_id_avg = 0.0f;
volatile float    debug_velocity_plus_vq_avg = 0.0f;
volatile float    debug_velocity_minus_vel_avg = 0.0f;
volatile float    debug_velocity_minus_iq_avg = 0.0f;
volatile float    debug_velocity_minus_id_avg = 0.0f;
volatile float    debug_velocity_minus_vq_avg = 0.0f;
volatile uint32_t debug_velocity_plus_faults = 0u;
volatile uint32_t debug_velocity_minus_faults = 0u;
#endif

#if POSITION_TEST_ENABLE
volatile uint8_t  debug_position_active = 0u;
volatile uint32_t debug_position_count = 0u;
volatile uint32_t debug_position_fault_count = 0u;
volatile float    debug_position_start_pos = 0.0f;
volatile float    debug_position_target_pos = 0.0f;
volatile float    debug_position_pos_sum = 0.0f;
volatile float    debug_position_vel_sum = 0.0f;
volatile float    debug_position_err_sum = 0.0f;
volatile float    debug_position_iq_sum = 0.0f;
volatile float    debug_position_id_sum = 0.0f;
volatile float    debug_position_vd_sum = 0.0f;
volatile float    debug_position_vq_sum = 0.0f;
volatile float    debug_position_pos_avg = 0.0f;
volatile float    debug_position_vel_avg = 0.0f;
volatile float    debug_position_err_avg = 0.0f;
volatile float    debug_position_iq_avg = 0.0f;
volatile float    debug_position_id_avg = 0.0f;
volatile float    debug_position_vd_avg = 0.0f;
volatile float    debug_position_vq_avg = 0.0f;
volatile uint32_t debug_position_faults = 0u;
#endif

/* ── Controller instances ────────────────────────────────────────────────── */
static SimplePI pi_d;
static SimplePI pi_q;
static SimplePI pi_vel;
static SimplePD pd_pos;

/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
#ifdef HAL_UART_MODULE_ENABLED
static void MX_USART1_UART_Init(void);
#endif

/* USER CODE BEGIN 0 */

/* Initialize all four controllers with the gain macros defined above. */
static void Controllers_Init(void)
{
    PI_Init(&pi_d,   CUR_KP, CUR_KI, CTRL_DT, CUR_V_LIMIT);  /* d-axis current PI */
    PI_Init(&pi_q,   CUR_KP, CUR_KI, CTRL_DT, CUR_V_LIMIT);  /* q-axis current PI */
    PI_Init(&pi_vel, VEL_KP, VEL_KI, CTRL_DT, VEL_IQ_LIMIT); /* velocity PI       */
    PD_Init(&pd_pos, POS_KP, POS_KD, POS_VEL_LIMIT);         /* position PD       */
}

#if CURRENT_POLARITY_TEST_ENABLE
static void CurrentPolarity_ResetWindow(uint8_t phase)
{
    debug_polarity_phase = phase;
    debug_polarity_count = 0u;
    debug_polarity_fault_count = 0u;
    debug_polarity_id_sum = 0.0f;
    debug_polarity_iq_sum = 0.0f;
    debug_polarity_vd_sum = 0.0f;
    debug_polarity_vq_sum = 0.0f;
    debug_polarity_active = 1u;
}

static void CurrentPolarity_StoreWindow(void)
{
    float n = (float)debug_polarity_count;

    if (debug_polarity_phase == 1u) {
        debug_polarity_plus_id_avg = debug_polarity_id_sum / n;
        debug_polarity_plus_iq_avg = debug_polarity_iq_sum / n;
        debug_polarity_plus_vd_avg = debug_polarity_vd_sum / n;
        debug_polarity_plus_vq_avg = debug_polarity_vq_sum / n;
        debug_polarity_plus_faults = debug_polarity_fault_count;
    } else if (debug_polarity_phase == 2u) {
        debug_polarity_minus_id_avg = debug_polarity_id_sum / n;
        debug_polarity_minus_iq_avg = debug_polarity_iq_sum / n;
        debug_polarity_minus_vd_avg = debug_polarity_vd_sum / n;
        debug_polarity_minus_vq_avg = debug_polarity_vq_sum / n;
        debug_polarity_minus_faults = debug_polarity_fault_count;
    }

    debug_polarity_active = 0u;
}

static void CurrentPolarity_DoneHook(void)
{
    __asm volatile ("nop");
}
#endif

#if VELOCITY_TEST_ENABLE
static void VelocityTest_ResetWindow(uint8_t phase)
{
    debug_velocity_phase = phase;
    debug_velocity_count = 0u;
    debug_velocity_fault_count = 0u;
    debug_velocity_vel_sum = 0.0f;
    debug_velocity_iq_sum = 0.0f;
    debug_velocity_id_sum = 0.0f;
    debug_velocity_vq_sum = 0.0f;
    debug_velocity_active = 1u;
}

static void VelocityTest_StoreWindow(void)
{
    float n = (float)debug_velocity_count;

    if (debug_velocity_phase == 1u) {
        debug_velocity_plus_vel_avg = debug_velocity_vel_sum / n;
        debug_velocity_plus_iq_avg = debug_velocity_iq_sum / n;
        debug_velocity_plus_id_avg = debug_velocity_id_sum / n;
        debug_velocity_plus_vq_avg = debug_velocity_vq_sum / n;
        debug_velocity_plus_faults = debug_velocity_fault_count;
    } else if (debug_velocity_phase == 2u) {
        debug_velocity_minus_vel_avg = debug_velocity_vel_sum / n;
        debug_velocity_minus_iq_avg = debug_velocity_iq_sum / n;
        debug_velocity_minus_id_avg = debug_velocity_id_sum / n;
        debug_velocity_minus_vq_avg = debug_velocity_vq_sum / n;
        debug_velocity_minus_faults = debug_velocity_fault_count;
    }

    debug_velocity_active = 0u;
}

static void VelocityTest_DoneHook(void)
{
    __asm volatile ("nop");
}
#endif

#if POSITION_TEST_ENABLE
static void PositionTest_ResetWindow(void)
{
    debug_position_count = 0u;
    debug_position_fault_count = 0u;
    debug_position_pos_sum = 0.0f;
    debug_position_vel_sum = 0.0f;
    debug_position_err_sum = 0.0f;
    debug_position_iq_sum = 0.0f;
    debug_position_id_sum = 0.0f;
    debug_position_vd_sum = 0.0f;
    debug_position_vq_sum = 0.0f;
    debug_position_active = 1u;
}

static void PositionTest_StoreWindow(void)
{
    float n = (float)debug_position_count;

    debug_position_pos_avg = debug_position_pos_sum / n;
    debug_position_vel_avg = debug_position_vel_sum / n;
    debug_position_err_avg = debug_position_err_sum / n;
    debug_position_iq_avg = debug_position_iq_sum / n;
    debug_position_id_avg = debug_position_id_sum / n;
    debug_position_vd_avg = debug_position_vd_sum / n;
    debug_position_vq_avg = debug_position_vq_sum / n;
    debug_position_faults = debug_position_fault_count;
    debug_position_active = 0u;
}

static void PositionTest_DoneHook(void)
{
    __asm volatile ("nop");
}
#endif

/* USER CODE END 0 */

/* ── main ────────────────────────────────────────────────────────────────── */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_TIM2_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_ADC3_Init();
#ifdef HAL_UART_MODULE_ENABLED
    MX_USART1_UART_Init();
#endif

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);

    DRV8323_Init();           /* bring up the gate driver                       */
    ADC_MeasureOffset();      /* zero-current ADC offset calibration            */
    Controllers_Init();       /* PI / PD instances                              */

    /* Initial encoder read → mechanical / electrical angle */
    encoder_raw    = Encoder_ReadAngle();
    elec_theta_enc = ElecThetaFromEncoder(encoder_raw, encoder_elec_zero);
    pos_mech_rad   = (float)encoder_raw * ENC_TO_MECH_RAD;
    pos_ref_cmd    = 0.0f;
    vel_mech_rads  = 0.0f;

    /* Set all phases to 50% duty (zero voltage) and start PWM channels */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_center);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_center);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm_center);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);   /* CH4 is the ADC trigger via TRGO */

#if NO_DRIVE_SAFETY_ENABLE
    while (1) {
        debug_current_test_stage = 90u;
        SVPWM_Output(0.0f, 0.0f, 0.0f);
        debug_drv_fault_pin = (HAL_GPIO_ReadPin(drv_fault_GPIO_Port, drv_fault_Pin) == GPIO_PIN_SET) ? 1u : 0u;
        debug_ccr1 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
        debug_ccr2 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);
        debug_ccr3 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_3);
        HAL_Delay(10u);
    }
#endif

#if PREFLIGHT_TEST_ENABLE
    while (1) {
        debug_current_test_stage = 10u;
        SVPWM_Output(0.0f, 0.0f, 0.0f);
        HAL_Delay(PREFLIGHT_GAP_MS);

        debug_current_test_stage = 11u;
        SVPWM_Output(POWER_STAGE_TEST_VD,
                     -0.5f * POWER_STAGE_TEST_VD,
                     -0.5f * POWER_STAGE_TEST_VD);
        HAL_Delay(PREFLIGHT_PULSE_MS);
    }
#endif

#if POWER_STAGE_TEST_ENABLE
    /*
     * Direct PWM/DRV/motor test: bypass the ADC ISR and FOC loop.
     * Applies a fixed d-axis voltage vector, then returns to zero voltage.
     */
    ctrl_mode = CTRL_CURRENT;
    id_ref = 0.0f;
    iq_ref_cmd = 0.0f;
    HAL_Delay(POWER_STAGE_TEST_ARM_DELAY_MS);

    SVPWM_Output(POWER_STAGE_TEST_VD,
                 -0.5f * POWER_STAGE_TEST_VD,
                 -0.5f * POWER_STAGE_TEST_VD);
    HAL_Delay(POWER_STAGE_TEST_PULSE_MS);

    SVPWM_Output(0.0f, 0.0f, 0.0f);
    while (1) {}
#endif

    /* Enable ADC ISR — ADC2/ADC3 are slaves; ADC1 is the master that fires the ISR */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
    HAL_ADC_Start(&hadc2);
    HAL_ADC_Start(&hadc3);
    HAL_ADC_Start_IT(&hadc1);

#if VELOCITY_TEST_ENABLE
    debug_current_test_stage = 70u;
    ctrl_mode = CTRL_VELOCITY;
    id_ref = 0.0f;
    iq_ref_cmd = 0.0f;
    vel_ref_cmd = 0.0f;
    PI_Reset(&pi_d);
    PI_Reset(&pi_q);
    PI_Reset(&pi_vel);
    HAL_Delay(VELOCITY_TEST_ARM_MS);

    debug_current_test_stage = 71u;
    vel_ref_cmd = VELOCITY_TEST_REF_RAD_S;
    PI_Reset(&pi_vel);
    VelocityTest_ResetWindow(1u);
    while (debug_velocity_active != 0u) {}

    vel_ref_cmd = 0.0f;
    id_ref = 0.0f;
    iq_ref_cmd = 0.0f;
    HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
    SVPWM_Output(0.0f, 0.0f, 0.0f);
    debug_ccr1 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
    debug_ccr2 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);
    debug_ccr3 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_3);
    debug_current_test_stage = 73u;
    VelocityTest_DoneHook();
    while (1) {
        SVPWM_Output(0.0f, 0.0f, 0.0f);
    }
#endif

#if POSITION_TEST_ENABLE
    debug_current_test_stage = 80u;
    ctrl_mode = CTRL_POSITION;
    id_ref = 0.0f;
    iq_ref_cmd = 0.0f;
    vel_ref_cmd = 0.0f;
    pos_ref_cmd = pos_mech_rad;
    PI_Reset(&pi_d);
    PI_Reset(&pi_q);
    PI_Reset(&pi_vel);
    PD_Reset(&pd_pos);
    HAL_Delay(POSITION_TEST_ARM_MS);

    debug_position_start_pos = pos_mech_rad;
    debug_position_target_pos = debug_position_start_pos + POSITION_TEST_STEP_RAD;
    pos_ref_cmd = debug_position_target_pos;
    debug_current_test_stage = 81u;
    HAL_Delay(POSITION_TEST_SETTLE_MS);

    debug_current_test_stage = 82u;
    PositionTest_ResetWindow();
    while (debug_position_active != 0u) {}

    pos_ref_cmd = pos_mech_rad;
    vel_ref_cmd = 0.0f;
    id_ref = 0.0f;
    iq_ref_cmd = 0.0f;
    HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
    SVPWM_Output(0.0f, 0.0f, 0.0f);
    debug_ccr1 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
    debug_ccr2 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);
    debug_ccr3 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_3);
    debug_current_test_stage = 83u;
    PositionTest_DoneHook();
    while (1) {
        SVPWM_Output(0.0f, 0.0f, 0.0f);
    }
#endif

#if CURRENT_POLARITY_TEST_ENABLE
    debug_current_test_stage = 60u;
    ctrl_mode = CTRL_CURRENT;
    id_ref = 0.0f;
    iq_ref_cmd = 0.0f;
    PI_Reset(&pi_d);
    PI_Reset(&pi_q);
    HAL_Delay(CURRENT_POLARITY_ARM_MS);

    debug_current_test_stage = 61u;
    id_ref = 0.0f;
    iq_ref_cmd = CURRENT_POLARITY_IQ_A;
    PI_Reset(&pi_d);
    PI_Reset(&pi_q);
    CurrentPolarity_ResetWindow(1u);
    while (debug_polarity_active != 0u) {}

    id_ref = 0.0f;
    iq_ref_cmd = 0.0f;
    PI_Reset(&pi_d);
    PI_Reset(&pi_q);
    HAL_Delay(CURRENT_POLARITY_GAP_MS);

    id_ref = 0.0f;
    iq_ref_cmd = 0.0f;
    HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
    SVPWM_Output(0.0f, 0.0f, 0.0f);
    debug_ccr1 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
    debug_ccr2 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);
    debug_ccr3 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_3);
    debug_current_test_stage = 63u;
    CurrentPolarity_DoneHook();
    while (1) {
        SVPWM_Output(0.0f, 0.0f, 0.0f);
    }
#endif

#if CURRENT_TEST_ENABLE
    debug_current_test_stage = 1u;
    ctrl_mode = CTRL_CURRENT;
    id_ref = 0.0f;
    iq_ref_cmd = 0.0f;
    HAL_Delay(CURRENT_TEST_ARM_DELAY_MS);

    if (debug_isr_count < ISR_ALIVE_MIN_COUNT) {
        debug_current_test_stage = 9u;
        SVPWM_Output(POWER_STAGE_TEST_VD,
                     -0.5f * POWER_STAGE_TEST_VD,
                     -0.5f * POWER_STAGE_TEST_VD);
        HAL_Delay(POWER_STAGE_TEST_PULSE_MS);
        SVPWM_Output(0.0f, 0.0f, 0.0f);
        while (1) {}
    }

    debug_current_test_stage = 2u;
    debug_force_zero_current_feedback = 0u;
    iq_ref_cmd = CURRENT_TEST_IQ_A;
    HAL_Delay(CURRENT_TEST_PULSE_MS);

    iq_ref_cmd = 0.0f;
    HAL_Delay(CURRENT_TEST_GAP_MS);
    PI_Reset(&pi_d);
    PI_Reset(&pi_q);

    debug_current_test_stage = 4u;
    debug_force_zero_current_feedback = 1u;
    iq_ref_cmd = CURRENT_TEST_IQ_A;
    HAL_Delay(CURRENT_TEST_PULSE_MS);

    iq_ref_cmd = 0.0f;
    debug_force_zero_current_feedback = 0u;
    debug_current_test_stage = 3u;
#endif

    while (1) {
        /* Default: empty.  Use the debugger Live Expressions to set ctrl_mode,
         * pos_ref_cmd, iq_ref_cmd, etc. while the system is running.
         * 
         * TODO: Implement UART monitoring using HAL_UART_Transmit().
         *   - Variables to monitor: pos_mech_rad, vel_mech_rads, current_A, current_B, etc.
         *
         * OPTIONAL — automatic step demo for Gate 5 (uncomment to use):
         *
         *   ctrl_mode   = CTRL_POSITION;
         *   pos_ref_cmd =  (float)M_PI / 2.0f;
         *   HAL_Delay(2000);
         *   pos_ref_cmd = -(float)M_PI / 2.0f;
         *   HAL_Delay(2000);
         */
    }
}

/* USER CODE BEGIN 4 */

/**
 * 20 kHz control ISR.
 * Fired automatically by ADC1 EOC, which itself is triggered by TIM2 TRGO.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance != ADC1) return;

    debug_isr_count++;
    debug_drv_fault_pin = (HAL_GPIO_ReadPin(drv_fault_GPIO_Port, drv_fault_Pin) == GPIO_PIN_SET) ? 1u : 0u;

    /* [1] Read ADC → phase currents */
    ADC_ReadCurrents();


    /* [2] Read encoder → electrical angle + mechanical angle / velocity */
    encoder_raw    = Encoder_ReadAngle();
    elec_theta_enc = ElecThetaFromEncoder(encoder_raw, encoder_elec_zero);
    UpdateMechAngleAndVelocity(encoder_raw);

    /* [3] Cascade FOC — produces 3-phase voltages and writes the TIM2 CCRs */
    float foc_current_A = (debug_force_zero_current_feedback != 0u) ? 0.0f : current_A;
    float foc_current_B = (debug_force_zero_current_feedback != 0u) ? 0.0f : current_B;

    UpdateCascadeFOC(foc_current_A, foc_current_B,
                     elec_theta_enc,
                     pos_mech_rad, vel_mech_rads,
                     pos_ref_cmd, vel_ref_cmd, iq_ref_cmd,
                     id_ref, ctrl_mode,
                     &pi_d, &pi_q, &pi_vel, &pd_pos,
                     CTRL_DT);

    debug_ccr1 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
    debug_ccr2 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);
    debug_ccr3 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_3);

#if CURRENT_POLARITY_TEST_ENABLE
    if (debug_polarity_active != 0u) {
        debug_polarity_id_sum += id_meas;
        debug_polarity_iq_sum += iq_meas;
        debug_polarity_vd_sum += vd_out;
        debug_polarity_vq_sum += vq_out;

        if (debug_drv_fault_pin == 0u) {
            debug_polarity_fault_count++;
        }

        debug_polarity_count++;
        if (debug_polarity_count >= CURRENT_POLARITY_SAMPLES) {
            CurrentPolarity_StoreWindow();
        }
    }
#endif

#if VELOCITY_TEST_ENABLE
    if (debug_velocity_active != 0u) {
        debug_velocity_vel_sum += vel_mech_rads;
        debug_velocity_iq_sum += iq_meas;
        debug_velocity_id_sum += id_meas;
        debug_velocity_vq_sum += vq_out;

        if (debug_drv_fault_pin == 0u) {
            debug_velocity_fault_count++;
        }

        debug_velocity_count++;
        if (debug_velocity_count >= VELOCITY_TEST_SAMPLES) {
            VelocityTest_StoreWindow();
        }
    }
#endif

#if POSITION_TEST_ENABLE
    if (debug_position_active != 0u) {
        debug_position_pos_sum += pos_mech_rad;
        debug_position_vel_sum += vel_mech_rads;
        debug_position_err_sum += pos_ref_cmd - pos_mech_rad;
        debug_position_iq_sum += iq_meas;
        debug_position_id_sum += id_meas;
        debug_position_vd_sum += vd_out;
        debug_position_vq_sum += vq_out;

        if (debug_drv_fault_pin == 0u) {
            debug_position_fault_count++;
        }

        debug_position_count++;
        if (debug_position_count >= POSITION_TEST_SAMPLES) {
            PositionTest_StoreWindow();
        }
    }
#endif

}

/* USER CODE END 4 */

/* ── Peripherals initialization (CubeMX auto-generated, do not modify) ───── */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = RCC_PLLM_DIV4;
    RCC_OscInitStruct.PLL.PLLN            = 85;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                     |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

static void MX_ADC1_Init(void)
{
    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef sConfig = {0};
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler=ADC_CLOCK_SYNC_PCLK_DIV4; hadc1.Init.Resolution=ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign=ADC_DATAALIGN_RIGHT; hadc1.Init.GainCompensation=0;
    hadc1.Init.ScanConvMode=ADC_SCAN_DISABLE; hadc1.Init.EOCSelection=ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait=DISABLE; hadc1.Init.ContinuousConvMode=DISABLE;
    hadc1.Init.NbrOfConversion=1; hadc1.Init.DiscontinuousConvMode=DISABLE;
    hadc1.Init.ExternalTrigConv=ADC_EXTERNALTRIG_T2_TRGO;
    hadc1.Init.ExternalTrigConvEdge=ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.DMAContinuousRequests=DISABLE; hadc1.Init.Overrun=ADC_OVR_DATA_PRESERVED;
    hadc1.Init.OversamplingMode=DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();
    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) Error_Handler();
    sConfig.Channel=ADC_CHANNEL_4; sConfig.Rank=ADC_REGULAR_RANK_1;
    sConfig.SamplingTime=ADC_SAMPLETIME_12CYCLES_5; sConfig.SingleDiff=ADC_SINGLE_ENDED;
    sConfig.OffsetNumber=ADC_OFFSET_NONE; sConfig.Offset=0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_ADC2_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    hadc2.Instance = ADC2;
    hadc2.Init.ClockPrescaler=ADC_CLOCK_SYNC_PCLK_DIV4; hadc2.Init.Resolution=ADC_RESOLUTION_12B;
    hadc2.Init.DataAlign=ADC_DATAALIGN_RIGHT; hadc2.Init.GainCompensation=0;
    hadc2.Init.ScanConvMode=ADC_SCAN_DISABLE; hadc2.Init.EOCSelection=ADC_EOC_SINGLE_CONV;
    hadc2.Init.LowPowerAutoWait=DISABLE; hadc2.Init.ContinuousConvMode=DISABLE;
    hadc2.Init.NbrOfConversion=1; hadc2.Init.DiscontinuousConvMode=DISABLE;
    hadc2.Init.ExternalTrigConv=ADC_EXTERNALTRIG_T2_TRGO;
    hadc2.Init.ExternalTrigConvEdge=ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc2.Init.DMAContinuousRequests=DISABLE; hadc2.Init.Overrun=ADC_OVR_DATA_PRESERVED;
    hadc2.Init.OversamplingMode=DISABLE;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) Error_Handler();
    sConfig.Channel=ADC_CHANNEL_3; sConfig.Rank=ADC_REGULAR_RANK_1;
    sConfig.SamplingTime=ADC_SAMPLETIME_12CYCLES_5; sConfig.SingleDiff=ADC_SINGLE_ENDED;
    sConfig.OffsetNumber=ADC_OFFSET_NONE; sConfig.Offset=0;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_ADC3_Init(void)
{
    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef sConfig = {0};
    hadc3.Instance = ADC3;
    hadc3.Init.ClockPrescaler=ADC_CLOCK_SYNC_PCLK_DIV4; hadc3.Init.Resolution=ADC_RESOLUTION_12B;
    hadc3.Init.DataAlign=ADC_DATAALIGN_RIGHT; hadc3.Init.GainCompensation=0;
    hadc3.Init.ScanConvMode=ADC_SCAN_DISABLE; hadc3.Init.EOCSelection=ADC_EOC_SINGLE_CONV;
    hadc3.Init.LowPowerAutoWait=DISABLE; hadc3.Init.ContinuousConvMode=DISABLE;
    hadc3.Init.NbrOfConversion=1; hadc3.Init.DiscontinuousConvMode=DISABLE;
    hadc3.Init.ExternalTrigConv=ADC_EXTERNALTRIG_T2_TRGO;
    hadc3.Init.ExternalTrigConvEdge=ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc3.Init.DMAContinuousRequests=DISABLE; hadc3.Init.Overrun=ADC_OVR_DATA_PRESERVED;
    hadc3.Init.OversamplingMode=DISABLE;
    if (HAL_ADC_Init(&hadc3) != HAL_OK) Error_Handler();
    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK) Error_Handler();
    sConfig.Channel=ADC_CHANNEL_1; sConfig.Rank=ADC_REGULAR_RANK_1;
    sConfig.SamplingTime=ADC_SAMPLETIME_12CYCLES_5; sConfig.SingleDiff=ADC_SINGLE_ENDED;
    sConfig.OffsetNumber=ADC_OFFSET_NONE; sConfig.Offset=0;
    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_SPI1_Init(void)
{
    hspi1.Instance=SPI1; hspi1.Init.Mode=SPI_MODE_MASTER;
    hspi1.Init.Direction=SPI_DIRECTION_2LINES; hspi1.Init.DataSize=SPI_DATASIZE_16BIT;
    hspi1.Init.CLKPolarity=SPI_POLARITY_LOW; hspi1.Init.CLKPhase=SPI_PHASE_2EDGE;
    hspi1.Init.NSS=SPI_NSS_SOFT; hspi1.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit=SPI_FIRSTBIT_MSB; hspi1.Init.TIMode=SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE; hspi1.Init.CRCPolynomial=7;
    hspi1.Init.CRCLength=SPI_CRC_LENGTH_DATASIZE; hspi1.Init.NSSPMode=SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

#ifdef HAL_UART_MODULE_ENABLED
static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
    if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
    if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
    if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) Error_Handler();
}
#endif

static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};
    TIM_OC_InitTypeDef      sConfigOC          = {0};
    htim2.Instance=TIM2; htim2.Init.Prescaler=0;
    htim2.Init.CounterMode=TIM_COUNTERMODE_CENTERALIGNED2; htim2.Init.Period=4249;
    htim2.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) Error_Handler();
    sMasterConfig.MasterOutputTrigger=TIM_TRGO_OC4REF;
    sMasterConfig.MasterSlaveMode=TIM_MASTERSLAVEMODE_ENABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
    sConfigOC.OCMode=TIM_OCMODE_PWM1; sConfigOC.Pulse=0;
    sConfigOC.OCPolarity=TIM_OCPOLARITY_HIGH; sConfigOC.OCFastMode=TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
    sConfigOC.OCMode=TIM_OCMODE_PWM2; sConfigOC.Pulse=4248;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) Error_Handler();
    HAL_TIM_MspPostInit(&htim2);
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOC, drv_mosi_Pin|motor_enable_Pin|motor_hiz_Pin|drv_sclk_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, drv_cs_Pin|GPIO_PIN_2, GPIO_PIN_SET);
    GPIO_InitStruct.Pin=drv_mosi_Pin|motor_enable_Pin|motor_hiz_Pin;
    GPIO_InitStruct.Mode=GPIO_MODE_OUTPUT_PP; GPIO_InitStruct.Pull=GPIO_NOPULL;
    GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    GPIO_InitStruct.Pin=drv_cs_Pin; GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(drv_cs_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin=GPIO_PIN_2; GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin=drv_fault_Pin; GPIO_InitStruct.Mode=GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull=GPIO_NOPULL;
    HAL_GPIO_Init(drv_fault_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin=drv_sclk_Pin; GPIO_InitStruct.Mode=GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(drv_sclk_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin=drv_miso_Pin; GPIO_InitStruct.Mode=GPIO_MODE_INPUT;
    HAL_GPIO_Init(drv_miso_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void) { __disable_irq(); while (1) {} }

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif

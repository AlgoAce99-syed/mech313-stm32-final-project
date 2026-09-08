/* USER CODE BEGIN Header */
/**
 * @file    main.c
 * @brief   Electrical angle zero calibration for encoder
 *
 * [Calibration procedure]
 *   1. Apply fixed d-axis voltage (Vd) with Vq=0 and electrical angle θ=0
 *   2. Wait for rotor to align to d-axis
 *   3. Read encoder raw value (averaged over multiple samples)
 *   4. Check elec_zero_result via debugger and write to encoder_elec_zero in encoder.c
 */
/* USER CODE END Header */

#include "main.h"
#include "svpwm.h"
#include "encoder.h"
#include "drv8323.h"
#include <math.h>

/* ── Macros ──────────────────────────────────────────────────────────────── */
#define VBUS              24.0f
#define CALIB_VD          2.0f    /* d-axis applied voltage [V] */
#define CALIB_WAIT_MS     1000    /* rotor convergence wait time [ms] */
#define CALIB_AVG_SAMPLES 64      /* number of encoder samples to average */

/* ── Peripheral handles ──────────────────────────────────────────────────── */
ADC_HandleTypeDef   hadc1, hadc2, hadc3;
SPI_HandleTypeDef   hspi1;
TIM_HandleTypeDef   htim2;

/* ── PWM timer parameters ────────────────────────────────────────────────── */
uint32_t pwm_period = 4249;
uint32_t pwm_center = 2125;

/* USER CODE BEGIN PV */

volatile uint16_t elec_zero_result = 0;   /* calibration result */
volatile uint16_t calib_raw_before = 0;
volatile uint16_t calib_raw_after  = 0;
volatile int16_t  calib_delta_counts = 0;

/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);

/* USER CODE BEGIN 0 */

/* Apply fixed d-axis voltage with θ=0:
 * cos(0)=1, sin(0)=0 → va=Vd, vb=vc=-Vd/2 */
static void ApplyDAxisVoltage(float vd)
{
    float va =  vd;
    float vb = -vd * 0.5f;
    float vc = -vd * 0.5f;
    SVPWM_Output(va, vb, vc);
}

/* Read encoder angle and return average over n_samples */
static uint16_t Encoder_ReadAverage(uint32_t n_samples)
{
    uint32_t sum = 0;
    for (uint32_t i = 0; i < n_samples; i++) {
        sum += Encoder_ReadAngle();
        HAL_Delay(1);
    }
    return (uint16_t)((sum / n_samples) & 0x3FFF);
}

static int16_t Encoder_DeltaCounts(uint16_t before, uint16_t after)
{
    int32_t delta = (int32_t)after - (int32_t)before;
    if (delta > 8192) {
        delta -= 16384;
    } else if (delta < -8192) {
        delta += 16384;
    }
    return (int16_t)delta;
}

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

    DRV8323_Init();

    /* Start PWM output at 50% duty cycle */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_center);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_center);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm_center);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    HAL_Delay(100);   /* wait for DRV8323 to stabilize */
    calib_raw_before = Encoder_ReadAverage(CALIB_AVG_SAMPLES);

    /* Apply d-axis voltage → wait for rotor to converge → read encoder */
    ApplyDAxisVoltage(CALIB_VD);
    HAL_Delay(CALIB_WAIT_MS);

    calib_raw_after = Encoder_ReadAverage(CALIB_AVG_SAMPLES);
    calib_delta_counts = Encoder_DeltaCounts(calib_raw_before, calib_raw_after);
    elec_zero_result = calib_raw_after;

    /* Return PWM to 50% duty (zero voltage) */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_center);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_center);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm_center);

    /*
     * How to read the result:
     *   Set a breakpoint at while(1) and read elec_zero_result in the debugger,
     *   or use Live Watch to observe the value.
     *   Write the result to encoder_elec_zero in encoder.c:
     *
     *   Example: const uint16_t encoder_elec_zero = 2422;
     */
    while (1) {}
}

/* ── Peripherals initialization ──────────────────────────────────────────── */
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

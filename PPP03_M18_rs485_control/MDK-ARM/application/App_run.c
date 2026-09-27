#include "App_run.h"
#include "port.h"

/* TIM4 prescaler 719 at 72 MHz: each count is 10 us. */
#define ENCODER_TIMEOUT_MS 600U
#define M2_ARM_WINDOW_MS 5000U
/* TIM2 prescaler 7199, ARR 499: one update every 50 ms. */
#define M4_NO_ENCODER_TICKS 8U
/* M5: 20 TIM2 updates is a strict one-second experiment limit. */
#define M2_MAX_PULSE_TICKS 20U
/* M11 stopped at F59 then coasted by K2 in three S20 unloaded trials.
   Advance the shutdown by two edges; verify actual angle before any load. */
#define M12_TARGET_EDGES 57U
/* Keep tracking encoder coast briefly after shutdown; the bridge remains off. */
#define M11_COAST_WINDOW_MS 500U
#define M5_OVERSPEED_RPM 45U
#define M5_OVERSPEED_SAMPLES 3U
#define M4_MIN_TARGET_RPM 15U
#define M4_MAX_TARGET_RPM 25U
#define M4_DEFAULT_TARGET_RPM 20U
#define M4_RESTART_GAP_MS 2000U
/* TIM4 ticks at 100 kHz: 60*100000/(11*21.428) = 25455.2. */
#define M4_RPM_FROM_PERIOD 25455U
/* The marked disc is on the encoder/motor-shaft end of the gearbox motor.
   Manual turns gave ~11 rising edges per encoder-disc revolution.
   The vendor's 21.428 reduction factor converts rotor to output-shaft RPM. */
#define ENCODER_EDGES_PER_MOTOR_REV 11.0f
#define GEAR_REDUCTION_RATIO 21.428f

float speed_now = 0.0f;
static volatile uint32_t encoder_period_ticks = 0;
static volatile uint32_t encoder_last_capture_ms = 0;
static volatile uint32_t encoder_capture_count = 0;
/* Absolute edge count never resets; the calibration uses an atomic baseline. */
static volatile uint32_t encoder_cal_start_count = 0;
static volatile uint8_t encoder_primed = 0;
static volatile uint8_t encoder_period_valid = 0;
static volatile M2_Status m2_status = M2_READY;
static volatile uint8_t m2_cleanup_pending = 0;
static uint32_t m2_arm_time_ms = 0;
static uint32_t m2_start_capture_count = 0;
static volatile uint8_t m2_pulse_ticks = 0;
static volatile uint32_t m3_drive_edges = 0;
/* Edge count latched immediately after the bridge shutdown command.
   D (m3_drive_edges) may subsequently grow during the coast window. */
static volatile uint32_t m10_cutoff_edges = 0;
static volatile uint8_t m3_drive_ticks = 0;
static volatile uint8_t m4_target_rpm = M4_DEFAULT_TARGET_RPM;
static volatile uint16_t m4_last_compare = 500U;
static uint32_t m4_last_edge_count = 0U;
static uint8_t m4_no_edge_ticks = 0U;
static uint8_t m5_overspeed_samples = 0U;
static volatile uint32_t m4_last_stop_ms = 0U;
static volatile uint8_t m4_has_stopped = 0U;
/* PA1/ADC1_IN1 reads YS_CUR. The supplied schematic shows a reference,
   but its revision may differ from the physical board. Raw ADC counts
   have not been calibrated into amperes. */
#define M7_ADC_SAMPLES 16U
static uint8_t m6_adc_ready = 0U;
static volatile uint16_t m6_adc_raw = 0U;
static uint16_t m6_adc_baseline = 0U;
static volatile uint16_t m6_peak_delta = 0U;
/* M8: signed average of samples 11..20 (0.55..1.00 s) relative to
   the baseline read immediately before the bridge is enabled. */
#define M8_STEADY_FIRST_TICK 11U
static int32_t m8_steady_sum = 0;
static uint8_t m8_steady_count = 0U;
static volatile int16_t m8_steady_mean_delta = 0;

static uint8_t App_M6_ReadAdc(uint16_t *raw)
{
    uint32_t tries;
    if (!m6_adc_ready)
    {
        return 0U;
    }
    ADC1->SR = 0U;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    /* A bounded wait works from both main and the 50 ms timer ISR. */
    for (tries = 0U; tries < 20000U; ++tries)
    {
        if ((ADC1->SR & ADC_SR_EOC) != 0U)
        {
            *raw = (uint16_t)(ADC1->DR & 0xFFFU);
            return 1U;
        }
    }
    return 0U;
}

/* Average a short group spanning multiple TIM1 PWM periods. With the
   239.5-cycle ADC sample time at 12 MHz, 16 readings take < 1 ms on a
   working ADC. The same averaging is used for the baseline and run data. */
static uint8_t App_M7_ReadMean(uint16_t *mean)
{
    uint32_t sum = 0U;
    uint8_t i;
    uint16_t raw;
    for (i = 0U; i < M7_ADC_SAMPLES; ++i)
    {
        if (!App_M6_ReadAdc(&raw)) return 0U;
        sum += raw;
    }
    *mean = (uint16_t)((sum + M7_ADC_SAMPLES / 2U) / M7_ADC_SAMPLES);
    return 1U;
}

static uint8_t App_M6_AdcInit(void)
{
    GPIO_InitTypeDef pin = {0};
    uint32_t start_ms;
    uint16_t first_raw;

    /* APB2 is 72 MHz: ADC clock = APB2/6 = 12 MHz. */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_ADC_CONFIG(RCC_ADCPCLK2_DIV6);
    __HAL_RCC_ADC1_CLK_ENABLE();
    pin.Pin = GPIO_PIN_1;
    pin.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &pin);
    ADC1->CR1 = 0U;
    ADC1->CR2 = ADC_CR2_EXTSEL | ADC_CR2_EXTTRIG | ADC_CR2_ADON;
    ADC1->SQR1 = 0U; /* One regular conversion. */
    ADC1->SQR3 = 1U; /* First conversion: channel 1 on PA1. */
    ADC1->SMPR2 = 7U << ADC_SMPR2_SMP1_Pos; /* 239.5 ADC cycles. */
    HAL_Delay(1U);
    ADC1->CR2 |= ADC_CR2_RSTCAL;
    start_ms = HAL_GetTick();
    while ((ADC1->CR2 & ADC_CR2_RSTCAL) != 0U)
    {
        if ((uint32_t)(HAL_GetTick() - start_ms) > 10U) return 0U;
    }
    ADC1->CR2 |= ADC_CR2_CAL;
    start_ms = HAL_GetTick();
    while ((ADC1->CR2 & ADC_CR2_CAL) != 0U)
    {
        if ((uint32_t)(HAL_GetTick() - start_ms) > 10U) return 0U;
    }
    m6_adc_ready = 1U;
    if (!App_M7_ReadMean(&first_raw))
    {
        m6_adc_ready = 0U;
        return 0U;
    }
    m6_adc_raw = first_raw;
    return 1U;
}

static void App_M4_FinishPulse(M2_Status status)
{
    /* De-energize before updating UI or recorded diagnostics. */
    Int_Motor_EmergencyStop();
    m10_cutoff_edges = encoder_capture_count - m2_start_capture_count;
    m3_drive_edges = m10_cutoff_edges;
    m3_drive_ticks = m2_pulse_ticks;
    m2_status = status;
    m2_cleanup_pending = 1U;
    m4_last_stop_ms = HAL_GetTick();
    m4_has_stopped = 1U;
}

static void App_M2_GetSpeed(void)
{
    uint32_t period = encoder_period_ticks;
    uint32_t age_ms = HAL_GetTick() - encoder_last_capture_ms;
    if (encoder_period_valid && period != 0U && age_ms <= ENCODER_TIMEOUT_MS)
    {
        speed_now = 60.0f * 100000.0f /
                    ((float)period * ENCODER_EDGES_PER_MOTOR_REV *
                     GEAR_REDUCTION_RATIO);
    }
    else
    {
        speed_now = 0.0f;
    }
    REG_INPUT_BUF[3] = (uint16_t)speed_now;
}

void App_Run_Init(void)
{
    Int_Motor_Init();
    if (!App_M6_AdcInit())
    {
        Int_Motor_EmergencyStop();
        m2_status = M2_FAULT;
        return;
    }
    if (HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1) != HAL_OK)
    {
        Int_Motor_EmergencyStop();
        m2_status = M2_FAULT;
    }
}

void App_M2_EmergencyStop(void)
{
    if (m2_status == M2_RUNNING)
    {
        App_M4_FinishPulse(M2_STOPPED);
        return;
    }
    Int_Motor_EmergencyStop();
    if (m2_status != M2_FAULT && m2_status != M2_OVERSPEED)
    {
        m2_status = M2_STOPPED;
    }
    m2_cleanup_pending = 1U;
    m4_last_stop_ms = HAL_GetTick();
    m4_has_stopped = 1U;
}

void App_M2_ArmOrStart(void)
{
    uint32_t now = HAL_GetTick();
    uint16_t baseline;
    if (m2_status == M2_RUNNING || m2_status == M2_FAULT ||
        m2_status == M2_OVERSPEED)
    {
        return;
    }
    if (m4_has_stopped && (uint32_t)(now - m4_last_stop_ms) < M4_RESTART_GAP_MS)
    {
        return;
    }
    if (m2_status != M2_ARMED || (uint32_t)(now - m2_arm_time_ms) > M2_ARM_WINDOW_MS)
    {
        m2_status = M2_ARMED;
        m2_arm_time_ms = now;
        return;
    }

    /* Never allow a prior stop's cleanup to cancel the next pulse. */
    if (m2_cleanup_pending)
    {
        Int_Motor_Stop();
        m2_cleanup_pending = 0U;
    }
    if (!App_M7_ReadMean(&baseline))
    {
        Int_Motor_EmergencyStop();
        m2_status = M2_FAULT;
        return;
    }
    m6_adc_baseline = baseline;
    m6_adc_raw = baseline;
    m6_peak_delta = 0U;
    m8_steady_sum = 0;
    m8_steady_count = 0U;
    m8_steady_mean_delta = 0;
    m2_pulse_ticks = 0U;
    m2_start_capture_count = encoder_capture_count;
    m4_last_edge_count = m2_start_capture_count;
    m4_no_edge_ticks = 0U;
    m5_overspeed_samples = 0U;
    m4_last_compare = M2_TEST_COMPARE;
    m3_drive_edges = 0U;
    m10_cutoff_edges = 0U;
    m3_drive_ticks = 0U;
    __disable_irq();
    m2_status = M2_RUNNING;
    if (!Int_Motor_TestPulse() ||
        HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_RESET)
    {
        Int_Motor_EmergencyStop();
        m4_last_compare = 500U;
        m2_status = M2_FAULT;
        m2_cleanup_pending = 1U;
    }
    __enable_irq();
}

void App_M2_Update(void)
{
    uint16_t raw;
    App_M2_GetSpeed();
    if (m2_status != M2_RUNNING && m6_adc_ready)
    {
        if (App_M7_ReadMean(&raw))
        {
            m6_adc_raw = raw;
        }
        else
        {
            Int_Motor_EmergencyStop();
            m2_status = M2_FAULT;
        }
    }
    if (m2_status == M2_ARMED &&
        (uint32_t)(HAL_GetTick() - m2_arm_time_ms) > M2_ARM_WINDOW_MS)
    {
        m2_status = M2_READY;
    }
    if (m2_cleanup_pending)
    {
        Int_Motor_Stop();
        m2_cleanup_pending = 0U;
    }
}

M2_Status App_M2_GetStatus(void)
{
    return m2_status;
}

void App_M3_ResetPulseCount(void)
{
    /* Ignore KEY2 while the bridge is active or a pulse is armed. */
    if (m2_status != M2_RUNNING && m2_status != M2_ARMED)
    {
        encoder_cal_start_count = encoder_capture_count;
        m3_drive_edges = 0U;
        m10_cutoff_edges = 0U;
        m3_drive_ticks = 0U;
    }
}

uint32_t App_M3_GetPulseCount(void)
{
    return encoder_capture_count - encoder_cal_start_count;
}

uint32_t App_M3_GetDriveEdges(void)
{
    return m3_drive_edges;
}

uint32_t App_M10_GetCutoffEdges(void)
{
    return m10_cutoff_edges;
}

/* Take one coherent UI snapshot while the capture ISR may update D.
   This main-loop-only critical section is just two aligned 32-bit reads. */
void App_M11_GetStopCounts(uint32_t *cutoff, uint32_t *final_edges)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *cutoff = m10_cutoff_edges;
    *final_edges = m3_drive_edges;
    if (primask == 0U) __enable_irq();
}

uint8_t App_M3_GetDriveTicks(void)
{
    return m3_drive_ticks;
}

void App_M4_AdjustTarget(int8_t change_rpm)
{
    int16_t desired;
    if (m2_status == M2_RUNNING || m2_status == M2_ARMED ||
        m2_status == M2_FAULT || m2_status == M2_OVERSPEED)
    {
        return;
    }
    desired = (int16_t)m4_target_rpm + (int16_t)change_rpm;
    if (desired >= (int16_t)M4_MIN_TARGET_RPM &&
        desired <= (int16_t)M4_MAX_TARGET_RPM)
    {
        m4_target_rpm = (uint8_t)desired;
    }
}

uint8_t App_M4_GetTargetRPM(void)
{
    return m4_target_rpm;
}

uint16_t App_M4_GetLastCompare(void)
{
    return m4_last_compare;
}

uint16_t App_M6_GetAdcRaw(void)
{
    return m6_adc_raw;
}

uint16_t App_M6_GetPeakDelta(void)
{
    return m6_peak_delta;
}

uint16_t App_M8_GetBaseline(void)
{
    return m6_adc_baseline;
}

int16_t App_M8_GetSteadyDelta(void)
{
    return m8_steady_mean_delta;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        uint32_t now = HAL_GetTick();
        uint32_t period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        __HAL_TIM_SET_COUNTER(htim, 0);
        encoder_capture_count++;
        /* Shut down at the threshold edge, rather than waiting for the next
           50 ms speed-control tick. TIM2 and TIM4 have equal NVIC priority. */
        if (m2_status == M2_RUNNING &&
            (uint32_t)(encoder_capture_count - m2_start_capture_count) >=
                M12_TARGET_EDGES)
        {
            App_M4_FinishPulse(M2_DONE);
        }
        /* D includes shaft coasting after the target has switched off PWM.
           It becomes fixed after 500 ms; do not touch the shaft meanwhile. */
        else if (m2_status == M2_DONE &&
                 (uint32_t)(now - m4_last_stop_ms) <= M11_COAST_WINDOW_MS)
        {
            m3_drive_edges = encoder_capture_count - m2_start_capture_count;
        }

        if (encoder_primed &&
            (uint32_t)(now - encoder_last_capture_ms) <= ENCODER_TIMEOUT_MS &&
            period != 0U)
        {
            encoder_period_ticks = period;
            encoder_period_valid = 1U;
        }
        else
        {
            encoder_period_valid = 0U;
        }
        encoder_primed = 1U;
        encoder_last_capture_ms = now;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2 && m2_status == M2_RUNNING)
    {
        uint32_t capture_count = encoder_capture_count;
        uint16_t next_compare = m4_last_compare;
        uint8_t fresh_edge = (capture_count != m4_last_edge_count);
        uint16_t raw;
        uint16_t delta;
        if (!App_M7_ReadMean(&raw))
        {
            App_M4_FinishPulse(M2_FAULT);
            return;
        }
        m6_adc_raw = raw;
        delta = (raw >= m6_adc_baseline)
                  ? (uint16_t)(raw - m6_adc_baseline)
                  : (uint16_t)(m6_adc_baseline - raw);
        if (delta > m6_peak_delta) m6_peak_delta = delta;
        ++m2_pulse_ticks;
        if (m2_pulse_ticks >= M8_STEADY_FIRST_TICK)
        {
            m8_steady_sum += (int32_t)raw - (int32_t)m6_adc_baseline;
            ++m8_steady_count;
            m8_steady_mean_delta =
                (int16_t)(m8_steady_sum / (int32_t)m8_steady_count);
        }
        if (m2_pulse_ticks >= M2_MAX_PULSE_TICKS)
        {
            App_M4_FinishPulse(M2_STOPPED);
            return;
        }
        if (fresh_edge)
        {
            m4_last_edge_count = capture_count;
            m4_no_edge_ticks = 0U;
        }
        else if (m4_no_edge_ticks < M4_NO_ENCODER_TICKS)
        {
            ++m4_no_edge_ticks;
        }
        if (m4_no_edge_ticks >= M4_NO_ENCODER_TICKS)
        {
            App_M4_FinishPulse(M2_NO_ENCODER);
            return;
        }

        /* One bounded 2-count adjustment per 50 ms. Retain the starting
           output until at least two fresh captures provide a speed sample. */
        if ((uint32_t)(capture_count - m2_start_capture_count) >= 2U &&
            fresh_edge &&
            encoder_period_valid && encoder_period_ticks != 0U &&
            (uint32_t)(HAL_GetTick() - encoder_last_capture_ms) <= 150U)
        {
            uint32_t measured_rpm = M4_RPM_FROM_PERIOD / encoder_period_ticks;
            if (measured_rpm >= M5_OVERSPEED_RPM)
            {
                if (++m5_overspeed_samples >= M5_OVERSPEED_SAMPLES)
                {
                    App_M4_FinishPulse(M2_OVERSPEED);
                    return;
                }
            }
            else
            {
                m5_overspeed_samples = 0U;
            }
            if (measured_rpm > (uint32_t)m4_target_rpm + 1U &&
                next_compare > M4_MIN_COMPARE)
            {
                next_compare -= 2U;
            }
            else if (measured_rpm + 1U < (uint32_t)m4_target_rpm &&
                     next_compare < M4_MAX_COMPARE)
            {
                next_compare += 2U;
            }
            if (next_compare != m4_last_compare)
            {
                if (!Int_Motor_SetTestCompare(next_compare))
                {
                    App_M4_FinishPulse(M2_FAULT);
                    return;
                }
                m4_last_compare = next_compare;
            }
        }
    }
}

#include "Int_motor.h"

static uint8_t m2_pwm_ready = 0;

void Int_Motor_EmergencyStop(void)
{
    /* Shutdown is the first action, including when KEY3 triggers EXTI. */
    HAL_GPIO_WritePin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);
}

void Int_Motor_Init(void)
{
    Int_Motor_EmergencyStop();
    REG_COILS_BUF[2] = 0;
    /* The original board drives two bridge inputs with complementary PWM. */
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
        return;
    }
    if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        return;
    }
    m2_pwm_ready = 1;
    Int_Motor_EmergencyStop();
}

uint8_t Int_Motor_TestPulse(void)
{
    if (!m2_pwm_ready)
    {
        return 0;
    }
    Int_Motor_EmergencyStop();
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, M2_TEST_COMPARE);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
    {
        Int_Motor_EmergencyStop();
        return 0;
    }
    HAL_GPIO_WritePin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin, GPIO_PIN_RESET);
    return 1;
}

uint8_t Int_Motor_SetTestCompare(uint16_t compare)
{
    /* The closed-loop experiment may only reduce the previously verified
       forward pulse, and may never bypass the existing shutdown pin. */
    if (!m2_pwm_ready || compare < M4_MIN_COMPARE || compare > M4_MAX_COMPARE)
    {
        return 0U;
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compare);
    return 1U;
}

void Int_Motor_Stop(void)
{
    Int_Motor_EmergencyStop();
    HAL_TIM_Base_Stop_IT(&htim2);
    REG_COILS_BUF[2] = 0;
}

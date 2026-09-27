#include "App_key.h"
#include "App_run.h"

volatile uint8_t key_value = 0;

void App_Key_Scan(void)
{
    uint8_t pending = key_value;
    key_value = 0U;

    if (pending == 3U)
    {
        App_M2_EmergencyStop();
    }
    else if (pending == 2U)
    {
        App_M4_AdjustTarget(5);
    }
    else if (pending == 1U)
    {
        App_M2_ArmOrStart();
    }
    else if (pending == 4U)
    {
        App_M4_AdjustTarget(-5);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_key_ms = 0U;
    uint32_t now = HAL_GetTick();

    /* KEY3 disables the bridge before the main loop and before debounce. */
    if (GPIO_Pin == KEY3_Pin &&
        HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_RESET)
    {
        App_M2_EmergencyStop();
        key_value = 3U;
        last_key_ms = now;
        return;
    }
    if ((uint32_t)(now - last_key_ms) < 30U)
    {
        return;
    }
    if (GPIO_Pin == KEY1_Pin &&
        HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
    {
        key_value = 1U;
        last_key_ms = now;
    }
    else if (GPIO_Pin == KEY2_Pin &&
             HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
    {
        key_value = 2U;
        last_key_ms = now;
    }
    else if (GPIO_Pin == KEY4_Pin &&
             HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin) == GPIO_PIN_RESET)
    {
        key_value = 4U;
        last_key_ms = now;
    }
}

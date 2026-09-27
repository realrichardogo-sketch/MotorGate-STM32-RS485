#include "Com_debug.h"

void Com_debug_init(void)
{
    // HAL库系统完成USART1的初始化 => main方法中已经执行过
    // MX_USART1_UART_Init();

    DEBUG_PRINTln("Com_debug_init");
}

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

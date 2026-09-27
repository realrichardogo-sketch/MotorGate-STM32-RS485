/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- static functions ---------------------------------*/
static void prvvTIMERExpiredISR(void);

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortTimersInit(USHORT usTim1Timerout50us)
{
    // MX_TIM3_Init();
    return TRUE;
}

void vMBPortTimersEnable()
{
    /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
    // 清空计数器的值
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    // 使能定时器
    HAL_TIM_Base_Start_IT(&htim3);
}

void vMBPortTimersDisable()
{
    /* Disable any pending timers. */
    HAL_TIM_Base_Stop_IT(&htim3);
}

/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */
static void prvvTIMERExpiredISR(void)
{
    (void)pxMBPortCBTimerExpired();
}


void TIM3_IRQHandler(void)
{
    // 判断是否为溢出中断
    if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET)
    {
        // 清除溢出中断标志位
        __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
        // 调用定时器中断处理函数
        prvvTIMERExpiredISR();
    }
    HAL_TIM_IRQHandler(&htim3);
}

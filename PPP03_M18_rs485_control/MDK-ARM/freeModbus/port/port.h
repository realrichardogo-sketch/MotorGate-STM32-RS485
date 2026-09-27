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

#ifndef _PORT_H
#define _PORT_H

#include <assert.h>
#include <inttypes.h>

#include "Com_debug.h"
#include "tim.h"
#define INLINE inline
#define PR_BEGIN_EXTERN_C \
    extern "C"            \
    {
#define PR_END_EXTERN_C }

#define ENTER_CRITICAL_SECTION()
#define EXIT_CRITICAL_SECTION()

typedef uint8_t BOOL;

typedef unsigned char UCHAR;
typedef char CHAR;

typedef uint16_t USHORT;
typedef int16_t SHORT;

typedef uint32_t ULONG;
typedef int32_t LONG;

// 定义十路线圈的大小   可读可写的二进制数据
#define REG_COILS_SIZE 10
// 声明线圈缓冲区，并初始化，用于存储十路线圈的状态
extern uint8_t REG_COILS_BUF[REG_COILS_SIZE];

// 定义十路离散量的大小   只读的二进制数据
#define REG_DISC_SIZE 10
// 声明离散量缓冲区，并初始化，用于存储十路离散量的状态
extern uint8_t REG_DISC_BUF[REG_DISC_SIZE];

// 定义十路保持寄存器的大小  可读可写的寄存器
#define REG_HOLD_SIZE 10
// 声明保持寄存器缓冲区，用于存储十路保持寄存器的值
extern uint16_t REG_HOLD_BUF[REG_HOLD_SIZE];

// 定义十路输入寄存器的大小  只读的寄存器
#define REG_INPUT_SIZE 10
// 声明输入寄存器缓冲区，用于存储十路输入寄存器的值
extern uint16_t REG_INPUT_BUF[REG_INPUT_SIZE];

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#endif

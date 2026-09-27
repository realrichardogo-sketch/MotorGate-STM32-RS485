#ifndef __COM_DEBUG__
#define __COM_DEBUG__

#include "usart.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"
/* 针对日志打印  需要手动设计一个开关 => 控制当前需不需要打印日志 */
#define DEBUG_ENABLE 1

#ifdef DEBUG_ENABLE

#define DEBUG_INIT() Com_debug_init()

// 将字符串User\main.c 切分转换为main.c
#define __FILE (strrchr(__FILE__, '\\') == NULL ? __FILE__ : strrchr(__FILE__, '\\') + 1)
#define _FILE (strrchr(__FILE, '/') == NULL ? __FILE : strrchr(__FILE, '/') + 1)

// printf() 打印信息 先输出文件名称和行号
#define DEBUG_PRINTF(format, ...) printf("[%s:%d]" format, _FILE, __LINE__, ##__VA_ARGS__)
#define DEBUG_PRINTln(format, ...) printf("[%s:%d]" format "\r\n", _FILE, __LINE__, ##__VA_ARGS__)

#else

#define DEBUG_INIT()
#define DEBUG_PRINTF(format, ...)
#define DEBUG_PRINTln(format, ...)
#endif

void Com_debug_init(void);

#endif // __COM_DEBUG__

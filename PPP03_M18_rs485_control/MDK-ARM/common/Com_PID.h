#ifndef __COM_PID__
#define __COM_PID__

#include "main.h"

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float error;
    float error_last;
    float error_sum;
    float output;
    float target;
    float measure;
} PID_Struct;

void Com_PID_Calculate(PID_Struct *pid, float target, float measure, float dt);

#endif // __COM_PID__

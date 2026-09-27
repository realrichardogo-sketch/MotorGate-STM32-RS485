#include "Com_PID.h"

void Com_PID_Calculate(PID_Struct *pid, float target, float measure, float dt)
{
    float error = target - measure;
    pid->error_sum += error * dt;
    float error_diff = (error - pid->error_last) / dt;
    pid->error_last = error;

    pid->output = pid->Kp * error + pid->Ki * pid->error_sum + pid->Kd * error_diff;
}

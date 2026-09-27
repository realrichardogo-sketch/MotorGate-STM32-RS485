#ifndef __APP_RUN__
#define __APP_RUN__

#include "Int_motor.h"

typedef enum
{
    M2_READY = 0,
    M2_ARMED,
    M2_RUNNING,
    M2_STOPPED,
    M2_DONE,
    M2_NO_ENCODER,
    M2_FAULT,
    M2_OVERSPEED
} M2_Status;

extern float speed_now;

void App_Run_Init(void);
void App_M2_Update(void);
void App_M2_ArmOrStart(void);
void App_M2_EmergencyStop(void);
M2_Status App_M2_GetStatus(void);
void App_M3_ResetPulseCount(void);
uint32_t App_M3_GetPulseCount(void);
uint32_t App_M3_GetDriveEdges(void);
uint32_t App_M10_GetCutoffEdges(void);
void App_M11_GetStopCounts(uint32_t *cutoff, uint32_t *final_edges);
uint8_t App_M3_GetDriveTicks(void);
void App_M4_AdjustTarget(int8_t change_rpm);
uint8_t App_M4_GetTargetRPM(void);
uint16_t App_M4_GetLastCompare(void);
uint16_t App_M6_GetAdcRaw(void);
uint16_t App_M6_GetPeakDelta(void);
uint16_t App_M8_GetBaseline(void);
int16_t App_M8_GetSteadyDelta(void);

#endif

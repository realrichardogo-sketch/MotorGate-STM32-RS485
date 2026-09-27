#ifndef __INT_MOTOR__
#define __INT_MOTOR__

#include "tim.h"
#include "port.h"

/* TIM1 ARR = 999; neutral compare = 500, fixed test = 560. */
#define M2_TEST_COMPARE 560U
#define M4_MIN_COMPARE 540U
#define M4_MAX_COMPARE M2_TEST_COMPARE
#if M2_TEST_COMPARE <= 500U || M2_TEST_COMPARE > 560U
#error "M2 test compare must stay within (500, 560]"
#endif
#if M4_MIN_COMPARE <= 500U || M4_MIN_COMPARE > M4_MAX_COMPARE
#error "M4 compare range must stay above neutral and below the M2 limit"
#endif

void Int_Motor_Init(void);
uint8_t Int_Motor_TestPulse(void);
uint8_t Int_Motor_SetTestCompare(uint16_t compare);
void Int_Motor_EmergencyStop(void);
void Int_Motor_Stop(void);

#endif

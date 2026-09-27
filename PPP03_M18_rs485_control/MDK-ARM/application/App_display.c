#include "App_display.h"
#include "App_run.h"

void App_display_init(void)
{
    Inf_OLED_Init();
}

void App_display_show(void)
{
    uint8_t i;
    uint32_t cutoff;
    uint32_t final_edges;
    const char *state = "M18:READY  ";

    App_M11_GetStopCounts(&cutoff, &final_edges);

    switch (App_M2_GetStatus())
    {
    case M2_ARMED:      state = "M18:ARMED  "; break;
    case M2_RUNNING:    state = "M18:RUN    "; break;
    case M2_STOPPED:    state = "M18:STOP   "; break;
    case M2_DONE:       state = "M18:DONE   "; break;
    case M2_NO_ENCODER: state = "M18:NO ENC "; break;
    case M2_FAULT:      state = "M18:FAULT  "; break;
    case M2_OVERSPEED:  state = "M18:OVRSPD "; break;
    default: break;
    }

    for (i = 0U; i < 7U; i++)
    {
        Inf_OLED_ShowChinese(8U + 16U * i, 4U, i, 16U, 1U);
    }
    Inf_OLED_ShowString(0U, 24U, (uint8_t *)state, 12U, 1U);
    /* Longest status uses 66 px including its trailing space. */
    Inf_OLED_ShowString(66U, 24U, (uint8_t *)"T", 12U, 1U);
    Inf_OLED_ShowNum(72U, 24U, App_M3_GetDriveTicks(), 2U, 12U, 1U);
    Inf_OLED_ShowString(90U, 24U, (uint8_t *)"D", 12U, 1U);
    Inf_OLED_ShowNum(96U, 24U, final_edges > 999U ? 999U : final_edges, 3U, 12U, 1U);
    Inf_OLED_ShowString(0U, 38U, (uint8_t *)"S:", 12U, 1U);
    Inf_OLED_ShowNum(12U, 38U, App_M4_GetTargetRPM(), 2U, 12U, 1U);
    Inf_OLED_ShowString(30U, 38U, (uint8_t *)" R:", 12U, 1U);
    Inf_OLED_ShowNum(48U, 38U, (uint32_t)speed_now, 3U, 12U, 1U);
    Inf_OLED_ShowString(72U, 38U, (uint8_t *)" A:", 12U, 1U);
    Inf_OLED_ShowNum(90U, 38U, App_M6_GetAdcRaw(), 4U, 12U, 1U);
    /* F is the shutdown count, K is D minus F (coast).
       Existing ADC baseline/peak/mean data remain available in firmware. */
    Inf_OLED_ShowString(0U, 52U, (uint8_t *)"C", 12U, 1U);
    Inf_OLED_ShowNum(6U, 52U, App_M4_GetLastCompare(), 3U, 12U, 1U);
    Inf_OLED_ShowString(30U, 52U, (uint8_t *)"F", 12U, 1U);
    {
        uint32_t coast = final_edges >= cutoff ? final_edges - cutoff : 0U;
        Inf_OLED_ShowNum(36U, 52U, cutoff > 999U ? 999U : cutoff, 3U, 12U, 1U);
        Inf_OLED_ShowString(66U, 52U, (uint8_t *)"K", 12U, 1U);
        Inf_OLED_ShowNum(72U, 52U, coast > 999U ? 999U : coast, 3U, 12U, 1U);
    }
    Inf_OLED_Refresh();
}

#ifndef __DEV_PWMIN_H__
#define __DEV_PWNIN_H__

#include <stdio.h>
#include "esp_system.h"
#include "app_main.h"

// 宏定义
//=============================================================================

#if (USE_PWM_PPM == RC_WMM_IN)

typedef struct 
{
    int16_t RC_ch[10];
    uint32_t recv_time;
    uint8_t ppm_lost;
}Rc_t;
extern Rc_t Rc;
//=============================================================================

// 函数声明
//=============================================================================
void RcIn_Init(void);
#endif

#endif

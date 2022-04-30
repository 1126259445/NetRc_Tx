#ifndef __NVS_H__
#define __NVS_H__
#include "esp_system.h"

typedef struct 
{
    uint8_t ssid[32];
    uint8_t password[64];
    uint8_t subid[32];
    uint16_t safe_ch_val[8];
}User_HttpSeverInfo_t;
extern User_HttpSeverInfo_t User_HttpSeverInfo;

void NetRc_Read_info(User_HttpSeverInfo_t *User_HttpSeverInfo);

void NetRc_save_info(User_HttpSeverInfo_t *User_HttpSeverInfo);

#endif

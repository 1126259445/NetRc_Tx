#include "User_NvsData.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"


static const char *TAG="USER_NvsData";

User_HttpSeverInfo_t User_HttpSeverInfo;


/**
 * @description: 
 * @param {type} 
 * @return: 
 */
void NetRc_Read_info(User_HttpSeverInfo_t *User_HttpSeverInfo)
{
    nvs_handle out_handle;
    //从本地存储读取USER info
    if (nvs_open("DeviceInfo", NVS_READONLY, &out_handle) == ESP_OK)
    {    
        size_t len = sizeof(User_HttpSeverInfo->ssid);
        if (nvs_get_str(out_handle, "Wifi_Ssid", (char *)User_HttpSeverInfo->ssid,&len) == ESP_OK)
        {
            ESP_LOGI(TAG, "wifi_info is have Wifi ssid %s ",User_HttpSeverInfo->ssid);
        }
        len = sizeof(User_HttpSeverInfo->password);
        if (nvs_get_str(out_handle, "Wifi_Password", (char *)User_HttpSeverInfo->password, &len) == ESP_OK)
        {
            ESP_LOGI(TAG, "wifi_info is have Wifi password %s ",User_HttpSeverInfo->password);
        }
        len = sizeof(User_HttpSeverInfo->subid);
         if (nvs_get_str(out_handle, "Subid", (char *)User_HttpSeverInfo->subid, &len) == ESP_OK)
        {
             ESP_LOGI(TAG, "wifi_info is have Subid  %s ",User_HttpSeverInfo->subid);
        }  

    
        nvs_get_u16(out_handle, "Safe_Ch_Val1", &User_HttpSeverInfo->safe_ch_val[0]);
        nvs_get_u16(out_handle, "Safe_Ch_Val2", &User_HttpSeverInfo->safe_ch_val[1]);
        nvs_get_u16(out_handle, "Safe_Ch_Val3", &User_HttpSeverInfo->safe_ch_val[2]);
        nvs_get_u16(out_handle, "Safe_Ch_Val4", &User_HttpSeverInfo->safe_ch_val[3]);
        nvs_get_u16(out_handle, "Safe_Ch_Val5", &User_HttpSeverInfo->safe_ch_val[4]);
        ESP_LOGI(TAG, "wifi_info is have Safe_Ch_Val  %d %d %d %d %d ",User_HttpSeverInfo->safe_ch_val[0],User_HttpSeverInfo->safe_ch_val[1],\
        User_HttpSeverInfo->safe_ch_val[2],User_HttpSeverInfo->safe_ch_val[3],User_HttpSeverInfo->safe_ch_val[4]);


        nvs_close(out_handle);
    }
}
/**
 * @description: 
 * @param {type} 
 * @return: 
 */
void NetRc_save_info(User_HttpSeverInfo_t *User_HttpSeverInfo)
{
    nvs_handle out_handle_Device;
 
    if (nvs_open("DeviceInfo", NVS_READWRITE, &out_handle_Device) != ESP_OK)
    {
        return;
    }

    if (nvs_set_str(out_handle_Device, "Wifi_Ssid", (char*)User_HttpSeverInfo->ssid) != ESP_OK)
    {
        printf("--Save Wifi_Ssid fail");
    }

    if (nvs_set_str(out_handle_Device, "Wifi_Password", (char*)User_HttpSeverInfo->password) != ESP_OK)
    {
        printf("--Save Wifi_Password fail");
    }

    if (nvs_set_str(out_handle_Device, "Subid", (char*)User_HttpSeverInfo->subid) != ESP_OK)
    {
        printf("--Save Device_Mac fail");
    }

    if (nvs_set_u16(out_handle_Device, "Safe_Ch_Val1", User_HttpSeverInfo->safe_ch_val[0]) != ESP_OK)
    {
        printf("--Save Safe_Ch_Val fail");
    }
    if (nvs_set_u16(out_handle_Device, "Safe_Ch_Val2", User_HttpSeverInfo->safe_ch_val[1]) != ESP_OK)
    {
        printf("--Save Safe_Ch_Val fail");
    }
    if (nvs_set_u16(out_handle_Device, "Safe_Ch_Val3", User_HttpSeverInfo->safe_ch_val[2]) != ESP_OK)
    {
        printf("--Save Safe_Ch_Val fail");
    }
    if (nvs_set_u16(out_handle_Device, "Safe_Ch_Val4", User_HttpSeverInfo->safe_ch_val[3]) != ESP_OK)
    {
        printf("--Save Safe_Ch_Val fail");
    }
    if (nvs_set_u16(out_handle_Device, "Safe_Ch_Val5", User_HttpSeverInfo->safe_ch_val[4]) != ESP_OK)
    {
        printf("--Save Safe_Ch_Val fail");
    }

    nvs_close(out_handle_Device);
}



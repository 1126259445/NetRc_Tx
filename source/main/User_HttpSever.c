/* Simple HTTP Server Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "router.h"

#include <esp_http_server.h>

static const char *TAG="USER_HTTP_Sever";

typedef struct 
{
    uint8_t ssid[32];
    uint8_t password[64];
    uint8_t devid[32];
}User_HttpSeverInfo_t;
User_HttpSeverInfo_t User_HttpSeverInfo;


static wifi_config_t Wifi_info;



/**
 * @description: 
 * @param {type} 
 * @return: 
 */
int NetRc_Read_info(uint8_t *data)
{
    nvs_handle out_handle;
        //从本地存储读取是否存在ssid和password
    if (nvs_open("DeviceInfo", NVS_READONLY, &out_handle) == ESP_OK)
    {    
        uint8_t size = 32;
        if (nvs_get_str(out_handle, "Device_Mac", (char *)data, &size) == ESP_OK)
        {
            ESP_LOGI(TAG, "Device_Mac = %s ",data);
            return 1;
        }
    }

    return 0;
}
/**
 * @description: 
 * @param {type} 
 * @return: 
 */
void NetRc_save_info(char *Device_Mac)
{
    nvs_handle out_handle_Device;
    char data[32];
    if (nvs_open("DeviceInfo", NVS_READWRITE, &out_handle_Device) != ESP_OK)
    {
        return;
    }

    memset(data, 0x0, sizeof(data));
    strncpy(data, Device_Mac, strlen(Device_Mac));
    if (nvs_set_str(out_handle_Device, "Device_Mac", data) != ESP_OK)
    {
        printf("--set Device_Mac fail");
    }

    nvs_close(out_handle_Device);
}


/* An HTTP GET handler */
esp_err_t home_get_handler(httpd_req_t *req)
{
    char*  buf;
    size_t buf_len;

    /* Read URL query string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) 
    {
        buf = malloc(buf_len);
         if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) 
         {
            ESP_LOGI(TAG, "hello Found URL query => %s", buf);
            char ssid[32] = {0};
            char password[32] = {0};
            char subid[32] = {0};
            /* Get value of expected key from query string */
            if ((httpd_query_key_value(buf, "ssid", ssid, sizeof(ssid)) == ESP_OK) && (httpd_query_key_value(buf, "password", password, sizeof(password)) == ESP_OK))
             {
                if(strlen(ssid) != 0 && strlen(password) != 0)
                {
                    router_wifi_save_info((uint8_t*)ssid,(uint8_t*)password);
                    ESP_LOGI(TAG, "HOME Found URL query parameter => ssid=%s", ssid);
                    ESP_LOGI(TAG, "HOME Found URL query parameter => password=%s", password);
                    free(buf);
                    return ESP_OK;
                }
                else
                {
                     ESP_LOGI(TAG, "Wifi Info is NULL");
                }
            }
        }
        free(buf);
    }

    /* Send response with custom headers and body set as the
     * string passed in user context*/
    const char* resp_str = (const char*) req->user_ctx;
    httpd_resp_send(req, resp_str, strlen(resp_str));

    /* After sending the HTTP response the old HTTP request
     * headers are lost. Check if HTTP request headers can be read now. */
    if (httpd_req_get_hdr_value_len(req, "Host") == 0) {
        ESP_LOGI(TAG, "Request headers lost");
    }
    return ESP_OK;
}

esp_err_t sys_reset_get_handler(httpd_req_t *req)
{
     ESP_LOGI(TAG, "sys_reset_get_handler");
     esp_restart();
     return ESP_OK;
}


char homeindex[1000] = {0};
httpd_uri_t home = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = home_get_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx  = homeindex
};

httpd_uri_t sys_reset ={
    .uri       = "/sys_reset",
    .method    = HTTP_GET,
    .handler   = sys_reset_get_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
//    .user_ctx  = home_html
};

httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &home);
        httpd_register_uri_handler(server, &sys_reset);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

void stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_stop(server);
}

static httpd_handle_t server = NULL;

static void disconnect_handler(void* arg, esp_event_base_t event_base, 
                               int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        stop_webserver(*server);
        *server = NULL;
    }
}

extern char deviceUUID[17];
void HttpSever_Init()
{
    sprintf(homeindex,"<!DOCTYPE html><html lang='en'><head><meta charset='UTF8'><title>NetRc设置 \
    </title><script type='text/javascript'>function WifiClicked(){ssid=document.getElementById('ssid').value; \
    password=document.getElementById('password').value;alert('SSID: '+ssid+'   PASSWORD: '+password)} \
    </script></head><body bgcolor=lightblue><style>#myHeader{padding-top:250px;color:black;text-align:center} \
    </style><div id=myHeader><h1>NetRc配网</h1><form name='my'>WiFi名称：<input type='text'name='ssid'id='ssid'placeholder='请输入您WiFi的名称'> \
    <br>WiFi密码：<input type='text'name='password'id='password'placeholder='请输入您WiFi的密码'><input type='submit'value='连接'onclick='WifiClicked()'> \
    <br><br>设备MAC：%s<br></form><br>系统设置：<br><form action='sys_reset'><input type='submit'value='重启'></form></body></html>",deviceUUID);

    server = start_webserver();
}

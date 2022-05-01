#ifndef __APP_MIAN_H__
#define __APP_MIAN_H__
#include "freertos/semphr.h"

#include "mqtt_client.h"

#define RC_PWM_IN       (0)
#define RC_PPM_IN       (1)     
#define USE_PWM_PPM     RC_PWM_IN   //0 PWMIN    1 PPM

extern char deviceUUID[17];
extern char MqttTopicSub[30];
extern char MqttTopicPub[30];

extern bool isConnect2Server;
extern bool isWifiConnectd;
extern esp_mqtt_client_handle_t client;
extern SemaphoreHandle_t xSemaphore;
#endif


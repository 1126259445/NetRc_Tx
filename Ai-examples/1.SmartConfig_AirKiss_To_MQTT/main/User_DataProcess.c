
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "cJSON.h"
#include <stdio.h>
#include "string.h"
#include "xpwm.h"

#include "app_main.h"
#include "User_DataProcess.h"
#include "Dev_Dht11.h"
#include "Dev_Pwm.h"
#include "Dev_Ppm.h"
#include "Dev_PwmIn.h"
#include "Dev_Led.h"

/*------------------------------JSON data---------------------------------------*/
/*init mqtt_client publish_data for mqtt*/
static const char *TAG = "User_DataProcess";

#ifndef DEVECE_ID
#define DEVECE_ID "DEV00003"
#endif


#define MSG_POWER_ON_ID 5001
#define MSG_HRAET_ID 5002
#define MSG_DATA_UP_ID 5003
#define MSG_DATA_DOWN_ID 5004

#define MSG_POWER_ON_ACK_ID 6001
#define MSG_HRAET_ACK_ID 6002
#define MSG_DATA_UP_ACK_ID 6003
#define MSG_DATA_DOWN_ACK_ID 6004


User_data user_data;

typedef struct 
{
    uint32_t  Switch ;
	int32_t Variable_Val[8] ;
}mqtt_cmd_struct;
mqtt_cmd_struct mqtt_cmd;;

/**
 * @description: mqtt_publish_data_interface
 * @param : publish_topic   
 *          pub_payload(json_data)  
 *          qos      0 1 or 2, see MQTT specification 
 *          retain   No don't retain such crappy payload.
 * @return: 
 */
extern esp_mqtt_client_handle_t client;
void mqtt_publish_data_interface(char *publish_topic, const char *pub_payload,uint8_t qos,uint8_t retain )
{
	esp_mqtt_client_publish(client, publish_topic, pub_payload, strlen(pub_payload), qos, retain);
}


/**************************************
 * @description: user public up_data, json_load_send_data
 * @param {type} 
 * @return: 
 */
#if 0
static int Json_Get_Battery()
{
	return 88;
}
static int Json_Get_Longitude()
{
	return 1131234567;
}
static int Json_Get_Latitude()
{
	return 221234567;
}
static int Json_Get_Altitude()
{
	return 1100;
}
static int Json_Get_Env_Temperature()
{
	return Get_Dht11_Temperature();
}
static int Json_Get_Env_Humidity()
{
	return Get_Dht11_Humidity();
}
static int Json_Get_Env_Pressure()
{
	return 1010+(rand()%30);
}
static int Json_Get_Wind_Speed()
{
	return 10+(rand()%30);
}
static int Json_Get_Wind_Direction()
{
	return 100+(rand()%30);
}
#endif
static uint32_t Json_Get_Switch()
{
	return 0;
}

static int Json_Get_Variable_Val_0()
{
	return Rc.RC_ch[0];;
}
static int Json_Get_Variable_Val_1()
{
	return Rc.RC_ch[1];
}
static int Json_Get_Variable_Val_2()
{
	return Rc.RC_ch[2];
}
static int Json_Get_Variable_Val_3()
{
	return Rc.RC_ch[3];
}
static int Json_Get_Variable_Val_4()
{
	return Rc.RC_ch[4];
}
static int Json_Get_Variable_Val_5()
{
	return Rc.RC_ch[5];
}
static int Json_Get_Variable_Val_6()
{
	return Rc.RC_ch[6];
}
static int Json_Get_Variable_Val_7()
{
	return Rc.RC_ch[7];
}


static void Json_Recv_Cmd_Process(mqtt_cmd_struct* cmd)
{
	/*code  cmd->Variable_Val_0 cmd->Variable_Val_1 cmd->Variable_Val_2*/
	Set_Pwm_All_Chinel_Val(8,(uint32_t*)(cmd->Variable_Val));
}
/*****************END*******************/



/**
 * @description: joson_create_uav_data_send
 * @param {type} 
 * @return: 
 */
void joson_create_uav_data_send()
{
    /* declare a few. */
	cJSON *root = NULL;
	cJSON *head = NULL;
	cJSON *data = NULL;
		
    /* Here we construct some JSON standards, from the JSON site. */
	static uint64_t msg_num = 0;
	msg_num++;	

		/*模拟JSON数据*/
	root = cJSON_CreateObject();
	cJSON_AddItemToObject(root,"head",head = cJSON_CreateObject());
		cJSON_AddNumberToObject(head, "msg_id", MSG_DATA_DOWN_ID);
		cJSON_AddNumberToObject(head, "msg_no", msg_num);
	cJSON_AddItemToObject(root,"data",data = cJSON_CreateObject());
/*		cJSON_AddNumberToObject(data, "Battery", Json_Get_Battery());
		cJSON_AddNumberToObject(data, "Longitude", Json_Get_Longitude());
		cJSON_AddNumberToObject(data, "Latitude", Json_Get_Latitude());
		cJSON_AddNumberToObject(data, "Altitude", Json_Get_Altitude());
		cJSON_AddNumberToObject(data, "Env_Temperature", Json_Get_Env_Temperature());
		cJSON_AddNumberToObject(data, "Env_Humidity", Json_Get_Env_Humidity());
		cJSON_AddNumberToObject(data, "Env_Pressure", Json_Get_Env_Pressure());
		cJSON_AddNumberToObject(data, "Wind_Speed", Json_Get_Wind_Speed());
		cJSON_AddNumberToObject(data, "Wind_Direction", Json_Get_Wind_Direction());
*/
		cJSON_AddNumberToObject(data, "ch_0", Json_Get_Variable_Val_0());
		cJSON_AddNumberToObject(data, "ch_1", Json_Get_Variable_Val_1());
		cJSON_AddNumberToObject(data, "ch_2", Json_Get_Variable_Val_2());
		cJSON_AddNumberToObject(data, "ch_3", Json_Get_Variable_Val_3());
		cJSON_AddNumberToObject(data, "ch_4", Json_Get_Variable_Val_4());
		cJSON_AddNumberToObject(data, "ch_5", Json_Get_Variable_Val_5());
		cJSON_AddNumberToObject(data, "ch_6", Json_Get_Variable_Val_6());
		cJSON_AddNumberToObject(data, "ch_7", Json_Get_Variable_Val_7());
		
		/*Cjson 2 char*/
		const char *pub_payload = NULL;
		pub_payload = cJSON_PrintUnformatted(root);

        /*publish JSON data to server*/
        mqtt_publish_data_interface(MqttTopicPub, pub_payload,0,0);

		if(pub_payload!=NULL)
		{
			free(pub_payload);
		}

    cJSON_Delete(root);
}



// /**
//  * @description: json_parse
//  * @param {type} 
//  * @return: 
//  */
// static uint8_t json_parse(User_data *pMqttMsg)
// {
//     cJSON *root, *head_item, *data_item;
    
//     /* Head*/
//     uint32_t msg_id = 0;
//     uint32_t msg_no = 0;
//     double timestamp = 0;
    

// 	////首先整体判断是否为一个json格式的数据
// 	root = cJSON_Parse(pMqttMsg->allData);
// 	//如果是否json格式数据
// 	if (root == NULL)
// 	{
// 		printf("[SY] Task_ParseJSON_Message xQueueReceive not json ... \n");
// 		cJSON_Delete(root);
// 		return 0;
// 	}

// 	printf( "\nbuf_LEN: %d\n", pMqttMsg->dataLen);
    
//     if (root)
//     {
// 		head_item = cJSON_GetObjectItem(root, "head");
        
//         if (head_item)
//         {
//             /* 获取消息ID确认消息类型 */
//             msg_id = cJSON_GetObjectItem(head_item, "msg_id")->valueint;
//             msg_no = cJSON_GetObjectItem(head_item, "msg_no")->valueint;
//             timestamp = cJSON_GetObjectItem(head_item, "timestamp")->valuedouble;
//             printf( "msg_id: %d\n", msg_id);
//             printf( "msg_no: %d\n", msg_no);
			
//             switch (msg_id)
//             {
//             case MSG_POWER_ON_ID:
//                 /* 开机应答 */
//                 data_item = cJSON_GetObjectItem(root, "data");                
//                 if (data_item)
//                 {
				
// 				}
//                 break;
            
//             case MSG_HRAET_ID:
//                 /* 心跳应答 */
//                 data_item = cJSON_GetObjectItem(root, "data");
                
//                 if (data_item)
//                 {

//                 }           
//                 break;
				
// 			case MSG_DATA_DOWN_ID:
// 				/*下发的控制命令*/
// 				 data_item = cJSON_GetObjectItem(root, "data");
// 				if (data_item)
//                 {
//                     mqtt_cmd.Switch = cJSON_GetObjectItem(data_item, "Switch")->valueint;
//                     mqtt_cmd.Variable_Val[0] = cJSON_GetObjectItem(data_item, "Variable_Val_0")->valueint;
// 					mqtt_cmd.Variable_Val[1] = cJSON_GetObjectItem(data_item, "Variable_Val_1")->valueint;
// 					mqtt_cmd.Variable_Val[2] = cJSON_GetObjectItem(data_item, "Variable_Val_2")->valueint;
// 				//	mqtt_cmd.Variable_Val[3] = cJSON_GetObjectItem(data_item, "Variable_Val_3")->valueint;
// 				//	mqtt_cmd.Variable_Val[4] = cJSON_GetObjectItem(data_item, "Variable_Val_4")->valueint;
// 				//	mqtt_cmd.Variable_Val[5] = cJSON_GetObjectItem(data_item, "Variable_Val_5")->valueint;
// 				//	mqtt_cmd.Variable_Val[6] = cJSON_GetObjectItem(data_item, "Variable_Val_6")->valueint;
// 				//	mqtt_cmd.Variable_Val[7] = cJSON_GetObjectItem(data_item, "Variable_Val_7")->valueint;

// 				mqtt_cmd.Variable_Val[0] = 1000+mqtt_cmd.Variable_Val[0]*4;
// 						mqtt_cmd.Variable_Val[1] = 1000+mqtt_cmd.Variable_Val[1]*4;
// 								mqtt_cmd.Variable_Val[2] = 1000+mqtt_cmd.Variable_Val[2]*4;
/* 					printf("Switch: %d\n 0: %d\n 1: %d\n 2: %d\n 3: %d\n 4: %d\n 5: %d\n 6: %d\n 7: %d\n", \
 					mqtt_cmd.Switch,mqtt_cmd.Variable_Val[0],mqtt_cmd.Variable_Val[1],mqtt_cmd.Variable_Val[2], \
 					mqtt_cmd.Variable_Val[3],mqtt_cmd.Variable_Val[4],mqtt_cmd.Variable_Val[5], \
 					mqtt_cmd.Variable_Val[6],mqtt_cmd.Variable_Val[7]);
*/
// 					Json_Recv_Cmd_Process(&mqtt_cmd);
// 				}
// 				break;
                   
//             }
//         }
//     }
    
//     cJSON_Delete(root);
    
//     return 1;
// }

// /* 
//  * @Description: 解析下发数据的队列逻辑处理
//  * @param: null
//  * @return: 
// */
// extern bool isRecvFlinis; //解析json数据的队列
// void Task_ParseJSON(void *pvParameters)
// {
// 	while (1)
// 	{
// 		if(isRecvFlinis == true)
// 		{
// 			isRecvFlinis = false;
// 			json_parse(&user_data);
// 		}
// 		vTaskDelay(100/portTICK_RATE_MS);
// 	}
// }

/**
 * @description: Task_CreatJSON 
 * @param {type} 
 * @return: 
 */
void Task_CreatJSON(void *pvParameters)
{
	static int16_t last_rc[10] = {0};
	uint8_t ifRcDataUpdate = 0;
	uint8_t OldDataRecvCount = 0;
	while(1)
	{
		if (isWifiConnectd && isConnect2Server)
		{
			if(Rc.ppm_lost == 0)
			{
				ifRcDataUpdate = 0;
				for(uint8_t i =0 ; i < 10;i++)
				{
					if(last_rc[i] != Rc.RC_ch[i])
					{
						ifRcDataUpdate  = 1;
					}
					last_rc[i] = Rc.RC_ch[i];
				}

				if(ifRcDataUpdate)
				{
					joson_create_uav_data_send();
					OldDataRecvCount = 0;
				}
				else 
				{
					if(++OldDataRecvCount > 5)
					{
						joson_create_uav_data_send();
						OldDataRecvCount = 0;
					}
				}
				Led_SetState(OFF);
			}
			else
			{
				Led_SetState(ONE_HZ);
			}		
		}
		else
		{
			if(Led_GetState() != FIVE_HZ)
			{
				Led_SetState(ON);
			}
		}
		
		vTaskDelay(100/portTICK_RATE_MS);
	}
}

// /**
//  * @description: Task_CreatJSON 
//  * @param {type} 
//  * @return: 
//  */
// extern bool isConnect2Server;
// void Task_CreatJSON(void *pvParameters)
// {
// 	while(1)
// 	{
// 		//post_data_to_clouds();
// 		if (isConnect2Server)
// 		{
// 			joson_create_uav_data_send();
// 		}

// 		vTaskDelay(1000/portTICK_RATE_MS);
// 	}
// }

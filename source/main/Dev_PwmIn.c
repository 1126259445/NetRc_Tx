/* pwm example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "esp8266/gpio_register.h"
#include "esp8266/pin_mux_register.h"

#include "driver/gpio.h"
#include "driver/hw_timer.h"
#include "Dev_PwmIn.h"

#if (USE_PWM_PPM == RC_PWM_IN)

static const char *TAG = "Dev_PwmIn";

#define PWM_START_LEAVE 0
#define PWM_END_LEAVE   1

#define SW_TIME         0   
#define HW_TIME         1
#define USE_TIME        SW_TIME
#define TIM_MAX_VAL     0xFFFFFFFF
#define PWMIN_STEP      50
#define PWM_MIN_VAL     1000
#define PWM_MAX_VAL     2000

#define USER_CH_NUM     5

//#define PPM_IN_PIN  GPIO_Pin_5
#define GPIO_INPUT_IO_0     13
#define GPIO_INPUT_IO_1     12
#define GPIO_INPUT_IO_2     14
#define GPIO_INPUT_IO_3     4
#define GPIO_INPUT_IO_4     5

#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1) | (1ULL<<GPIO_INPUT_IO_2) | (1ULL<<GPIO_INPUT_IO_3) | (1ULL<<GPIO_INPUT_IO_4))

#define LIMIT(val,low,high) (val>high?high:val<low?low:val)

Rc_t Rc = {{1500,1500,1500,1500,PWM_MIN_VAL,PWM_MIN_VAL,PWM_MIN_VAL,PWM_MIN_VAL,PWM_MIN_VAL,PWM_MIN_VAL}};

static uint32_t hw_time_count = 0;
/******************************************************************************
 * FunctionName : gpio_intr_handler
 * Description  : gpio interrupt callback funtion
 * Parameters   : void
 * Returns      : void
*******************************************************************************/
extern uint32_t esp_get_time(void);



static uint32_t get_time_us()
{
    uint32_t current_val = 0;
#if(USE_TIME == SW_TIME)
    current_val = esp_get_time();
#else if(USE_TIME == HW_TIME)
    current_val  = (hw_time_count * 1000000 ) + ((hw_timer_get_load_data() - hw_timer_get_count_data()) / 5);
#endif
    return current_val;
}


#define FIFTER_NUM  5
static uint16_t ch_data_fifter(uint8_t ch ,uint16_t new_data)
{
    uint32_t sum = 0,min = 2000,max = 1000;
    static uint8_t index[USER_CH_NUM] = {0};
    static uint16_t chbuf[USER_CH_NUM][FIFTER_NUM];
    chbuf[ch][index[ch]] = new_data;
    if(++index[ch]>=FIFTER_NUM) index[ch] = 0;

    for(uint8_t i = 0; i < FIFTER_NUM;i++)
    {
        sum += chbuf[ch][i];
        if(chbuf[ch][i] < min) min = chbuf[ch][i];
        else if(chbuf[ch][i] > max) max = chbuf[ch][i];
    }
    return (sum - min - max)/(FIFTER_NUM - 2);
}


static void gpio_0_isr_handler(void *arg)
{
    static uint32_t start_time = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
    static uint32_t end_time = 0;
    int16_t current_ch_val = 0;


    if(gpio_get_level(GPIO_INPUT_IO_0) == PWM_START_LEAVE)
    {
        start_time = get_time_us();
    }else if(gpio_get_level(GPIO_INPUT_IO_0) == PWM_END_LEAVE)
    {
        end_time = get_time_us();

        if(end_time > start_time)
        {
            current_ch_val = end_time - start_time;
        }
        else if(end_time < start_time)
        {
            current_ch_val = end_time + (TIM_MAX_VAL - start_time);
        }
        current_ch_val = ch_data_fifter(0,current_ch_val);
        current_ch_val = LIMIT(current_ch_val,PWM_MIN_VAL,PWM_MAX_VAL);
        Rc.RC_ch[0] = (current_ch_val/PWMIN_STEP)*PWMIN_STEP;
        Rc.recv_time[0] = get_time_us();
    }
}

static void gpio_1_isr_handler(void *arg)
{
    static uint32_t start_time = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
    static uint32_t end_time = 0;
    int16_t current_ch_val = 0;

    if(gpio_get_level(GPIO_INPUT_IO_1) == PWM_START_LEAVE)
    {
        start_time = get_time_us();
    }else if(gpio_get_level(GPIO_INPUT_IO_1) == PWM_END_LEAVE)
    {
        end_time = get_time_us();
        if(end_time > start_time)
        {
            current_ch_val = end_time - start_time;
        }
        else if(end_time < start_time)
        {
            current_ch_val = end_time + (TIM_MAX_VAL - start_time);
        }
        current_ch_val = ch_data_fifter(1,current_ch_val);
        current_ch_val = LIMIT(current_ch_val,PWM_MIN_VAL,PWM_MAX_VAL);
        Rc.RC_ch[1] = (current_ch_val/PWMIN_STEP)*PWMIN_STEP;
        Rc.recv_time[1] = get_time_us();
    }
}

static void gpio_2_isr_handler(void *arg)
{
    static uint32_t start_time = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
    static uint32_t end_time = 0;
    int16_t current_ch_val = 0;

    if(gpio_get_level(GPIO_INPUT_IO_2) == PWM_START_LEAVE)
    {
        start_time = get_time_us();
    }else if(gpio_get_level(GPIO_INPUT_IO_2) == PWM_END_LEAVE)
    {
        end_time = get_time_us();
        if(end_time > start_time)
        {
            current_ch_val = end_time - start_time;
        }
        else if(end_time < start_time)
        {
            current_ch_val = end_time + (TIM_MAX_VAL - start_time);
        }
        current_ch_val = ch_data_fifter(2,current_ch_val);
        current_ch_val = LIMIT(current_ch_val,PWM_MIN_VAL,PWM_MAX_VAL);
        Rc.RC_ch[2] = (current_ch_val/PWMIN_STEP)*PWMIN_STEP;
        Rc.recv_time[2] = get_time_us();
    }
}

static void gpio_3_isr_handler(void *arg)
{
    static uint32_t start_time = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
    static uint32_t end_time = 0;
    int16_t current_ch_val = 0;

    if(gpio_get_level(GPIO_INPUT_IO_3) == PWM_START_LEAVE)
    {
        start_time = get_time_us();
    }else if(gpio_get_level(GPIO_INPUT_IO_3) == PWM_END_LEAVE)
    {
        end_time = get_time_us();
        if(end_time > start_time)
        {
            current_ch_val = end_time - start_time;
        }
        else if(end_time < start_time)
        {
            current_ch_val = end_time + (TIM_MAX_VAL - start_time);
        }
        current_ch_val = ch_data_fifter(3,current_ch_val);
        current_ch_val = LIMIT(current_ch_val,PWM_MIN_VAL,PWM_MAX_VAL);
        Rc.RC_ch[3] = (current_ch_val/PWMIN_STEP)*PWMIN_STEP;
        Rc.recv_time[3] = get_time_us();
    }
}

static void gpio_4_isr_handler(void *arg)
{
    static uint32_t start_time = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
    static uint32_t end_time = 0;
    int16_t current_ch_val = 0;

    if(gpio_get_level(GPIO_INPUT_IO_4) == PWM_START_LEAVE)
    {
        start_time = get_time_us();
    }else if(gpio_get_level(GPIO_INPUT_IO_4) == PWM_END_LEAVE)
    {
        end_time = get_time_us();
        if(end_time > start_time)
        {
            current_ch_val = end_time - start_time;
        }
        else if(end_time < start_time)
        {
            current_ch_val = end_time + (TIM_MAX_VAL - start_time);
        }
        current_ch_val = ch_data_fifter(4,current_ch_val);
        current_ch_val = LIMIT(current_ch_val,PWM_MIN_VAL,PWM_MAX_VAL);
        Rc.RC_ch[4] = (current_ch_val/PWMIN_STEP)*PWMIN_STEP;
        Rc.recv_time[4] = get_time_us();
    }
}



static void Task_Show_PwmIn(void *pvParameters) 
{
    uint32_t recv_lost_time = 0;
    uint8_t ch_num = 0;
    static uint8_t lost_state = 0xff;
    while(1)
    {
        lost_state = 0xff;

        for(ch_num = 0;ch_num < USER_CH_NUM ;ch_num++)
        {
            if(get_time_us() > Rc.recv_time[ch_num])
            {
                recv_lost_time = get_time_us() - Rc.recv_time[ch_num];
            }
            else if(get_time_us() < Rc.recv_time[ch_num])
            {
                recv_lost_time = get_time_us() + (TIM_MAX_VAL - Rc.recv_time[ch_num]);
            }

            if(recv_lost_time > 30000)
            {
                Rc.RC_ch[ch_num] = 1500;
                lost_state = lost_state | (1 << ch_num);
            }
            else
            {
                lost_state = lost_state & (~(1 << ch_num));
            }
        }

        if(lost_state == 0xff)
        {
            Rc.ppm_lost = 1;
            memset(Rc.RC_ch,1500,10);
            ESP_LOGI(TAG, "RC_PMM_IN lost!!!!!!!!!!!!");
        }
        else
        {
            Rc.ppm_lost = 0;
            static uint8_t count = 0;
            if(++count > 20)
            {
                count = 0;
                ESP_LOGI(TAG, "PWM_IN lost_state = [%X]",lost_state);
                ESP_LOGI(TAG, "RC_ch= %d,%d,%d,%d,%d,%d,%d,%d,%d,%d",Rc.RC_ch[0],Rc.RC_ch[1],Rc.RC_ch[2],Rc.RC_ch[3],Rc.RC_ch[4],Rc.RC_ch[5],Rc.RC_ch[6],Rc.RC_ch[7],Rc.RC_ch[8],Rc.RC_ch[9]);
            }
        }
        vTaskDelay(10/portTICK_RATE_MS);
    }
}

static void hw_timer_callback1(void *arg)
{
    hw_time_count++;
}

static void GpioInputeConfig(void)
{
    gpio_config_t io_conf;    //Define GPIO Init Structure
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    //bit mask of the pins, use GPIO here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;

    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    //gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);

    //install gpio isr service
    gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin again
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_0_isr_handler, (void *) GPIO_INPUT_IO_0);
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_1_isr_handler, (void *) GPIO_INPUT_IO_1);
    gpio_isr_handler_add(GPIO_INPUT_IO_2, gpio_2_isr_handler, (void *) GPIO_INPUT_IO_2);
    gpio_isr_handler_add(GPIO_INPUT_IO_3, gpio_3_isr_handler, (void *) GPIO_INPUT_IO_3);
    gpio_isr_handler_add(GPIO_INPUT_IO_4, gpio_4_isr_handler, (void *) GPIO_INPUT_IO_4);
}

void RcIn_Init()
{
    hw_timer_init(hw_timer_callback1, NULL);
    ESP_LOGI(TAG, "Set hw_timer timing time 1000000us with reload");
    hw_timer_alarm_us(1000000, true);

    GpioInputeConfig();
    xTaskCreate(Task_Show_PwmIn, "Task_Show_PwmIn", 1024, NULL, 5, NULL);
}

#endif

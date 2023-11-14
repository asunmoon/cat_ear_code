#include "driver/i2c.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "time.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_rom_sys.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "ear.hpp"
#include "esp_random.h"
extern "C"
{
#include "MPU6050.h"
#include "inv_mpu.h"
#include "servo_smooth.h"
}

#define GATTS_TAG "GATTS_DEMO"
#define I2C_NUM I2C_NUM_0

struct mpu6050_Data mpu6050_data;
TaskHandle_t myHandle = NULL;
TaskHandle_t myaction = NULL;
u_int8_t myaction_id = 255;

ledc_timer_config_t servo_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
    .timer_num = LEDC_TIMER_1,
    .freq_hz = SERVO_FREQUENCY, // frequency of PWM signal
    .clk_cfg = LEDC_AUTO_CLK,   // Auto select the source clock
};
ledc_channel_config_t servo_channe0 = {
    .gpio_num = 8,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_1,
    .timer_sel = LEDC_TIMER_1,
    .duty = 0,
    .hpoint = 0,
    {0}};
ledc_channel_config_t servo_channe1 = {
    .gpio_num = 9,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_2,
    .timer_sel = LEDC_TIMER_1,
    .duty = 0,
    .hpoint = 0,
    {0}};
ledc_channel_config_t servo_channe2 = {
    .gpio_num = 18,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_3,
    .timer_sel = LEDC_TIMER_1,
    .duty = 0,
    .hpoint = 0,
    {0}};
ledc_channel_config_t servo_channe3 = {
    .gpio_num = 19,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_4,
    .timer_sel = LEDC_TIMER_1,
    .duty = 0,
    .hpoint = 0,
    {0}};

struct Servo_kinestate a0;
struct Servo_kinestate a1;
struct Servo_kinestate a2;
struct Servo_kinestate a3;
static void task_0(void *pvParameters);
static void task_1(void *pvParameters);
static void task_2(void *pvParameters);
static void task_3(void *pvParameters);
static void task_4(void *pvParameters);

static void task_0(void *pvParameters) // 低头0
{

    for (;;)
    {
    }
    vTaskDelete(NULL);
    ;
}
static void task_1(void *pvParameters) // 仰头1
{
    u_int32_t random0=esp_random();
    {
                a0.timestep = 0.0009 * 5,
                a0.thetai = 0,
                a0.thetaf = double(115 + (random0 % 5)),
                a0.omegai = 0,
                a0.omegaf = 0,
                a0.acci = 35,
                a0.accf = 35,
                a0.tf = 0.35;

                a1.timestep = 0.0009 * 5,
                a1.thetai = 0,
                a1.thetaf = double(90 + (random0 % 89 % 5)),
                a1.omegai = 0,
                a1.omegaf = 0,
                a1.acci = 35,
                a1.accf = 35,
                a1.tf = 0.35;

                a2.timestep = 0.0009 * 5,
                a2.thetai = 0,
                a2.thetaf = double(140 + (random0 % 5)),
                a2.omegai = 0,
                a2.omegaf = 0,
                a2.acci = 35,
                a2.accf = 35,
                a2.tf = 0.35;

                
                a3.timestep = 0.0009 * 5,
                a3.thetai = 0,
                a3.thetaf = double(140 + (random0 % 5)),
                a3.omegai = 0,
                a3.omegaf = 0,
                a3.acci = 35,
                a3.accf = 35,
                a3.tf = 0.35;
            }
            printf("a1=%lf,a3=%lf\n", a1.thetaf, a3.thetaf);
            Servo_run(servo_channe0, servo_channe1, servo_channe2, servo_channe3, &a0, &a1, &a2, &a3);
            vTaskDelay(10 / portTICK_RATE_MS);
    for (;;)
        ;
    vTaskDelete(NULL);
    ;
}
static void task_2(void *pvParameters) // 左侧头2
{
    for (;;)
        ;
    vTaskDelete(NULL);
    ;
}
static void task_3(void *pvParameters) // 右侧头3
{
    for (;;)
        ;
    vTaskDelete(NULL);
    ;
}
static void task_4(void *pvParameters)
{
    ESP_LOGI("aaa", "aaaaa");
    bool i = 0;
    a0.thetaf = 90;
    a2.thetaf = 90;
    a1.thetaf = 45;
    a3.thetaf = 45;
    Servo_run(servo_channe0, servo_channe1, servo_channe2, servo_channe3, &a0, &a1, &a2, &a3);
    for (;;)
    {
        u_int32_t random0 = esp_random();
        printf("random0 % 10=%d\n", (random0 % 10));
        if (true || random0 % 10 == 3)
        {
            random0 = esp_random();
            // u_int32_t duty = convert_servo_angle_to_duty(140);
            // ledc_set_duty(servo_channe1.speed_mode, servo_channe1.channel, duty);
            // ledc_update_duty(servo_channe1.speed_mode, servo_channe1.channel);
            // duty = convert_servo_angle_to_duty(45);
            // ledc_set_duty(servo_channe3.speed_mode, servo_channe3.channel, duty);
            // ledc_update_duty(servo_channe3.speed_mode, servo_channe3.channel);
            // vTaskDelay(200 / portTICK_RATE_MS);
            // vTaskDelay(10 / portTICK_RATE_MS);

            // duty = convert_servo_angle_to_duty(115);
            // ledc_set_duty(servo_channe1.speed_mode, servo_channe1.channel, duty);
            // ledc_update_duty(servo_channe1.speed_mode, servo_channe1.channel);
            // duty = convert_servo_angle_to_duty(90);
            // ledc_set_duty(servo_channe3.speed_mode, servo_channe3.channel, duty);
            // ledc_update_duty(servo_channe3.speed_mode, servo_channe3.channel);
            // vTaskDelay(200 / portTICK_RATE_MS);

            // u_int32_t duty1 = convert_servo_angle_to_duty(141);
            // ledc_set_duty(servo_channe1.speed_mode, servo_channe1.channel, duty1);
            // ledc_update_duty(servo_channe1.speed_mode, servo_channe1.channel);
            // u_int32_t duty2 = convert_servo_angle_to_duty(46);
            // ledc_set_duty(servo_channe3.speed_mode, servo_channe3.channel, duty2);
            // ledc_update_duty(servo_channe3.speed_mode, servo_channe3.channel);
            // vTaskDelay(200 / portTICK_RATE_MS);
            // vTaskDelay(10 / portTICK_RATE_MS);
            // for(int i=15;i>0;i--)
            // {
            // duty1 = duty1+i;
            // ledc_set_duty(servo_channe1.speed_mode, servo_channe1.channel, duty1);
            // ledc_update_duty(servo_channe1.speed_mode, servo_channe1.channel);
            // duty2 = duty2+i;
            // ledc_set_duty(servo_channe3.speed_mode, servo_channe3.channel, duty2);
            // ledc_update_duty(servo_channe3.speed_mode, servo_channe3.channel);
            // // vTaskDelay(10 / portTICK_RATE_MS);
            // vTaskDelay(10 / portTICK_RATE_MS);
            // }
            // vTaskDelay(10000 / portTICK_RATE_MS);
            a0.timestep = 0.0009 * 5,
            a0.tf = 0.05;
            a1.servo_channel = &servo_channe1,
            a1.timestep = 0.0009 * 5,
            a1.thetai = 0,
            a1.thetaf = double(144 + (random0 % 17 % 3)),
            // .omegai = 0,
                a1.omegaf = 0,
            a1.acci = double(55 - (random0 % 19 % 3)),
            a1.accf = 55,
            a1.tf = 0.05;
            a2.timestep = 0.0009 * 5,
            a2.tf = 0.05,
            a3.servo_channel = &servo_channe3,
            a3.timestep = 0.0009 * 5,
            a3.thetai = 115,
            a3.thetaf = double(41 - (random0 % 61 % 3)),
            // .omegai = 0,
                a3.omegaf = 0,
            a3.acci = double(55 - (random0 % 57 % 3)),
            a3.accf = 55,
            a3.tf = 0.05;
            Servo_run(servo_channe0, servo_channe1, servo_channe2, servo_channe3, &a0, &a1, &a2, &a3);
            vTaskDelay(50/portTICK_RATE_MS);
            a0.timestep = 0.0009 * 5,
            a0.tf = 0.05;
            a1.servo_channel = &servo_channe1,
            a1.timestep = 0.0009 * 5,
            
            a1.thetaf = 0,
            // .omegai = 0,
                a1.omegaf = 0,
            a1.acci = double(55 - (random0 % 19 % 3)),
            a1.accf = 55,
            a1.tf = 0.05;
            a2.timestep = 0.0009 * 5,
            a2.tf = 0.05;
            a3.servo_channel = &servo_channe3,
            a3.timestep = 0.0009 * 5,
            
            a3.thetaf = double(115- (random0 % 13 % 3)),
            // .omegai = 0,
                a3.omegaf = 0,
            a3.acci = double(55 - (random0 % 79 % 3)),
            a3.accf = 55,
            a3.tf = 0.05;
            Servo_run(servo_channe0, servo_channe1, servo_channe2, servo_channe3, &a0, &a1, &a2, &a3);
            vTaskDelay(80/portTICK_RATE_MS);
            a0.timestep = 0.0009 * 5,
            a0.tf = 0.05;
            a1.servo_channel = &servo_channe1,
            a1.timestep = 0.0009 * 5,
            
            a1.thetaf = double(144 + (random0 % 17 % 3)),
            // .omegai = 0,
                a1.omegaf = 0,
            a1.acci = double(55 - (random0 % 19 % 3)),
            a1.accf = 55,
            a1.tf = 0.05;
            a2.timestep = 0.0009 * 5,
            a2.tf = 0.05,
            a3.servo_channel = &servo_channe3,
            a3.timestep = 0.0009 * 5,
            
            a3.thetaf = double(41 - (random0 % 61 % 3)),
            // .omegai = 0,
                a3.omegaf = 0,
            a3.acci = double(55 - (random0 % 119 % 3)),
            a3.accf = 55,
            a3.tf = 0.05;
            Servo_run(servo_channe0, servo_channe1, servo_channe2, servo_channe3, &a0, &a1, &a2, &a3);
            vTaskDelay(80/portTICK_RATE_MS);
            a0.timestep = 0.0009 * 5,
            a0.tf = 0.05;
            a1.servo_channel = &servo_channe1,
            a1.timestep = 0.0009 * 5,
            
            a1.thetaf = 0,
            // .omegai = 0,
                a1.omegaf = 0,
            a1.acci = double(55 - (random0 % 19 % 3)),
            a1.accf = 55,
            a1.tf = 0.05;
            a2.timestep = 0.0009 * 5,
            a2.tf = 0.05;
            a3.servo_channel = &servo_channe3,
            a3.timestep = 0.0009 * 5,
            
            a3.thetaf = double(115- (random0 % 113 % 3)),
            // .omegai = 0,
                a3.omegaf = 0,
            a3.acci = double(55 - (random0 % 57 % 3)),
            a3.accf = 55,
            a3.tf = 0.05;
            Servo_run(servo_channe0, servo_channe1, servo_channe2, servo_channe3, &a0, &a1, &a2, &a3);
            vTaskDelay(1500/portTICK_RATE_MS);
            // a0.timestep = 0.0009 * 5,
            // a0.tf = 0.3;
            // a1.servo_channel = &servo_channe1,
            // a1.timestep = 0.0009 * 5,
            // a1.thetai = 145,
            // a1.thetaf = double(43 + (random0 % 17 % 5)),
            // // .omegai = 0,
            //     a1.omegaf = 0,
            // a1.acci = double(55 - (random0 % 19 % 3)),
            // a1.accf = 55,
            // a1.tf = 0.3;
            // a2.timestep = 0.0009 * 5,
            // a2.tf = 0.3;
            // a3.servo_channel = &servo_channe3,
            // a3.timestep = 0.0009 * 5,
            // a3.thetai = 40,
            // a3.thetaf = double(115 - (random0 % 61 % 5)),
            // // .omegai = 0,
            //     a3.omegaf = 0,
            // a3.acci = double(55 - (random0 % 57 % 3)),
            // a3.accf = 55,
            // a3.tf = 0.3;
            // Servo_run(servo_channe0, servo_channe1, servo_channe2, servo_channe3, &a0, &a1, &a2, &a3);

            // for (int i = 2; i > 0; i--)
            // {

            //         a1.servo_channel = &servo_channe1,
            //         a1.timestep = 0.0009 * 5,
            //         // .thetai = 0,
            //         a1.thetaf = double(140 ),
            //         a1.omegai = double(140),
            //         a1.omegaf = 0,
            //         a1.acci = 55,
            //         a1.accf = 35,
            //         a1.tf = 0.25;

            //         a3.servo_channel = &servo_channe3,
            //         a3.timestep = 0.0009 * 5,
            //         // .thetai = 0,
            //         a3.thetaf = double(65 ),
            //         a3.omegai = double(65 ),
            //         a3.omegaf = 0,
            //         a3.acci = 50,
            //         a3.accf = 35,
            //         a3.tf = 0.25;
            //     printf("%d:1a1=%lf,a3=%lf\n",i,a1.thetai,a3.thetai);
            //     // Servo_run(servo_channe0, servo_channe1, servo_channe2, servo_channe3, &a0, &a1, &a2, &a3);
            //     u_int32_t  duty = convert_servo_angle_to_duty(140);
            //     ledc_set_duty(servo_channe1.speed_mode, servo_channe1.channel, duty);
            //     ledc_update_duty(servo_channe1.speed_mode, servo_channe1.channel);
            //       duty = convert_servo_angle_to_duty(45);
            //     ledc_set_duty(servo_channe3.speed_mode, servo_channe3.channel, duty);
            //     ledc_update_duty(servo_channe3.speed_mode, servo_channe3.channel);
            //     vTaskDelay(200 / portTICK_RATE_MS);
            //     vTaskDelay(10 / portTICK_RATE_MS);
            //     if (i == 1)
            //     {

            //             a1.servo_channel = &servo_channe1,
            //             a1.timestep = 0.0009 * 5,
            //             // .thetai = 0,
            //             a1.thetaf = double(40 + (random0 % 17 % 5)),
            //             // .omegai = 0,
            //             a1.omegaf = 0,
            //             a1.acci = double(35 - (random0 % 19 % 3)),
            //             a1.accf = 30,
            //             a1.tf = 0.3;

            //             a3.servo_channel = &servo_channe3,
            //             a3.timestep = 0.0009 * 5,
            //             // .thetai = 0,
            //             a3.thetaf = double(115 - (random0 % 61 % 5)),
            //             // .omegai = 0,
            //             a3.omegaf = 0,
            //             a3.acci = double(35 - (random0 % 57 % 3)),
            //             a3.accf = 30,
            //             a3.tf = 0.3;
            //             u_int32_t  duty = convert_servo_angle_to_duty(115);
            //     ledc_set_duty(servo_channe1.speed_mode, servo_channe1.channel, duty);
            //     ledc_update_duty(servo_channe1.speed_mode, servo_channe1.channel);
            //       duty = convert_servo_angle_to_duty(90);
            //     ledc_set_duty(servo_channe3.speed_mode, servo_channe3.channel, duty);
            //     ledc_update_duty(servo_channe3.speed_mode, servo_channe3.channel);
            //     vTaskDelay(200 / portTICK_RATE_MS);
            //     }
            //     else if(i==2)
            //     {

            //             a1.servo_channel = &servo_channe1,
            //             a1.timestep = 0.0009 * 5,
            //             // .thetai = 0,
            //             a1.thetaf = double(40+30 + (random0 % 17 % 5)),
            //             // .omegai = 2,
            //             a1.omegaf = 0,
            //             a1.acci = double(155 - (random0 % 19 % 3)),
            //             a1.accf = 155,
            //             a1.tf = 0.25;

            //             a3.servo_channel = &servo_channe3,
            //             a3.timestep = 0.0009 * 5,
            //             // .thetai = 0,
            //             a3.thetaf = double(115-30 - (random0 % 61 % 5)),
            //             // .omegai = 0,
            //             a3.omegaf = 0,
            //             a3.acci = double(155 - (random0 % 57 % 3)),
            //             a3.accf = 155,
            //             a3.tf = 0.25;
            //             Servo_run(servo_channe0, servo_channe1, servo_channe2, servo_channe3, &a0, &a1, &a2, &a3);
            //     }
            //     printf("%d:2a1=%lf,a3=%lf\n",i,a1.thetai,a3.thetai);

            //     if(i == 1)
            //     {
            //          vTaskDelay(600 / portTICK_RATE_MS);
            //          ESP_LOGI("A","a");
            //     }
            //     vTaskDelay(10 / portTICK_RATE_MS);
            // }
        }
        else
        {
            {
                a0.timestep = 0.0009 * 5,
                a0.tf = 0.35;

                a1.servo_channel = &servo_channe1,
                a1.timestep = 0.0009 * 5,
                a1.thetai = 0,
                a1.thetaf = double(40 + (random0 % 89 % 5)),
                a1.omegai = 0,
                a1.omegaf = 0,
                a1.acci = 35,
                a1.accf = 35,
                a1.tf = 0.35;

                a2.timestep = 0.0009 * 5,
                a2.tf = 0.35;

                a3.servo_channel = &servo_channe3,
                a3.timestep = 0.0009 * 5,
                a3.thetai = 0,
                a3.thetaf = double(140 + (random0 % 5)),
                a3.omegai = 0,
                a3.omegaf = 0,
                a3.acci = 35,
                a3.accf = 35,
                a3.tf = 0.35;
            }
            printf("a1=%lf,a3=%lf\n", a1.thetaf, a3.thetaf);
            Servo_run(servo_channe0, servo_channe1, servo_channe2, servo_channe3, &a0, &a1, &a2, &a3);
            vTaskDelay(10 / portTICK_RATE_MS);

            {
                a0.timestep = 0.0009 * 5,
                a0.tf = 0.35;

                a1.servo_channel = &servo_channe1,
                a1.timestep = 0.0009 * 5,
                a1.thetai = 0,
                a1.thetaf = double(115 + (esp_random() % 5)),
                a1.omegai = 0,
                a1.omegaf = 0,
                a1.acci = 35,
                a1.accf = 35,
                a1.tf = 0.35;

                a2.timestep = 0.0009 * 5,
                a2.tf = 0.35;

                a3.servo_channel = &servo_channe3,
                a3.timestep = 0.0009 * 5,
                a3.thetai = 0,
                a3.thetaf = double(65 + (esp_random() % 5)),
                a3.omegai = 0,
                a3.omegaf = 0,
                a3.acci = 35,
                a3.accf = 35,
                a3.tf = 0.35;
            }
            printf("a1=%lf,a3=%lf\n", a1.thetaf, a3.thetaf);
            Servo_run(servo_channe0, servo_channe1, servo_channe2, servo_channe3, &a0, &a1, &a2, &a3);
            vTaskDelay(10 / portTICK_RATE_MS);

            vTaskDelay(1000 + random0 % 1000 / portTICK_RATE_MS);
        }
    }
    vTaskDelete(NULL);
}
void which_run()
{
    if (myaction == NULL)
    {
        if (mpu6050_data.pitch < -20) // 低头0
        {
            // ESP_LOGI("task0", "begin");
            // vTaskDelay(1000 / portTICK_RATE_MS);
            // myaction_id = 0;
            // xTaskCreatePinnedToCore(&task_0, "task_0", 1024 * 10, NULL, 6, &myaction, 0);
        }
        else if (mpu6050_data.pitch > 20) // 仰头1
        {
            // ESP_LOGI("task1", "begin");
            // vTaskDelay(1000 / portTICK_RATE_MS);
            // myaction_id = 1;
            // xTaskCreatePinnedToCore(&task_1, "task_2", 1024 * 10, NULL, 6, &myaction, 0);
        }
        else if (mpu6050_data.roll < -20) // 左侧头2
        {
            // ESP_LOGI("task2", "begin");
            // vTaskDelay(1000 / portTICK_RATE_MS);
            // myaction_id = 2;
            // xTaskCreatePinnedToCore(&task_2, "task_2", 1024 * 10, NULL, 6, &myaction, 0);
        }
        else if (mpu6050_data.roll > 20) // 右侧头3
        {
            // ESP_LOGI("task3", "begin");
            // vTaskDelay(1000 / portTICK_RATE_MS);
            // myaction_id = 3;
            // xTaskCreatePinnedToCore(&task_3, "task_3", 1024 * 10, NULL, 6, &myaction, 0);
        }
        else
        { // 中间4
            ESP_LOGI("task4", "begin");
            vTaskDelay(1000 / portTICK_RATE_MS);
            myaction_id = 4;
            xTaskCreatePinnedToCore(&task_4, "task_4", 1024 * 10, NULL, 6, &myaction, 0);
        }
    }
    else
    {
        // ESP_LOGI("bbb","NOT NULL");
        if (myaction_id == 0 && (mpu6050_data.pitch > -15 && mpu6050_data.pitch < 0)) // 低头0
        {
            vTaskDelete(myaction);
            myaction = NULL;
            myaction_id = 255;
        }
        else if (myaction_id == 1 && (mpu6050_data.pitch < 15 && mpu6050_data.pitch > 0)) // 仰头1
        {
            vTaskDelete(myaction);
            myaction = NULL;
            myaction_id = 255;
        }
        else if (myaction_id == 2 && (mpu6050_data.roll > -15 && mpu6050_data.roll < 0)) // 左侧头2
        {
            vTaskDelete(myaction);
            myaction = NULL;
            myaction_id = 255;
        }
        else if (myaction_id == 3 && (mpu6050_data.roll < 15 && mpu6050_data.roll > 0)) // 右侧头3
        {
            vTaskDelete(myaction);
            myaction = NULL;
            myaction_id = 255;
        }
        else if (myaction_id == 4 && (abs(mpu6050_data.pitch) > 20 || abs(mpu6050_data.roll) > 20))
        { // 中间4
            // ESP_LOGI("bbb","bbbbbb");
            vTaskDelete(myaction);
            myaction = NULL;
            myaction_id = 255;
        }
    }
}
static void s_task(void *pvParameters);
static void a_task(void *pvParameters)
{
    printf("task begin\n");
    for (;;)
    {
        ESP_LOGI("a", "a\n");
        vTaskDelay(200 / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}
static void task(void *pvParameters)
{
    while (1)
    {
        ESP_LOGE("TAG", "Sub task is running");
        // vTaskDelay(1000/portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}
static void mpu6050_task(void *pvParameters)
{
    MPU6050 mpu(i2c_gpio_scl, i2c_gpio_sda, I2C_NUM);

    if (!mpu.init())
    {
        ESP_LOGE("mpu6050", "init failed!");
        vTaskDelete(0);
    }
    ESP_LOGI("mpu6050", "init success!");

    KALMAN pfilter(0.005);
    KALMAN rfilter(0.005);

    uint32_t lasttime = 0;
    int count = 0;
    Servo_run(servo_channe0, servo_channe1, servo_channe2, servo_channe3, &a0, &a1, &a2, &a3);
    while (1)
    {
        mpu6050_data.ax = -mpu.getAccX();
        mpu6050_data.ay = -mpu.getAccY();
        mpu6050_data.az = -mpu.getAccZ();
        mpu6050_data.gx = mpu.getGyroX();
        mpu6050_data.gy = mpu.getGyroY();
        mpu6050_data.gz = mpu.getGyroZ();
        mpu6050_data.pitch = atan(mpu6050_data.ax / mpu6050_data.az) * 57.2958;
        mpu6050_data.roll = atan(mpu6050_data.ay / mpu6050_data.az) * 57.2958;
        mpu6050_data.fpitch = pfilter.filter(mpu6050_data.pitch, mpu6050_data.gy);
        mpu6050_data.froll = rfilter.filter(mpu6050_data.roll, -mpu6050_data.gx);
        count++;
        if (esp_log_timestamp() / 100 != lasttime)
        {
            lasttime = esp_log_timestamp() / 100;
            // printf("Samples:%d ", lasttime);
            // printf("Pitch:%lf \n", mpu6050_data.pitch);
            count = 0;
            which_run();
            // if (mpu6050_data.pitch < -45)
            // {
            //     if (myHandle == NULL)
            //     {
            //         xTaskCreatePinnedToCore(&s_task, "a_task", 1024 * 10, NULL, 6, &myHandle, 0);
            //     }
            // }
            // if (myHandle != NULL && mpu6050_data.pitch > 0)
            // {
            //     vTaskDelete(myHandle);
            //     myHandle = NULL;
            // }
            // printf(" Acc:(%4.2f,%4.2f,%4.2f)", mpu6050_data.ax, mpu6050_data.ay, mpu6050_data.az);
            // printf("Gyro:(%6.3f,%6.3f,%6.3f)", mpu6050_data.gx, mpu6050_data.gy, mpu6050_data.gz);
            // printf(" Pitch:%6.3f ", mpu6050_data.pitch);
            // printf(" Roll:%6.3f ", mpu6050_data.roll);
            // printf(" FPitch:%6.3f ", mpu6050_data.fpitch);
            // printf(" FRoll:%6.3f \n", mpu6050_data.froll);
        }
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

static void s_task(void *pvParameters)
{

    a0 = {
        .servo_channel = &servo_channe0,
        .timestep = 0.0009 * 5,
        .thetai = 0,
        .thetaf = 180,
        .omegai = 0,
        .omegaf = 0,
        .acci = 35,
        .accf = 35,
        .tf = 0.3};
    a1 = {
        .servo_channel = &servo_channe1,
        .timestep = 0.0009 * 5,
        .thetai = 0,
        .thetaf = 45,
        .omegai = 0,
        .omegaf = 0,
        .acci = 35,
        .accf = 35,
        .tf = 0.3};
    a2 = {
        .servo_channel = &servo_channe2,
        .timestep = 0.0009 * 5,
        .thetai = 0,
        .thetaf = 180,
        .omegai = 0,
        .omegaf = 0,
        .acci = 35,
        .accf = 35,
        .tf = 0.3};
    a3 = {
        .servo_channel = &servo_channe3,
        .timestep = 0.0009 * 5,
        .thetai = 0,
        .thetaf = 45,
        .omegai = 0,
        .omegaf = 0,
        .acci = 35,
        .accf = 35,
        .tf = 0.3};
    // printf("1thetai:%lf,thetaf:%lf\n",a0.thetai,a0.thetaf);
    // vTaskDelay(1000 / portTICK_RATE_MS);

    Servo_run(servo_channe0, servo_channe1, servo_channe2, servo_channe3, &a0, &a1, &a2, &a3);
    // printf("%f,%f,%f\n", a2.omegai, a2.acci, a2.thetai);
    // MPU6050 mpu(i2c_gpio_scl, i2c_gpio_sda, I2C_NUM);
    // if (!mpu.init())
    // {
    //     ESP_LOGE("mpu6050", "init failed!");
    //     vTaskDelay(0);
    // }
    // ESP_LOGI("mpu6050", "init success!");
    //  mpu6050_task();

    // servo_control_task();

    // for (;;)
    // {
    //     printf(" Pitch:%6.3f \n", mpu6050_data.pitch);
    //     if (mpu6050_data.pitch < -45)
    //     {
    //         if (myHandle == NULL)
    //         {
    //             // xTaskCreatePinnedToCore(task,"myTask",1024,NULL,1,&myHandle);
    //             xTaskCreatePinnedToCore(&task, "myTask", 2048 * 8, NULL, 7, &myHandle, 0);
    //         }
    //     }
    //     if (myHandle != NULL)
    //     {
    //         vTaskDelete(myHandle);
    //     }
    //     vTaskDelay(100 / portTICK_RATE_MS);
    //     // ear_task(mpu6050_data, servo_timer, servo_channe0, servo_channe1, servo_channe2, servo_channe3, &a0, &a1, &a2, &a3);
    //     // Servo_run(servo_channe0, servo_channe1, servo_channe2, servo_channe3, &a0, &a1, &a2, &a3);
    // }
    for (;;)
    {
        vTaskDelay(200 / portTICK_RATE_MS);
        ESP_LOGI("tag", "a");
    }
    vTaskDelete(NULL);
}

extern "C" void app_main(void)
{
    ledc_timer_config(&servo_timer);
    ledc_channel_config(&servo_channe0);
    ledc_channel_config(&servo_channe1);
    ledc_channel_config(&servo_channe2);
    ledc_channel_config(&servo_channe3);
    vTaskDelay(1);
    a0 = {
        .servo_channel = &servo_channe0,
        .timestep = 0.0009 * 5,
        .thetai = 0,
        .thetaf = 180,
        .omegai = 0,
        .omegaf = 0,
        .acci = 35,
        .accf = 35,
        .tf = 0.3};
    a1 = {
        .servo_channel = &servo_channe1,
        .timestep = 0.0009 * 5,
        .thetai = 0,
        .thetaf = 40,
        .omegai = 0,
        .omegaf = 0,
        .acci = 35,
        .accf = 35,
        .tf = 0.3};
    a2 = {
        .servo_channel = &servo_channe2,
        .timestep = 0.0009 * 5,
        .thetai = 0,
        .thetaf = 180,
        .omegai = 0,
        .omegaf = 0,
        .acci = 35,
        .accf = 35,
        .tf = 0.3};
    a3 = {
        .servo_channel = &servo_channe3,
        .timestep = 0.0009 * 5,
        .thetai = 0,
        .thetaf = 140,
        .omegai = 0,
        .omegaf = 0,
        .acci = 35,
        .accf = 35,
        .tf = 0.3};
    // testservo(servo_channe0, servo_channe2);
    // testservo(servo_channe1, servo_channe3);
    vTaskDelay(1000 / portTICK_RATE_MS);
    printf("hello world\n");

    xTaskCreatePinnedToCore(&mpu6050_task, "mpu6050_task", 2048 * 2, NULL, 5, NULL, 0);
}

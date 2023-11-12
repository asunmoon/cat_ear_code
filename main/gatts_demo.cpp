// /*
//    This example code is in the Public Domain (or CC0 licensed, at your option.)

//    Unless required by applicable law or agreed to in writing, this
//    software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
//    CONDITIONS OF ANY KIND, either express or implied.
// */

// /****************************************************************************
// *
// * This demo showcases BLE GATT server. It can send adv data, be connected by client.
// * Run the gatt_client demo, the client demo will automatically connect to the gatt_server demo.
// * Client demo will enable gatt_server's notify after connection. The two devices will then exchange
// * data.
// *
// ****************************************************************************/
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
extern "C"
{
#include "MPU6050.h"
#include "inv_mpu.h"
#include "servo_smooth.h"
}
// #include "ble.h"

#define GATTS_TAG "GATTS_DEMO"
#define I2C_NUM I2C_NUM_0
///
struct mpu6050_Data mpu6050_data;
TaskHandle_t myHandle = NULL;
// void app_main(void)
// {
//     ledc_channel_config_t servo_channel;
//     // printf("hhhh\n");
//     // vTaskDelay(1000/portTICK_RATE_MS);
//     // while(1)
//     // {
//     //     servo_control_task();
//     // }
//     esp_err_t ret;

//     // Initialize NVS.
//     ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK( ret );

//     ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

//     esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
//     ret = esp_bt_controller_init(&bt_cfg);
//     if (ret) {
//         ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
//         return;
//     }

//     ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
//     if (ret) {
//         ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
//         return;
//     }
//     ret = esp_bluedroid_init();
//     if (ret) {
//         ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
//         return;
//     }
//     ret = esp_bluedroid_enable();
//     if (ret) {
//         ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
//         return;
//     }

//     ret = esp_ble_gatts_register_callback(gatts_event_handler);
//     if (ret){
//         ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
//         return;
//     }
//     ret = esp_ble_gap_register_callback(gap_event_handler);
//     if (ret){
//         ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
//         return;
//     }
//     ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
//     if (ret){
//         ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
//         return;
//     }
//     ret = esp_ble_gatts_app_register(PROFILE_B_APP_ID);
//     if (ret){
//         ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
//         return;
//     }
//     esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
//     if (local_mtu_ret){
//         ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
//     }

//     return;
// }

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
            printf("Samples:%d ", lasttime);
            printf("Pitch:%lf \n", mpu6050_data.pitch);
            count = 0;
            if (mpu6050_data.pitch < -45)
            {
                if (myHandle == NULL)
                {
                    xTaskCreatePinnedToCore(&s_task, "a_task", 1024 * 10, NULL, 6, &myHandle, 0);
                }
            }
            if(myHandle != NULL&&mpu6050_data.pitch>0)
            {
                vTaskDelete(myHandle);
                myHandle=NULL;
            }
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
    ledc_timer_config_t servo_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .timer_num = LEDC_TIMER_1,
        .freq_hz = SERVO_FREQUENCY, // frequency of PWM signal
        .clk_cfg = LEDC_AUTO_CLK,   // Auto select the source clock
    };
    ledc_timer_config(&servo_timer);

    ledc_channel_config_t servo_channe0 = {
        .gpio_num = 8,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0,
        {0}};
    ledc_channel_config(&servo_channe0);
    ledc_channel_config_t servo_channe1 = {
        .gpio_num = 9,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0,
        {0}};
    ledc_channel_config(&servo_channe1);
    ledc_channel_config_t servo_channe2 = {
        .gpio_num = 18,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_3,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0,
        {0}};
    ledc_channel_config(&servo_channe2);
    ledc_channel_config_t servo_channe3 = {
        .gpio_num = 19,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_4,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0,
        {0}};
    ledc_channel_config(&servo_channe3);

    testservo(servo_channe0,servo_channe2);
    testservo(servo_channe1,servo_channe3);

    struct Servo_kinestate a0 = {
        .servo_channel = &servo_channe0,
        .timestep = 0.0009,
        .thetai = 0,
        .thetaf = 180,
        .omegai = 0,
        .omegaf = 0,
        .acci = 5,
        .accf = 15,
        .tf = 0.8};
    struct Servo_kinestate a1 = {
        .servo_channel = &servo_channe1,
        .timestep = 0.0009,
        .thetai = 0,
        .thetaf = 45,
        .omegai = 0,
        .omegaf = 0,
        .acci = 5,
        .accf = 15,
        .tf = 0.8};
    struct Servo_kinestate a2 = {
        .servo_channel = &servo_channe2,
        .timestep = 0.0009,
        .thetai = 0,
        .thetaf = 180,
        .omegai = 0,
        .omegaf = 0,
        .acci = 5,
        .accf = 15,
        .tf = 0.8};
    struct Servo_kinestate a3 = {
        .servo_channel = &servo_channe3,
        .timestep = 0.0009,
        .thetai = 0,
        .thetaf = 45,
        .omegai = 0,
        .omegaf = 0,
        .acci = 5,
        .accf = 15,
        .tf = 0.8};
    // printf("1thetai:%lf,thetaf:%lf\n",a0.thetai,a0.thetaf);
    // vTaskDelay(1000 / portTICK_RATE_MS);

    Servo_run(servo_channe0, servo_channe1,servo_channe2, servo_channe3, &a0, &a1,&a2,&a3);
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
    
    for (;;)
    {
        printf(" Pitch:%6.3f \n", mpu6050_data.pitch);
        if (mpu6050_data.pitch < -45)
        {
            if (myHandle == NULL)
            {
                // xTaskCreatePinnedToCore(task,"myTask",1024,NULL,1,&myHandle);
                xTaskCreatePinnedToCore(&task, "myTask", 2048 * 8, NULL, 7, &myHandle, 0);
            }
        }
        if (myHandle != NULL)
        {
            vTaskDelete(myHandle);
        }
        vTaskDelay(100 / portTICK_RATE_MS);
        // ear_task(mpu6050_data, servo_timer, servo_channe0, servo_channe1, servo_channe2, servo_channe3, &a0, &a1, &a2, &a3);
        // Servo_run(servo_channe0, servo_channe1, servo_channe2, servo_channe3, &a0, &a1, &a2, &a3);
    }
}


extern "C" void app_main(void)
{

    printf("hello world\n");

    xTaskCreatePinnedToCore(&mpu6050_task, "mpu6050_task", 2048 * 2, NULL, 5, NULL, 0);

    // xTaskCreatePinnedToCore(&s_task, "s_task", 1024 * 6, NULL, 6, NULL, 0);
    // for(int i=0;i<255;i++)
    // {
    //     vTaskDelay(200/portTICK_RATE_MS);
    //     printf("i=%d\n",i);
    //     if(i==25)
    //     {
    //         if(myHandle==NULL)
    //         {
    //             xTaskCreatePinnedToCore(&a_task, "a_task", 1024 * 6, NULL, 6, &myHandle, 0);
    //         }
    //     }
    //     if (i>50)
    //     {
    //          if(myHandle!=NULL)
    //         {
    //             vTaskDelete(myHandle);
    //             myHandle = NULL;
    //         }
    //     }

    // }
}

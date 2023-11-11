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
#include "driver/ledc.h"

#define SERVO_MIN_PULSEWIDTH_US (500)  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US (2500) // Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE (180)         // Maximum angle in degree upto which servo can rotate
#define SERVO_FREQUENCY (50)           // PWM frequency

#ifndef servo_smooth
#define servo_smooth

struct Servo_kinestate
{
    ledc_channel_config_t *servo_channel;
    double timestep; // 时间步幅
    double thetai;   // 起点角度
    double thetaf;   // 终点角度
    double omegai;   // 起点角速度
    double omegaf;   // 终点角速度
    double acci;     // 起点加速度
    double accf;     // 终点加速度
    double tf;       // 总运动时间
};
#endif
uint32_t convert_servo_angle_to_duty(double angle);

void Servo_run(ledc_channel_config_t  servo_channe0,ledc_channel_config_t  servo_channe1,
ledc_channel_config_t  servo_channe3,ledc_channel_config_t  servo_channe4,
struct Servo_kinestate* servo_kinestate0, struct Servo_kinestate* servo_kinestate1,
struct Servo_kinestate* servo_kinestate3, struct Servo_kinestate* servo_kinestate4);
void servo_control_task_begin(ledc_timer_config_t servo_timer, ledc_channel_config_t servo_channel
                                , ledc_channel_config_t servo_channe2);

void servo_control_task();
void testservo(ledc_channel_config_t servo_channel,ledc_channel_config_t servo_channe2);
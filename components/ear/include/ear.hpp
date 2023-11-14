#include <string.h>
#include <stdio.h>
#include <stdlib.h>
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
#include "driver/i2c.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
extern "C"{ 
    // #include "inv_mpu.h"
    #include "servo_smooth.h"
    }
#include "mpu6050.hpp"
#include "kalmanfilter.hpp"
#include <cmath>

struct mpu6050_Data{
    float ax,ay,az,gx,gy,gz;
    float pitch, roll;
    float fpitch, froll;
    };


static gpio_num_t i2c_gpio_sda = (gpio_num_t)4;
static gpio_num_t i2c_gpio_scl = (gpio_num_t)5;
#define I2C_NUM I2C_NUM_0


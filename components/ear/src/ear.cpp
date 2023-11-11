#include "ear.hpp"

uint32_t lasttime = 0;
int count = 0;

void ear_run(mpu6050_Data mpu6050_data, struct Servo_kinestate *a0,
             struct Servo_kinestate *a1,
             struct Servo_kinestate *a2,
             struct Servo_kinestate *a3)
{
    if (mpu6050_data.pitch < -15) // 低头
    {
        // 设置struct Servo_kinestate a0;
        // (*a0).thetai = 0;
        // (*a0).thetaf = 180;
        // (*a0).omegai = 0;
        // (*a0).omegaf = 0;
        // (*a0).acci = 5;
        // (*a0).accf = 15;
        // (*a0).timestep = 0.0009;
        // (*a0).tf = 0.8;
        // 设置struct Servo_kinestate a1;
        // (*a1).thetai = 0;
        (*a1).thetaf = 90;
        // (*a1).omegai = 0;
        // (*a1).omegaf = 0;
        // (*a1).acci = 5;
        // (*a1).accf = 15;
        // (*a1).timestep = 0.0009;
        // (*a1).tf = 0.8;
        // 设置struct Servo_kinestate a2;
        // (*a2).thetai = 0;
        // (*a2).thetaf = 180;
        // (*a2).omegai = 0;
        // (*a2).omegaf = 0;
        // (*a2).acci = 5;
        // (*a2).accf = 15;
        // (*a2).timestep = 0.0009;
        // (*a2).tf = 0.8;
        // 设置struct Servo_kinestate a3;
        // (*a3).thetai = 0;
        (*a3).thetaf = 90;
        // (*a3).omegai = 0;
        // (*a3).omegaf = 0;
        // (*a3).acci = 5;
        // (*a3).accf = 15;
        // (*a3).timestep = 0.0009;
        // (*a3).tf = 0.8;
    }
    else if (mpu6050_data.pitch > 15) // 仰头
    {
    }
    else if (mpu6050_data.roll < -15) // 左侧头
    {
    }
    else if (mpu6050_data.roll > 15) // 右侧头
    {
    }

    // Servo_run(servo_channel, servo_channe2, a1, a2);
}

extern "C" void ear_task(mpu6050_Data mpu6050_data, ledc_timer_config_t servo_timer,
                         ledc_channel_config_t servo_channe0,
                         ledc_channel_config_t servo_channe1,
                         ledc_channel_config_t servo_channe2,
                         ledc_channel_config_t servo_channe3,
                         struct Servo_kinestate *a0,
                         struct Servo_kinestate *a1,
                         struct Servo_kinestate *a2,
                         struct Servo_kinestate *a3)
{

    // printf("21thetai:%lf,thetaf:%lf\n", (*a0).thetai, (*a0).thetaf);

    // (*a0).thetaf = 90;
    // (*a1).thetaf = 45;
    // (*a2).thetaf = 90;
    // (*a3).thetaf = 45;
    // printf("22thetai:%lf,thetaf:%lf\n", (*a0).thetai, (*a0).thetaf);
    // printf("qqqqqqqqqqqqqqqqq\n");
    // vTaskDelay(1000 / portTICK_RATE_MS);
    // Servo_run(servo_channe0, servo_channe1, servo_channe2, servo_channe3, a0, a1, a2, a3);

    // 00
    printf(" Acc:(%4.2f,%4.2f,%4.2f)", mpu6050_data.ax, mpu6050_data.ay, mpu6050_data.az);
    printf("Gyro:(%6.3f,%6.3f,%6.3f)", mpu6050_data.gx, mpu6050_data.gy, mpu6050_data.gz);
    printf(" Pitch:%6.3f ", mpu6050_data.pitch);
    printf(" Roll:%6.3f ", mpu6050_data.roll);
    printf(" FPitch:%6.3f ", mpu6050_data.fpitch);
    printf(" FRoll:%6.3f \n", mpu6050_data.froll);
    // ear_run(mpu6050_data, a0, a1, a2, a3);
}

#include "servo_smooth.h"

uint32_t convert_servo_angle_to_duty(double angle)
{
    uint32_t us;
    if (angle > SERVO_MAX_DEGREE)
    {
        angle = SERVO_MAX_DEGREE;
    }
    else if (angle < 0)
    {
        angle = 0;
    }
    us = SERVO_MIN_PULSEWIDTH_US + (angle) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / SERVO_MAX_DEGREE;
    int a = angle;
    // printf("angle: %d  us:%d\r\n", a, us);
    return (int)(SERVO_FREQUENCY * us * 8191 / 1000000); // 仅针对LEDC_TIMER_13_BIT
}

void five_data(struct Servo_kinestate servo_kinestate0, double *theta0, double *omega0, double *acc0, double t)
{
    double a0 = 0;
    double a1 = 0;
    double a2 = 0;
    double a3 = 0;
    double a4 = 0;
    double a5 = 0;
    a0 = servo_kinestate0.thetai;
    a1 = servo_kinestate0.omegai;
    a2 = servo_kinestate0.acci / 2;
    a3 = (20 * servo_kinestate0.thetaf - 20 * servo_kinestate0.thetai - (8 * servo_kinestate0.omegaf + 12 * servo_kinestate0.omegai) * servo_kinestate0.tf - (3 * servo_kinestate0.acci - servo_kinestate0.accf) * (pow(servo_kinestate0.tf, 2))) / (2 * (pow(servo_kinestate0.tf, 3)));
    a4 = (30 * servo_kinestate0.thetai - 30 * servo_kinestate0.thetaf + (14 * servo_kinestate0.omegaf + 16 * servo_kinestate0.omegai) * servo_kinestate0.tf + (3 * servo_kinestate0.acci - 2 * servo_kinestate0.accf) * (pow(servo_kinestate0.tf, 2))) / (2 * (pow(servo_kinestate0.tf, 4)));
    a5 = (12 * servo_kinestate0.thetaf - 12 * servo_kinestate0.thetai - (6 * servo_kinestate0.omegaf + 6 * servo_kinestate0.omegai) * servo_kinestate0.tf - (servo_kinestate0.acci - servo_kinestate0.accf) * (pow(servo_kinestate0.tf, 2))) / (2 * (pow(servo_kinestate0.tf, 5)));
    *theta0 = a0 + a1 * t + a2 * (pow(t, 2)) + a3 * (pow(t, 3)) + a4 * (pow(t, 4)) + a5 * (pow(t, 5));
    *omega0 = a1 + 2 * a2 * t + 3 * a3 * (pow(t, 2)) + 4 * a4 * (pow(t, 3)) + 5 * a5 * (pow(t, 4));
    *acc0 = 2 * a2 + 6 * a3 * t + 12 * a4 * (pow(t, 2)) + 20 * a5 * (pow(t, 3));
}



void Servo_run(ledc_channel_config_t servo_channe0, ledc_channel_config_t servo_channe1,
               ledc_channel_config_t servo_channe2, ledc_channel_config_t servo_channe3,
               struct Servo_kinestate *servo_kinestate0, struct Servo_kinestate *servo_kinestate1,
               struct Servo_kinestate *servo_kinestate2, struct Servo_kinestate *servo_kinestate3)
{
    double theta0;
    double omega0;
    double acc0;
    u_int32_t duty;

    for (double t = 0; t <= (*servo_kinestate0).tf; t += (*servo_kinestate0).timestep)
    {
        five_data(*servo_kinestate0, &theta0, &omega0, &acc0, t);
        duty = convert_servo_angle_to_duty(theta0);
        // vTaskDelay(100/portTICK_RATE_MS);
        ledc_set_duty(servo_channe0.speed_mode, servo_channe0.channel, duty);
        ledc_update_duty(servo_channe0.speed_mode, servo_channe0.channel);

        five_data(*servo_kinestate1, &theta0, &omega0, &acc0, t);
        duty = convert_servo_angle_to_duty(theta0);
        ledc_set_duty(servo_channe1.speed_mode, servo_channe1.channel, duty);
        ledc_update_duty(servo_channe1.speed_mode, servo_channe1.channel);

        five_data(*servo_kinestate2, &theta0, &omega0, &acc0, t);
        duty = convert_servo_angle_to_duty(theta0);
        ledc_set_duty(servo_channe2.speed_mode, servo_channe2.channel, duty);
        ledc_update_duty(servo_channe2.speed_mode, servo_channe2.channel);

        five_data(*servo_kinestate3, &theta0, &omega0, &acc0, t);
        duty = convert_servo_angle_to_duty(theta0);
        ledc_set_duty(servo_channe3.speed_mode, servo_channe3.channel, duty);
        ledc_update_duty(servo_channe3.speed_mode, servo_channe3.channel);
        // ets_delay_us(1000000 * (*servo_kinestate0).timestep);
        if (servo_kinestate1->tf>=0.1)
        {
            // printf("wait\n");
            vTaskDelay(10 / portTICK_RATE_MS);
        }
        vTaskDelay(1 / portTICK_RATE_MS);
    }

    
    (*servo_kinestate0).thetai = (*servo_kinestate0).thetaf;
    (*servo_kinestate0).omegai = (*servo_kinestate0).omegaf;
    (*servo_kinestate0).acci = (*servo_kinestate0).accf;
    
    (*servo_kinestate1).thetai = (*servo_kinestate1).thetaf;
    (*servo_kinestate1).omegai = (*servo_kinestate1).omegaf;
    (*servo_kinestate1).acci = (*servo_kinestate1).accf;

    (*servo_kinestate2).thetai = (*servo_kinestate2).thetaf;
    (*servo_kinestate2).omegai = (*servo_kinestate2).omegaf;
    (*servo_kinestate2).acci = (*servo_kinestate2).accf;

    (*servo_kinestate3).thetai = (*servo_kinestate3).thetaf;
    (*servo_kinestate3).omegai = (*servo_kinestate3).omegaf;
    (*servo_kinestate3).acci = (*servo_kinestate3).accf;
}

void testservo(ledc_channel_config_t servo_channel, ledc_channel_config_t servo_channe2)
{
    uint16_t duty; // from 0 to 8*1024-1
    for (int i = 0; i <= 200; i += 10)
    {
        duty = convert_servo_angle_to_duty(i);
        // printf("Servo is on %d degree\r\n",i);
        ledc_set_duty(servo_channel.speed_mode, servo_channel.channel, duty);
        ledc_update_duty(servo_channel.speed_mode, servo_channel.channel);
        ledc_set_duty(servo_channe2.speed_mode, servo_channe2.channel, duty);
        ledc_update_duty(servo_channe2.speed_mode, servo_channe2.channel);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    for (int i = 200; i >= 0; i -= 10)
    {
        duty = convert_servo_angle_to_duty(i);
        // printf("Servo is on %d degree\r\n",i);
        ledc_set_duty(servo_channel.speed_mode, servo_channel.channel, duty);
        ledc_update_duty(servo_channel.speed_mode, servo_channel.channel);
        ledc_set_duty(servo_channe2.speed_mode, servo_channe2.channel, duty);
        ledc_update_duty(servo_channe2.speed_mode, servo_channe2.channel);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        u_int32_t di = ledc_get_duty(servo_channel.speed_mode, servo_channel.channel);
        // printf("get%d:%d\n", di, duty);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}




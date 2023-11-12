
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

void five_data(struct Servo_kinestate servo_kinestate0,double* theta0,double* omega0,double* acc0,double t)
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

void update_Servo_kinestate(struct Servo_kinestate servo_kinestate0)
{
    servo_kinestate0.thetai=servo_kinestate0.thetaf;
    servo_kinestate0.omegai=servo_kinestate0.omegaf;
    servo_kinestate0.acci=servo_kinestate0.accf;
}

void Servo_run(ledc_channel_config_t  servo_channe0,ledc_channel_config_t  servo_channe1,
ledc_channel_config_t  servo_channe2,ledc_channel_config_t  servo_channe3,
struct Servo_kinestate* servo_kinestate0, struct Servo_kinestate* servo_kinestate1,
struct Servo_kinestate* servo_kinestate2, struct Servo_kinestate* servo_kinestate3)
{
    double theta0;
    double omega0 ;
    double acc0 ;
    u_int32_t duty;
    //printf("0thetai:%d,thetaf:%d\n",servo_kinestate0.thetai,servo_kinestate0.thetaf);
    
    for (double t = 0; t <= (*servo_kinestate0).tf; t += (*servo_kinestate0).timestep)
        { 
            five_data(*servo_kinestate0,&theta0,&omega0,&acc0,t);
            duty = convert_servo_angle_to_duty(theta0);
            // vTaskDelay(100/portTICK_RATE_MS);
            ledc_set_duty(servo_channe0.speed_mode, servo_channe0.channel, duty);
            ledc_update_duty(servo_channe0.speed_mode, servo_channe0.channel);
            
            five_data(*servo_kinestate1,&theta0,&omega0,&acc0,t);
            duty = convert_servo_angle_to_duty(theta0);
            ledc_set_duty(servo_channe1.speed_mode, servo_channe1.channel, duty);
            ledc_update_duty(servo_channe1.speed_mode, servo_channe1.channel);
            
            five_data(*servo_kinestate2,&theta0,&omega0,&acc0,t);
            duty = convert_servo_angle_to_duty(theta0);
            ledc_set_duty(servo_channe2.speed_mode, servo_channe2.channel, duty);
            ledc_update_duty(servo_channe2.speed_mode, servo_channe2.channel);

            five_data(*servo_kinestate3,&theta0,&omega0,&acc0,t);
            duty = convert_servo_angle_to_duty(theta0);
            ledc_set_duty(servo_channe3.speed_mode, servo_channe3.channel, duty);
            ledc_update_duty(servo_channe3.speed_mode, servo_channe3.channel);
            // ets_delay_us(1000000 * (*servo_kinestate0).timestep);
            vTaskDelay(5/portTICK_RATE_MS);
        }
        
    // printf("0thetai:%lf,thetaf:%lf\n",servo_kinestate0.thetai,servo_kinestate0.thetaf);
    // update_Servo_kinestate(servo_kinestate0);
    // printf("1thetai:%lf,thetaf:%lf\n",(*servo_kinestate0).thetai,(*servo_kinestate0).thetaf);
    (*servo_kinestate0).thetai=(*servo_kinestate0).thetaf;
    (*servo_kinestate0).omegai=(*servo_kinestate0).omegaf;
    (*servo_kinestate0).acci=(*servo_kinestate0).accf;
    // printf("2thetai:%lf,thetaf:%lf\n",(*servo_kinestate0).thetai,(*servo_kinestate0).thetaf);
    (*servo_kinestate1).thetai=(*servo_kinestate1).thetaf;
    (*servo_kinestate1).omegai=(*servo_kinestate1).omegaf;
    (*servo_kinestate1).acci=(*servo_kinestate1).accf;

    (*servo_kinestate2).thetai=(*servo_kinestate2).thetaf;
    (*servo_kinestate2).omegai=(*servo_kinestate2).omegaf;
    (*servo_kinestate2).acci=(*servo_kinestate2).accf;

    (*servo_kinestate3).thetai=(*servo_kinestate3).thetaf;
    (*servo_kinestate3).omegai=(*servo_kinestate3).omegaf;
    (*servo_kinestate3).acci=(*servo_kinestate3).accf;
    // vTaskDelay(10000/portTICK_RATE_MS);
    // update_Servo_kinestate(servo_kinestate1);
    // update_Servo_kinestate(servo_kinestate2);
    // update_Servo_kinestate(servo_kinestate3);
    // printf("kjasgjhadfgjagfjkas\n");
    // // servo0
    // double theta0 = 0;
    // double omega0 = 0;
    // double acc0 = 0;
    // double t0 = 0;
    // // servo1
    // double theta0_1 = 0;
    // double omega0_1 = 0;
    // double acc0_1 = 0;
    // double t0_1 = 0;
    // //
    // for (double t = 0; t <= servo_kinestate0.tf; t += servo_kinestate0.timestep)
    // {
    //     five_data(servo_kinestate0,&theta0,&omega0,&acc0,t);
    //     u_int32_t duty = convert_servo_angle_to_duty(theta0);
    //     // printf("Servo is on %d degree\r\n",i);
    //     printf("**");
    //     // vTaskDelay(1000/portTICK_RATE_MS);
    //     ledc_set_duty(servo_channel.speed_mode, servo_channel.channel, duty);
    //     ledc_update_duty(servo_channel.speed_mode, servo_channel.channel);// servo1
    //     // vTaskDelay(100/portTICK_RATE_MS);
    //     five_data(servo_kinestate1,&theta0_1,&omega0_1,&acc0_1,t);
    //     // u_int32_t duty1 = convert_servo_angle_to_duty(theta0_1);
    //     // // printf("Servo is on %d degree\r\n",i);
    //     // // ledc_set_duty(servo_kinestate1.servo_channel->speed_mode, servo_kinestate1.servo_channel->channel, duty1);
    //     // // ledc_update_duty(servo_kinestate1.servo_channel->speed_mode, servo_kinestate1.servo_channel->channel);
    //     // ledc_set_duty(servo_channel.speed_mode, servo_channel.channel, duty);
    //     // ledc_update_duty(servo_channel.speed_mode, servo_channel.channel);// servo1
        
    //     ets_delay_us(1000000 * servo_kinestate0.timestep);
    //     printf("t:%f\n", t);
    //     t0 = t;
    // }
    // servo_kinestate0.omegai = omega0;
    // servo_kinestate0.acci = acc0;
    // servo_kinestate0.thetai = theta0;

    // servo_kinestate1.omegai = omega0_1;
    // servo_kinestate1.acci = acc0_1;
    // servo_kinestate1.thetai = theta0_1;
    // vTaskDelay(10 / portTICK_RATE_MS);
    
}

void testservo(ledc_channel_config_t servo_channel,ledc_channel_config_t servo_channe2)
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

void servo_control_task_begin(ledc_timer_config_t servo_timer, ledc_channel_config_t servo_channel, ledc_channel_config_t servo_channe2)
{

    servo_timer.duty_resolution = LEDC_TIMER_13_BIT; // resolution of PWM duty
    servo_timer.freq_hz = SERVO_FREQUENCY;           // frequency of PWM signal
    servo_timer.speed_mode = LEDC_LOW_SPEED_MODE;    // timer mode
    servo_timer.timer_num = LEDC_TIMER_1;            // timer index
    servo_timer.clk_cfg = LEDC_AUTO_CLK;             // Auto select the source clock


    ledc_timer_config(&servo_timer);
    
    servo_channel.channel = 1;
    servo_channel.duty = 0;
    servo_channel.gpio_num = 8;
    servo_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    servo_channel.hpoint = 0;
    servo_channel.timer_sel = LEDC_TIMER_1;
    servo_channel.flags.output_invert = 0;
    // servo_channel.intr_type=LEDC_INTR_DISABLE;
    
    ledc_channel_config(&servo_channel);

    servo_channe2.channel = 2;
    servo_channe2.duty = 0;
    servo_channe2.gpio_num = 9;
    servo_channe2.speed_mode = LEDC_LOW_SPEED_MODE;
    servo_channe2.hpoint = 0;
    servo_channe2.timer_sel = LEDC_TIMER_1;
    servo_channe2.flags.output_invert = 0;
    // servo_channe2.intr_type=LEDC_INTR_DISABLE;
    ledc_channel_config(&servo_channe2);

    // testservo(servo_channel);
    // testservo(servo_channe2);
    // testservo(servo_channe3);
    // testservo(servo_channe4);
}

// void servo_control_task()
// {
//     ledc_timer_config_t servo_timer = {
//         .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
//         .freq_hz = SERVO_FREQUENCY,           // frequency of PWM signal
//         .speed_mode = LEDC_LOW_SPEED_MODE,    // timer mode
//         .timer_num = LEDC_TIMER_1,            // timer index
//         .clk_cfg = LEDC_AUTO_CLK,             // Auto select the source clock
//     };
//     ledc_timer_config(&servo_timer);

//     ledc_channel_config_t servo_channel = {
//         .channel = 1,
//         .duty = 0,
//         .gpio_num = 8,
//         .speed_mode = LEDC_LOW_SPEED_MODE,
//         .hpoint = 0,
//         .timer_sel = LEDC_TIMER_1,
//         .flags.output_invert = 0};
//     ledc_channel_config(&servo_channel);
//     ledc_channel_config_t servo_channe2 = {
//         .channel = 2,
//         .duty = 0,
//         .gpio_num = 9,
//         .speed_mode = LEDC_LOW_SPEED_MODE,
//         .hpoint = 0,
//         .timer_sel = LEDC_TIMER_1,
//         .flags.output_invert = 0};
//     ledc_channel_config(&servo_channe2);

//     testservo(servo_channel);
//     testservo(servo_channe2);

//     struct Servo_kinestate a =
//         {
//             .thetai = 0,
//             .thetaf = 180,
//             .omegai = 0,
//             .omegaf = 0,
//             .acci = 5,
//             .accf = 15,
//             .timestep = 0.0009,
//             .servo_channel = &servo_channel,
//             .tf = 0.8};

//     printf("%f,%f,%f\n", a.omegai, a.acci, a.thetai);

//     struct Servo_kinestate a2 =
//         {
//             .thetai = 0,
//             .thetaf = 45,
//             .omegai = 0,
//             .omegaf = 0,
//             .acci = 5,
//             .accf = 15,
//             .timestep = 0.0009,
//             .servo_channel = &servo_channe2,
//             .tf = 0.8};
//     // Servo_run(servo_channel,a, a2);
//     printf("%f,%f,%f\n", a2.omegai, a2.acci, a2.thetai);

//     do
//     {
//         double timestep = 0.0009;
//         double a0;
//         double a1;
//         double a2;
//         double a3;
//         double a4;
//         double a5;
//         double omegaf;
//         double omegai;
//         double acci;
//         double accf;
//         double tf = 0.8;
//         for (double t = 0; t <= tf; t += timestep)
//         { // goes from 0 degrees to 180 degrees
//             // a0 = 70;a1 = 0;a2 = 30 ;a3 = -10;
//             double thetai = 0;
//             double thetaf = 180;
//             double omegai = 0;
//             double omegaf = 0;
//             double acci = 5;
//             double accf = 15;
//             a0 = thetai;
//             a1 = omegai;
//             a2 = acci / 2;
//             a3 = (20 * thetaf - 20 * thetai - (8 * omegaf + 12 * omegai) * tf - (3 * acci - accf) * (pow(tf, 2))) / (2 * (pow(tf, 3)));
//             a4 = (30 * thetai - 30 * thetaf + (14 * omegaf + 16 * omegai) * tf + (3 * acci - 2 * accf) * (pow(tf, 2))) / (2 * (pow(tf, 4)));
//             a5 = (12 * thetaf - 12 * thetai - (6 * omegaf + 6 * omegai) * tf - (acci - accf) * (pow(tf, 2))) / (2 * (pow(tf, 5)));
//             double theta0 = a0 + a1 * t + a2 * (pow(t, 2)) + a3 * (pow(t, 3)) + a4 * (pow(t, 4)) + a5 * (pow(t, 5));
//             double omega0 = a1 + 2 * a2 * t + 3 * a3 * (pow(t, 2)) + 4 * a4 * (pow(t, 3)) + 5 * a5 * (pow(t, 4));
//             double acc0 = 2 * a2 + 6 * a3 * t + 12 * a4 * (pow(t, 2)) + 20 * a5 * (pow(t, 3));

//             u_int32_t duty = convert_servo_angle_to_duty(theta0);
//             // printf("Servo is on %d degree\r\n",i);
//             ledc_set_duty(servo_channel.speed_mode, servo_channel.channel, duty);
//             ledc_update_duty(servo_channel.speed_mode, servo_channel.channel);
//             // vTaskDelay(50 / portTICK_PERIOD_MS);
//             //  tell servo to go to position in variable 'pos'
//             //  Serial.print(theta0);
//             //  Serial.println(t);
//             //  waits 15 ms for the servo to reach the position
//             ets_delay_us(1000000 * timestep);
//         }
//         ets_delay_us(500000);
//         vTaskDelay(10 / portTICK_RATE_MS);
//         for (double t = 0; t <= tf; t += timestep)
//         { // goes from 0 degrees to 180 degrees
//             // a0 = 70;a1 = 0;a2 = 30 ;a3 = -10;
//             double thetai = 180;
//             double thetaf = 0;
//             double omegai = 50;
//             double omegaf = 40;
//             double acci = 15;
//             double accf = 5;
//             a0 = thetai;
//             a1 = omegai;
//             a2 = acci / 2;
//             a3 = (20 * thetaf - 20 * thetai - (8 * omegaf + 12 * omegai) * tf - (3 * acci - accf) * (pow(tf, 2))) / (2 * (pow(tf, 3)));
//             a4 = (30 * thetai - 30 * thetaf + (14 * omegaf + 16 * omegai) * tf + (3 * acci - 2 * accf) * (pow(tf, 2))) / (2 * (pow(tf, 4)));
//             a5 = (12 * thetaf - 12 * thetai - (6 * omegaf + 6 * omegai) * tf - (acci - accf) * (pow(tf, 2))) / (2 * (pow(tf, 5)));
//             double theta0 = a0 + a1 * t + a2 * (pow(t, 2)) + a3 * (pow(t, 3)) + a4 * (pow(t, 4)) + a5 * (pow(t, 5));
//             double omega0 = a1 + 2 * a2 * t + 3 * a3 * (pow(t, 2)) + 4 * a4 * (pow(t, 3)) + 5 * a5 * (pow(t, 4));
//             double acc0 = 2 * a2 + 6 * a3 * t + 12 * a4 * (pow(t, 2)) + 20 * a5 * (pow(t, 3));

//             u_int32_t duty = convert_servo_angle_to_duty(theta0);
//             // printf("Servo is on %d degree\r\n",duty);
//             ledc_set_duty(servo_channel.speed_mode, servo_channel.channel, duty);
//             ledc_update_duty(servo_channel.speed_mode, servo_channel.channel);
//             // vTaskDelay(50 / portTICK_PERIOD_MS);// tell servo to go to position in variable 'pos'
//             //  Serial.print(theta0);
//             //  Serial.println(t);
//             //  waits 15 ms for the servo to reach the position
//         }
//         printf("hhhhhhh\n");
//         vTaskDelay(1000/portTICK_RATE_MS);
//     } while (1);
// }

void servo_control_task()
{
    ledc_timer_config_t servo_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = SERVO_FREQUENCY,           // frequency of PWM signal
        .speed_mode = LEDC_LOW_SPEED_MODE,    // timer mode
        .timer_num = LEDC_TIMER_1,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,             // Auto select the source clock
    };
    ledc_timer_config(&servo_timer);

    ledc_channel_config_t servo_channel = {
        .channel = 1,
        .duty = 0,
        .gpio_num = 8,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_1,
        .flags.output_invert = 0};
    ledc_channel_config(&servo_channel);
    ledc_channel_config_t servo_channe2 = {
        .channel = 2,
        .duty = 0,
        .gpio_num = 9,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_1,
        .flags.output_invert = 0};
    ledc_channel_config(&servo_channe2);

    // testservo(servo_channel);
    // testservo(servo_channe2);

    struct Servo_kinestate a =
        {
            .thetai = 0,
            .thetaf = 180,
            .omegai = 0,
            .omegaf = 0,
            .acci = 5,
            .accf = 15,
            .timestep = 0.0009,
            .servo_channel = &servo_channel,
            .tf = 0.8};

    // printf("%f,%f,%f\n", a.omegai, a.acci, a.thetai);

    struct Servo_kinestate a2 =
        {
            .thetai = 0,
            .thetaf = 45,
            .omegai = 0,
            .omegaf = 0,
            .acci = 5,
            .accf = 15,
            .timestep = 0.0009,
            .servo_channel = &servo_channe2,
            .tf = 0.8};
    // Servo_run(servo_channel,servo_channe2,a, a2);
    // printf("%f,%f,%f\n", a2.omegai, a2.acci, a2.thetai);

    // do
    // {
    //     double timestep = 0.0009;
    //     double a0;
    //     double a1;
    //     double a2;
    //     double a3;
    //     double a4;
    //     double a5;
    //     double omegaf;
    //     double omegai;
    //     double acci;
    //     double accf;
    //     double tf = 0.8;
    //     for (double t = 0; t <= tf; t += timestep)
    //     { // goes from 0 degrees to 180 degrees
    //         // a0 = 70;a1 = 0;a2 = 30 ;a3 = -10;
    //         double thetai = 0;
    //         double thetaf = 180;
    //         double omegai = 0;
    //         double omegaf = 0;
    //         double acci = 5;
    //         double accf = 15;
    //         a0 = thetai;
    //         a1 = omegai;
    //         a2 = acci / 2;
    //         a3 = (20 * thetaf - 20 * thetai - (8 * omegaf + 12 * omegai) * tf - (3 * acci - accf) * (pow(tf, 2))) / (2 * (pow(tf, 3)));
    //         a4 = (30 * thetai - 30 * thetaf + (14 * omegaf + 16 * omegai) * tf + (3 * acci - 2 * accf) * (pow(tf, 2))) / (2 * (pow(tf, 4)));
    //         a5 = (12 * thetaf - 12 * thetai - (6 * omegaf + 6 * omegai) * tf - (acci - accf) * (pow(tf, 2))) / (2 * (pow(tf, 5)));
    //         double theta0 = a0 + a1 * t + a2 * (pow(t, 2)) + a3 * (pow(t, 3)) + a4 * (pow(t, 4)) + a5 * (pow(t, 5));
    //         double omega0 = a1 + 2 * a2 * t + 3 * a3 * (pow(t, 2)) + 4 * a4 * (pow(t, 3)) + 5 * a5 * (pow(t, 4));
    //         double acc0 = 2 * a2 + 6 * a3 * t + 12 * a4 * (pow(t, 2)) + 20 * a5 * (pow(t, 3));

    //         u_int32_t duty = convert_servo_angle_to_duty(theta0);
    //         // printf("Servo is on %d degree\r\n",i);
    //         ledc_set_duty(servo_channel.speed_mode, servo_channel.channel, duty);
    //         ledc_update_duty(servo_channel.speed_mode, servo_channel.channel);
    //         // vTaskDelay(50 / portTICK_PERIOD_MS);
    //         //  tell servo to go to position in variable 'pos'
    //         //  Serial.print(theta0);
    //         //  Serial.println(t);
    //         //  waits 15 ms for the servo to reach the position
    //         ets_delay_us(1000000 * timestep);
    //     }
    //     ets_delay_us(500000);
    //     vTaskDelay(10 / portTICK_RATE_MS);
    //     for (double t = 0; t <= tf; t += timestep)
    //     { // goes from 0 degrees to 180 degrees
    //         // a0 = 70;a1 = 0;a2 = 30 ;a3 = -10;
    //         double thetai = 180;
    //         double thetaf = 0;
    //         double omegai = 50;
    //         double omegaf = 40;
    //         double acci = 15;
    //         double accf = 5;
    //         a0 = thetai;
    //         a1 = omegai;
    //         a2 = acci / 2;
    //         a3 = (20 * thetaf - 20 * thetai - (8 * omegaf + 12 * omegai) * tf - (3 * acci - accf) * (pow(tf, 2))) / (2 * (pow(tf, 3)));
    //         a4 = (30 * thetai - 30 * thetaf + (14 * omegaf + 16 * omegai) * tf + (3 * acci - 2 * accf) * (pow(tf, 2))) / (2 * (pow(tf, 4)));
    //         a5 = (12 * thetaf - 12 * thetai - (6 * omegaf + 6 * omegai) * tf - (acci - accf) * (pow(tf, 2))) / (2 * (pow(tf, 5)));
    //         double theta0 = a0 + a1 * t + a2 * (pow(t, 2)) + a3 * (pow(t, 3)) + a4 * (pow(t, 4)) + a5 * (pow(t, 5));
    //         double omega0 = a1 + 2 * a2 * t + 3 * a3 * (pow(t, 2)) + 4 * a4 * (pow(t, 3)) + 5 * a5 * (pow(t, 4));
    //         double acc0 = 2 * a2 + 6 * a3 * t + 12 * a4 * (pow(t, 2)) + 20 * a5 * (pow(t, 3));

    //         u_int32_t duty = convert_servo_angle_to_duty(theta0);
    //         // printf("Servo is on %d degree\r\n",duty);
    //         ledc_set_duty(servo_channel.speed_mode, servo_channel.channel, duty);
    //         ledc_update_duty(servo_channel.speed_mode, servo_channel.channel);
    //         // vTaskDelay(50 / portTICK_PERIOD_MS);// tell servo to go to position in variable 'pos'
    //         //  Serial.print(theta0);
    //         //  Serial.println(t);
    //         //  waits 15 ms for the servo to reach the position
    //     }
    //     printf("hhhhhhh\n");
    //     vTaskDelay(1000/portTICK_RATE_MS);
    // } while (1);
}
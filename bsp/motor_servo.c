/**
 * *****************************************************************************
 * @file        motor_servo.c
 * @brief       R车模电机和舵机驱动以及相应的编码器
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-26
 * @copyright   cita
 * *****************************************************************************
 */

/*----------------------------------include-----------------------------------*/
#include "zf_common_headfile.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/
double motor_duty                = 0;
bool   dir                       = true;
int16  encoder_data_1            = 0;
uint32 cumulative_encoder_data_1 = 0;
// uint8 motor_encoder_pit_state = 0;
double servo_value = SERVO_MOTOR_MID;
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/

void motor_encoder_proc()
{
    // motor_encoder_pit_state = 0;
    encoder_data_1 = encoder_get_count(ENCODER1_TIM) / 5;
    cumulative_encoder_data_1 += encoder_data_1;
    encoder_clear_count(ENCODER1_TIM);   // 清空编码器计数
                                         // drv8701_motor_test();
    // printf("encoder_data_1: %d\r\n", encoder_data_1);
    //  printf("cumulative_encoder_data_1: %d\r\n", cumulative_encoder_data_1);
#if MOTOR_ENCODER_WIFI_SEND_FLAG
    // wifi_spi_send();
#endif
    disable_handler("motor_encoder");
}
void drv8701_motor_init()   // 电机和编码器初始化
{
    motor_duty = 0;
    gpio_init(DIR1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PWM1, 17000, 0);
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);   // 初始化编码器模块与引脚 正交解码编码器模式
}

void drv8701_motor_set(double duty)   // 电机驱动
{
    motor_duty = duty;
    if (motor_duty > MAX_DUTY)
    {
        motor_duty = MAX_DUTY;
    }
    else if (motor_duty < -MAX_DUTY)
    {
        motor_duty = -MAX_DUTY;
    }
    if (motor_duty >= 0)
    {
        gpio_set_level(DIR1, GPIO_LOW);                            // DIR输出高电平
        pwm_set_duty(PWM1, (motor_duty) * (PWM_DUTY_MAX / 100));   // 计算占空比
    }
    else
    {
        gpio_set_level(DIR1, GPIO_HIGH);                            // DIR输出低电平
        pwm_set_duty(PWM1, (-motor_duty) * (PWM_DUTY_MAX / 100));   // 计算占空比
    }
}
void bldc_motor_init()   // 电机和编码器初始化
{
    motor_duty = 0;
    pwm_init(PWM_CH1, 1000, 0);                                    // PWM 通道1 初始化频率1KHz 占空比初始为0
    gpio_init(DIR_CH1, GPO, GPIO_HIGH, GPO_PUSH_PULL);             // 初始化电机方向输出引脚
    encoder_dir_init(ENCODER1_TIM, ENCODER1_PLUS, ENCODER1_DIR);   // 初始化编码器采值引脚及定时器
    PidInit(&PID_Init);
}

void bldc_motor_set(double duty)   // 电机驱动
{
    motor_duty = duty;
    if (motor_duty > MAX_DUTY)
    {
        motor_duty = MAX_DUTY;
    }
    else if (motor_duty < -MAX_DUTY)
    {
        motor_duty = -MAX_DUTY;
    }
    if (motor_duty >= 0)
    {
        gpio_set_level(DIR_CH1, GPIO_HIGH);                           // DIR输出高电平
        pwm_set_duty(PWM_CH1, (motor_duty) * (PWM_DUTY_MAX / 100));   // 计算占空比
    }
    else
    {
        gpio_set_level(DIR_CH1, GPIO_LOW);                             // DIR输出低电平
        pwm_set_duty(PWM_CH1, (-motor_duty) * (PWM_DUTY_MAX / 100));   // 计算占空比
    }
}

void bldc_motor_speed_ctrl(double speed)
{
    double target_speed  = speed;                          // 目标速度
    double current_speed = encoder_data_1;                 // 当前速度(编码器数据)
    double speed_gap     = target_speed - current_speed;   // 速度差距

    // 使用PID控制器计算输出
    double out = PidIncCtrl(&PID_MOTOR, speed_gap);

    // 控制台输出调试信息
    // printf("%.2f,%.2f,%.2f,%.2f\n", target_speed, current_speed, speed_gap, out);



    // 限制输出范围
    if (out > MAX_DUTY)
    {
        out = MAX_DUTY;
    }
    if (out < -MAX_DUTY)
    {
        out = -MAX_DUTY;
    }
    // 发送数据到上位机示波器
#if PID_WIFI_SEND_FLAG
    // 打包数据到示波器格式
    seekfree_assistant_oscilloscope_data.data[0]     = target_speed;    // 通道1：目标速度
    seekfree_assistant_oscilloscope_data.data[1]     = current_speed;   // 通道2：当前速度
    seekfree_assistant_oscilloscope_data.data[2]     = speed_gap;       // 通道3：速度差
    seekfree_assistant_oscilloscope_data.data[3]     = out;             // 通道4：PID输出
    seekfree_assistant_oscilloscope_data.channel_num = 4;               // 使用4个通道

    // 发送数据到上位机并处理可能的返回数据
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
    seekfree_assistant_data_analysis();
#endif
    // 调用现有函数设置电机输出
    bldc_motor_set(out);
}
void servo_init()   // 舵机初始化
{
    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY(SERVO_MOTOR_MID));
    PidInit(&PID_Init);
}

void servo_set(double angle)   // 舵机驱动
{
    servo_value = angle;
    if (servo_value < SERVO_MOTOR_L_MAX)
    {
        servo_value = SERVO_MOTOR_L_MAX;
    }
    if (servo_value > SERVO_MOTOR_R_MAX)
    {
        servo_value = SERVO_MOTOR_R_MAX;
    }
    // 为了反转舵机角度方向，需要2 * SERVO_MOTOR_MID - angle才是真实angle
    pwm_set_duty(SERVO_MOTOR_PWM, (uint32)SERVO_MOTOR_DUTY(2 * SERVO_MOTOR_MID - servo_value));
    // printf("servo_value: %f\r\n", servo_value);
}

/*------------------------------------test------------------------------------*/
void drv8701_motor_test()   // 电机测试
{
    key_scan();
    if (key1_flag)
    {
        key1_flag = 0;
        motor_duty += 10;
    }
    if (key2_flag)
    {
        key2_flag = 0;
        motor_duty -= 10;
    }
    if (key3_flag)
    {
        key3_flag = 0;
        motor_duty += 1;
    }
    if (key4_flag)
    {
        key4_flag = 0;
        motor_duty -= 1;
    }
    // 读取并加载上位机发送过来的参数
    // motor_duty = seekfree_assistant_parameter[4];
    drv8701_motor_set(motor_duty);
    ips_show_int(100, 16 * 3, motor_duty, 5);
    ips_show_int(100, 16 * 4, encoder_data_1, 9);
    ips_show_int(100, 16 * 5, cumulative_encoder_data_1, 9);
}

void servo_test()   // 舵机测试
{

    key_scan();
    if (key1_flag)
    {
        key1_flag = 0;
        servo_value += 10;
    }
    if (key2_flag)
    {
        key2_flag = 0;
        servo_value -= 10;
    }
    if (key3_flag)
    {
        key3_flag = 0;
        servo_value += 1;
        // angle = SERVO_MOTOR_L_MAX; // 左打死
    }
    if (key4_flag)
    {
        key4_flag = 0;
        servo_value -= 1;
        // angle = SERVO_MOTOR_R_MAX; // 右打死
    }
    servo_set(servo_value);
    ips_show_int(100, 16 * 3, servo_value, 5);
}

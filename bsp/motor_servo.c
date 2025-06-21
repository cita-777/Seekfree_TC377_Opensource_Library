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
volatile uint8_t encoder_updated = 0;
double           servo_value     = SERVO_MOTOR_MID;
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/

void motor_encoder_proc()
{
    // motor_encoder_pit_state = 0;
    encoder_data_1 = -encoder_get_count(ENCODER_1);
    cumulative_encoder_data_1 += encoder_data_1;
    encoder_clear_count(ENCODER_1);   // 清空编码器计数
    encoder_updated = 1;              // 设置标志，表示编码器已更新
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

    // 添加调试输出
    // printf("Motor control: duty=%.2f, limited_duty=%.2f\n", duty, motor_duty);

    if (motor_duty >= 0)   // 正转
    {
        gpio_set_level(DIR1, GPIO_LOW);                            // 尝试改为HIGH
        pwm_set_duty(PWM1, (motor_duty) * (PWM_DUTY_MAX / 100));   // 计算占空比
        // printf("Forward: DIR=HIGH, PWM=%.0f\n", (motor_duty) * (PWM_DUTY_MAX / 100));
    }
    else   // 反转
    {
        gpio_set_level(DIR1, GPIO_HIGH);                            // 尝试改为LOW
        pwm_set_duty(PWM1, (-motor_duty) * (PWM_DUTY_MAX / 100));   // 计算占空比
        // printf("Reverse: DIR=LOW, PWM=%.0f\n", (-motor_duty) * (PWM_DUTY_MAX / 100));
    }
}
void drv8701_motor_speed_ctrl(double speed)
{
    // 如果编码器没有更新，直接返回
    if (!encoder_updated)
    {
        return;
    }

    double target_speed  = speed;
    double current_speed = encoder_data_1 / 31.85;   // 假设编码器数据是整数，转换为浮点数表示速度
    double speed_gap     = target_speed - current_speed;

    // 零速度处理
    if (fabs(target_speed) < 0.1)
    {
        drv8701_motor_set(0);
        PID_MOTOR.out   = 0;   // 重置PID输出
        encoder_updated = 0;
        return;
    }

    // 设置PID参数
    PID_MOTOR.kp = 2.8;    // 比例系数
    PID_MOTOR.ki = 0.02;   // 积分系数
    PID_MOTOR.kd = 0.3;    // 微分系数

    // 使用PID控制器计算输出
    double out = PidIncCtrl(&PID_MOTOR, speed_gap);

    // 限制输出范围
    if (out > MAX_DUTY)
    {
        out = MAX_DUTY;
    }
    if (out < -MAX_DUTY)
    {
        out = -MAX_DUTY;
    }

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

    encoder_updated = 0;
    drv8701_motor_set(out);
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
    // 记录上一次的目标速度，用于检测速度变化
    static double last_target_speed = 0;

    // 如果编码器没有更新，直接返回
    if (!encoder_updated)
    {
        return;
    }

    double target_speed  = speed;
    double current_speed = encoder_data_1;
    double speed_gap     = target_speed - current_speed;

    // 快速停车处理：如果从非零速度变为零速度
    if (fabs(last_target_speed) > 0.5 && fabs(target_speed) < 0.1)
    {
        // 重置PID状态
        PID_MOTOR.out             = 0;
        PID_MOTOR.last_error      = 0;
        PID_MOTOR.last_derivative = 0;

        // 短时反向制动（通过反向占空比快速减速）
        double brake_duty = (last_target_speed > 0) ? -8.0 : 8.0;
        bldc_motor_set(brake_duty);
        // systick_delay_ms(50);   // 短时间反向制动

        // 完全停止
        bldc_motor_set(0);
        encoder_updated   = 0;
        last_target_speed = target_speed;
        return;
    }

    // 普通零速度处理
    if (fabs(target_speed) < 0.1)
    {
        // 如果速度已经很低，直接停止
        if (fabs(current_speed) < 2.0)
        {
            bldc_motor_set(0);
            PID_MOTOR.out     = 0;   // 重置PID输出
            encoder_updated   = 0;
            last_target_speed = target_speed;
            return;
        }
        // 否则使用增强制动参数
        PID_MOTOR.kp = 1.5;   // 增大比例系数
        PID_MOTOR.ki = 0;     // 禁用积分项
        PID_MOTOR.kd = 0.3;   // 增大微分系数
    }
    else
    {
        // 提高响应速度的参数
        PID_MOTOR.kp = 2.5;    // 大幅提高比例系数
        PID_MOTOR.ki = 0.02;   // 略微提高积分系数
        PID_MOTOR.kd = 0.2;    // 保持微分系数
    }

    // 使用PID控制器计算输出
    double out = PidIncCtrl(&PID_MOTOR, speed_gap);

    // 简化的静摩擦补偿（启动助力）
    double static_friction = 8.0;   // 提高静摩擦补偿值
    if (fabs(out) < static_friction && fabs(speed_gap) > 2.0)
    {
        out = (out >= 0) ? static_friction : -static_friction;
    }

    // 限制输出范围
    if (out > MAX_DUTY)
    {
        out = MAX_DUTY;
    }
    if (out < -MAX_DUTY)
    {
        out = -MAX_DUTY;
    }

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

    encoder_updated = 0;
    bldc_motor_set(out);
    last_target_speed = target_speed;
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
/**
 * @brief 舵机角度PD闭环控制函数
 * @param target_angle 目标角度(度)
 * @details 通过IMU角度反馈，使用PD控制器来控制车辆转向到指定角度
 *          当车辆偏离目标角度时，PD控制器会计算修正值
 *          使舵机转向，将车辆调整到正确角度
 */
void servo_set_pd(float target_angle)
{
    // 获取当前IMU偏航角
    float current_angle = g_imu_angle.yaw;

    // 计算误差：目标角度 - 当前角度
    float error = target_angle - current_angle;
    // 角度差归一化处理，解决180度跳变问题
    while (error > 180.0f)
    {
        error -= 360.0f;
    }
    while (error < -180.0f)
    {
        error += 360.0f;
    }
    // 使用PD控制器计算输出值
    double pd_output = PidLocCtrl(&PID_IMU, error);
    if (pd_output > SERVO_MOTOR_R_MAX - SERVO_MOTOR_MID)
    {
        pd_output = SERVO_MOTOR_R_MAX - SERVO_MOTOR_MID;   // 限制最大修正值
    }
    else if (pd_output < SERVO_MOTOR_L_MAX - SERVO_MOTOR_MID)
    {
        pd_output = SERVO_MOTOR_L_MAX - SERVO_MOTOR_MID;   // 限制最小修正值
    }
    // 设置舵机值为中间值加上PD修正值
    servo_set(SERVO_MOTOR_MID - pd_output);

    // 可选：发送数据到上位机示波器显示
#if PD_WIFI_SEND_FLAG
    // 打包数据到示波器格式
    seekfree_assistant_oscilloscope_data.data[0]     = target_angle;    // 通道1：目标角度
    seekfree_assistant_oscilloscope_data.data[1]     = current_angle;   // 通道2：当前IMU角度
    seekfree_assistant_oscilloscope_data.data[2]     = error;           // 通道3：角度误差
    seekfree_assistant_oscilloscope_data.data[3]     = pd_output;       // 通道4：PD输出
    seekfree_assistant_oscilloscope_data.data[4]     = servo_value;     // 通道5：舵机最终设置值
    seekfree_assistant_oscilloscope_data.channel_num = 5;               // 使用5个通道

    // 发送数据到上位机并处理可能的返回数据
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
#endif
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

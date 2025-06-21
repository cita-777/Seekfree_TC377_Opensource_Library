/**
 * *****************************************************************************
 * @file        ips_sbus.c
 * @brief       2寸ips显示屏和sbus遥控器接受模块
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-26
 * @copyright   cita
 * *****************************************************************************
 */

/*----------------------------------include-----------------------------------*/
#include "zf_common_headfile.h"
/*-----------------------------------macro------------------------------------*/
#define BRAKE_THRESHOLD 5   // 认为已停止的速度阈值
/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/
uint8 point_flag = 0;
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void ips_sbus_init()
{
    ips200_init(IPS200_TYPE);
    uart_receiver_init();
}
void emergency_brake()
{
    extern int16 encoder_data_1;     // 引用编码器变量
    static uint8 stable_count = 0;   // 稳定计数器

    // 打印当前编码器值，便于调试
    printf("Emergency braking: encoder=%d\n", encoder_data_1);

    if (abs(encoder_data_1) < BRAKE_THRESHOLD)
    {
        // 车速已经很低，增加稳定计数
        stable_count++;

        if (stable_count >= 3)
        {
            // 连续几次读数都在阈值内，认为已稳定停止
            bldc_motor_set(0.0);   // 使用0.0而不是0
            stable_count = 0;      // 重置计数器
            printf("Vehicle stopped\n");
        }
        else
        {
            // 施加小的保持力矩防止滑动 - 使用double类型
            bldc_motor_set(encoder_data_1 > 0 ? 3.0 : -3.0);   // 增加制动力并使用double
        }
    }
    else
    {
        // 重置稳定计数器
        stable_count = 0;

        // 根据当前速度方向和大小，应用反向制动力矩
        if (encoder_data_1 > BRAKE_THRESHOLD)
        {
            // 正向运动中，需要反向制动 - 使用double类型
            double brake_power = (double)encoder_data_1 / 5.0;   // 使用double计算

            // 限制最大制动力 - 增大最大制动力值
            if (brake_power > 10.0) brake_power = 10.0;

            bldc_motor_set(brake_power);   // 已经是double类型
            printf("Forward braking: power=%.2f\n", brake_power);
        }
        else if (encoder_data_1 < -BRAKE_THRESHOLD)
        {
            // 反向运动中，需要正向制动 - 使用double类型
            double brake_power = (double)(-encoder_data_1) / 5.0;   // 使用double计算

            // 限制最大制动力 - 增大最大制动力值
            if (brake_power > 10.0) brake_power = 10.0;

            bldc_motor_set(-brake_power);   // 已经是double类型
            printf("Reverse braking: power=%.2f\n", brake_power);
        }
    }
}
void sbus_proc()
{
    // 记录上升沿
    static bool prev_channels_4_5_active = false;

    if (!uart_receiver.finsh_flag) return;

    // 遥控器失控处理
    if (uart_receiver.state != 1)
    {
        drv8701_motor_set(0);
        // bldc_motor_set(0);
        // emergency_brake();
        servo_set(90);
        // zf_log(0,"Remote control lost control.");
        // zf_assert(0);
    }
    else
    {
        // printf("CH1-CH6 data: ");
        // for (int i = 0; i < 6; i++)
        // {
        //     printf("%d ", uart_receiver.channel[i]);   // 串口输出6个通道数据
        // }
        printf("\r\n");
        // 通道5、6上升沿检测
        bool current_active = (uart_receiver.channel[4] == 1792 && uart_receiver.channel[5] == 1792);
        if (current_active && !prev_channels_4_5_active)
        {
            point_flag = 1;
            printf("Point flag is set.\n");
        }
        // printf("prev_channels_4_5_active: %d\n", prev_channels_4_5_active);
        prev_channels_4_5_active = current_active;
        // 打印prev_channels_4_5_active
        // printf("current_active: %d\n", current_active);

        // 通道1，2，3控制逻辑
        if (uart_receiver.channel[3] == 992)
        {
            // servo_set(uart_receiver.channel[0] * 0.025 + 64);

            // 科目1，51000编码器值是3m
            if (cumulative_encoder_data_1 >= 120000)
            {
                servo_set_pd(0.0);
            }
            else if (cumulative_encoder_data_1 > 30000 && cumulative_encoder_data_1 < 120000)
            {
                servo_set_pd(180.0);
            }
            else
            {
                servo_set_pd(0.0);
            }
            // servo_set(90.0);
            //  printf("drv8701_motor_set input: %.2f\n", 123 - uart_receiver.channel[1] * 0.125);
            //   drv8701_motor_set(123 - uart_receiver.channel[1] * 0.125);
            drv8701_motor_speed_ctrl(123 - uart_receiver.channel[1] * 0.125);
            //   bldc_motor_set(123 - uart_receiver.channel[1] * 0.125);
            //    timer_enable();
            //    bldc_motor_speed_ctrl(123 - uart_receiver.channel[1] * 0.125);
            //    timer_disable();
        }
        else
        {
            // 蜂鸣器
            Buzzer_check(200, 50);
            zf_log(0, "channel 3 stop.");
            drv8701_motor_set(0);
            // bldc_motor_set(0);
            //  emergency_brake();
            servo_set(90);
            // printf("channel 3 stop.\r\n");
        }
    }

    uart_receiver.finsh_flag = 0;   // 统一收尾
    disable_handler("sbus");
}

void ips_show_string(uint16 x, uint16 y, const char* str)
{
    ips200_show_string(0, y, "                             ");   // 29*8=237
    ips200_show_string(x, y, str);
}
/*------------------------------------test------------------------------------*/
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
        // }        printf("\r\n");
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
            if (cumulative_encoder_data_1 >= 600000)
            {
                // 使用扩展角度进行精确调头控制
                // 0.0度表示保持当前角度，180.0度表示调头180度，-180.0度表示反方向调头
                servo_set_pd_extended(360.0, true);   // 使用扩展角度模式
            }
            else if (cumulative_encoder_data_1 > 200000 && cumulative_encoder_data_1 < 400000)
            {
                // 进行180度调头，使用扩展角度可以精确控制是向左还是向右调头
                // 如果当前扩展角度是30度，目标210度表示顺时针转180度
                // 如果当前扩展角度是30度，目标-150度表示逆时针转180度
                servo_set_pd_extended(180.0, true);   // 使用扩展角度模式
            }
            else
            {
                servo_set_pd_extended(0.0, true);   // 使用扩展角度模式
            }

            // 科目2，八字s弯
            // 11000是60cm
            // 核心控制参数
            // float encoder_scale = 1.0f;   // 编码器缩放系数
            // float servo_scale   = 1.0f;   // 舵机角度缩放系数

            // // 路径参数
            // float init_straight = 15000;   // 初始直行距离

            // // 去程S弯参数
            // float s_angle_go   = 40.0f;   // 去程S弯角度
            // float s_segment_go = 50000;   // 去程单个S弯长度

            // // 回程S弯参数
            // float s_angle_back   = 40.0f;   // 回程S弯角度（可单独调整）
            // float s_segment_back = 50000;   // 回程单个S弯长度（可单独调整）

            // float pre_turn_straight = 30000;     // 掉头前直行距离
            // float turn_distance     = 50000;     // 掉头阶段距离（1段）
            // float turn_angle        = -180.0f;   // 掉头基准角度（负值=右转）

            // float enc = cumulative_encoder_data_1 * encoder_scale;

            // // 1. 初始直行
            // if (enc < init_straight)
            // {
            //     servo_set_pd_extended(0.0, true);
            // }
            // // 2. 去程S弯（3段）
            // else if (enc < init_straight + 3 * s_segment_go)
            // {
            //     int   phase = (int)((enc - init_straight) / s_segment_go) % 3;
            //     float angle = (phase == 1 ? -s_angle_go : s_angle_go);   // 相位1右转，其他左转
            //     servo_set_pd_extended(angle * servo_scale, true);
            // }
            // // 3. 掉头前直行
            // else if (enc < init_straight + 3 * s_segment_go + pre_turn_straight)
            // {
            //     servo_set_pd_extended(0.0, true);
            // }
            // // 4. 掉头阶段（1段）
            // else if (enc < init_straight + 3 * s_segment_go + pre_turn_straight + turn_distance)
            // {
            //     servo_set_pd_extended(turn_angle * servo_scale, true);   // 固定-180°掉头
            // }
            // // 5. 回程S弯（3段，独立参数控制）
            // else if (enc < init_straight + 3 * s_segment_go + pre_turn_straight + turn_distance + 3 * s_segment_back)
            // {
            //     int phase = (int)((enc - init_straight - 3 * s_segment_go - pre_turn_straight - turn_distance) /
            //                       s_segment_back) %
            //                 3;
            //     float angle = turn_angle + (phase == 1 ? s_angle_back : -s_angle_back);   // 相位1:-140°，其他:-220°
            //     servo_set_pd_extended(angle * servo_scale, true);
            // }
            // // 6. 最终直行（保持-180°）
            // else
            // {
            //     servo_set_pd_extended(turn_angle * servo_scale, true);
            // }

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


/*
 * 扩展角度系统使用示例：
 *
 * 1. 传统模式（向后兼容）：

 *    servo_set_pd(90.0);  // 转到90度，但受限于-180~180范围
 *
 * 2. 扩展角度模式：
 *    servo_set_pd_extended(270.0, true);   // 从0度顺时针转270度
 *    servo_set_pd_extended(-90.0, true);   // 从0度逆时针转90度
 *    servo_set_pd_extended(450.0, true);   // 从0度顺时针转450度（1圈+90度）
 *
 * 3. 精确调头控制：
 *    float current = IMU_GetExtendedYaw();
 *    servo_set_pd_extended(current + 180.0, true);  // 顺时针调头180度
 *    servo_set_pd_extended(current - 180.0, true);  // 逆时针调头180度
 *
 * 4. 角度重置：
 *    IMU_ResetExtendedAngles();  // 重置累积角度
 *
 * 5. 设置初始角度：
 *    IMU_SetExtendedYaw(100.0);  // 设置当前扩展角度为100度
 */


/*------------------------------------test------------------------------------*/
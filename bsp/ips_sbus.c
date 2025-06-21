/**
 * *****************************************************************************
 * @file        ips_sbus.c
 * @brief       2��ips��ʾ����sbusң��������ģ��
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-26
 * @copyright   cita
 * *****************************************************************************
 */

/*----------------------------------include-----------------------------------*/
#include "zf_common_headfile.h"
/*-----------------------------------macro------------------------------------*/
#define BRAKE_THRESHOLD 5   // ��Ϊ��ֹͣ���ٶ���ֵ
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
    extern int16 encoder_data_1;     // ���ñ���������
    static uint8 stable_count = 0;   // �ȶ�������

    // ��ӡ��ǰ������ֵ�����ڵ���
    printf("Emergency braking: encoder=%d\n", encoder_data_1);

    if (abs(encoder_data_1) < BRAKE_THRESHOLD)
    {
        // �����Ѿ��ܵͣ������ȶ�����
        stable_count++;

        if (stable_count >= 3)
        {
            // �������ζ���������ֵ�ڣ���Ϊ���ȶ�ֹͣ
            bldc_motor_set(0.0);   // ʹ��0.0������0
            stable_count = 0;      // ���ü�����
            printf("Vehicle stopped\n");
        }
        else
        {
            // ʩ��С�ı������ط�ֹ���� - ʹ��double����
            bldc_motor_set(encoder_data_1 > 0 ? 3.0 : -3.0);   // �����ƶ�����ʹ��double
        }
    }
    else
    {
        // �����ȶ�������
        stable_count = 0;

        // ���ݵ�ǰ�ٶȷ���ʹ�С��Ӧ�÷����ƶ�����
        if (encoder_data_1 > BRAKE_THRESHOLD)
        {
            // �����˶��У���Ҫ�����ƶ� - ʹ��double����
            double brake_power = (double)encoder_data_1 / 5.0;   // ʹ��double����

            // ��������ƶ��� - ��������ƶ���ֵ
            if (brake_power > 10.0) brake_power = 10.0;

            bldc_motor_set(brake_power);   // �Ѿ���double����
            printf("Forward braking: power=%.2f\n", brake_power);
        }
        else if (encoder_data_1 < -BRAKE_THRESHOLD)
        {
            // �����˶��У���Ҫ�����ƶ� - ʹ��double����
            double brake_power = (double)(-encoder_data_1) / 5.0;   // ʹ��double����

            // ��������ƶ��� - ��������ƶ���ֵ
            if (brake_power > 10.0) brake_power = 10.0;

            bldc_motor_set(-brake_power);   // �Ѿ���double����
            printf("Reverse braking: power=%.2f\n", brake_power);
        }
    }
}
void sbus_proc()
{
    // ��¼������
    static bool prev_channels_4_5_active = false;

    if (!uart_receiver.finsh_flag) return;

    // ң����ʧ�ش���
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
        //     printf("%d ", uart_receiver.channel[i]);   // �������6��ͨ������
        // }
        printf("\r\n");
        // ͨ��5��6�����ؼ��
        bool current_active = (uart_receiver.channel[4] == 1792 && uart_receiver.channel[5] == 1792);
        if (current_active && !prev_channels_4_5_active)
        {
            point_flag = 1;
            printf("Point flag is set.\n");
        }
        // printf("prev_channels_4_5_active: %d\n", prev_channels_4_5_active);
        prev_channels_4_5_active = current_active;
        // ��ӡprev_channels_4_5_active
        // printf("current_active: %d\n", current_active);

        // ͨ��1��2��3�����߼�
        if (uart_receiver.channel[3] == 992)
        {
            // servo_set(uart_receiver.channel[0] * 0.025 + 64);

            // ��Ŀ1��51000������ֵ��3m
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
            // ������
            Buzzer_check(200, 50);
            zf_log(0, "channel 3 stop.");
            drv8701_motor_set(0);
            // bldc_motor_set(0);
            //  emergency_brake();
            servo_set(90);
            // printf("channel 3 stop.\r\n");
        }
    }

    uart_receiver.finsh_flag = 0;   // ͳһ��β
    disable_handler("sbus");
}

void ips_show_string(uint16 x, uint16 y, const char* str)
{
    ips200_show_string(0, y, "                             ");   // 29*8=237
    ips200_show_string(x, y, str);
}
/*------------------------------------test------------------------------------*/
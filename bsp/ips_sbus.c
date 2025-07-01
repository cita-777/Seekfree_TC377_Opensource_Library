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
        // }        printf("\r\n");
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
            if (cumulative_encoder_data_1 >= 600000)
            {
                // ʹ����չ�ǶȽ��о�ȷ��ͷ����
                // 0.0�ȱ�ʾ���ֵ�ǰ�Ƕȣ�180.0�ȱ�ʾ��ͷ180�ȣ�-180.0�ȱ�ʾ�������ͷ
                servo_set_pd_extended(360.0, true);   // ʹ����չ�Ƕ�ģʽ
            }
            else if (cumulative_encoder_data_1 > 200000 && cumulative_encoder_data_1 < 400000)
            {
                // ����180�ȵ�ͷ��ʹ����չ�Ƕȿ��Ծ�ȷ���������������ҵ�ͷ
                // �����ǰ��չ�Ƕ���30�ȣ�Ŀ��210�ȱ�ʾ˳ʱ��ת180��
                // �����ǰ��չ�Ƕ���30�ȣ�Ŀ��-150�ȱ�ʾ��ʱ��ת180��
                servo_set_pd_extended(180.0, true);   // ʹ����չ�Ƕ�ģʽ
            }
            else
            {
                servo_set_pd_extended(0.0, true);   // ʹ����չ�Ƕ�ģʽ
            }

            // ��Ŀ2������s��
            // 11000��60cm
            // ���Ŀ��Ʋ���
            // float encoder_scale = 1.0f;   // ����������ϵ��
            // float servo_scale   = 1.0f;   // ����Ƕ�����ϵ��

            // // ·������
            // float init_straight = 15000;   // ��ʼֱ�о���

            // // ȥ��S�����
            // float s_angle_go   = 40.0f;   // ȥ��S��Ƕ�
            // float s_segment_go = 50000;   // ȥ�̵���S�䳤��

            // // �س�S�����
            // float s_angle_back   = 40.0f;   // �س�S��Ƕȣ��ɵ���������
            // float s_segment_back = 50000;   // �س̵���S�䳤�ȣ��ɵ���������

            // float pre_turn_straight = 30000;     // ��ͷǰֱ�о���
            // float turn_distance     = 50000;     // ��ͷ�׶ξ��루1�Σ�
            // float turn_angle        = -180.0f;   // ��ͷ��׼�Ƕȣ���ֵ=��ת��

            // float enc = cumulative_encoder_data_1 * encoder_scale;

            // // 1. ��ʼֱ��
            // if (enc < init_straight)
            // {
            //     servo_set_pd_extended(0.0, true);
            // }
            // // 2. ȥ��S�䣨3�Σ�
            // else if (enc < init_straight + 3 * s_segment_go)
            // {
            //     int   phase = (int)((enc - init_straight) / s_segment_go) % 3;
            //     float angle = (phase == 1 ? -s_angle_go : s_angle_go);   // ��λ1��ת��������ת
            //     servo_set_pd_extended(angle * servo_scale, true);
            // }
            // // 3. ��ͷǰֱ��
            // else if (enc < init_straight + 3 * s_segment_go + pre_turn_straight)
            // {
            //     servo_set_pd_extended(0.0, true);
            // }
            // // 4. ��ͷ�׶Σ�1�Σ�
            // else if (enc < init_straight + 3 * s_segment_go + pre_turn_straight + turn_distance)
            // {
            //     servo_set_pd_extended(turn_angle * servo_scale, true);   // �̶�-180���ͷ
            // }
            // // 5. �س�S�䣨3�Σ������������ƣ�
            // else if (enc < init_straight + 3 * s_segment_go + pre_turn_straight + turn_distance + 3 * s_segment_back)
            // {
            //     int phase = (int)((enc - init_straight - 3 * s_segment_go - pre_turn_straight - turn_distance) /
            //                       s_segment_back) %
            //                 3;
            //     float angle = turn_angle + (phase == 1 ? s_angle_back : -s_angle_back);   // ��λ1:-140�㣬����:-220��
            //     servo_set_pd_extended(angle * servo_scale, true);
            // }
            // // 6. ����ֱ�У�����-180�㣩
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


/*
 * ��չ�Ƕ�ϵͳʹ��ʾ����
 *
 * 1. ��ͳģʽ�������ݣ���

 *    servo_set_pd(90.0);  // ת��90�ȣ���������-180~180��Χ
 *
 * 2. ��չ�Ƕ�ģʽ��
 *    servo_set_pd_extended(270.0, true);   // ��0��˳ʱ��ת270��
 *    servo_set_pd_extended(-90.0, true);   // ��0����ʱ��ת90��
 *    servo_set_pd_extended(450.0, true);   // ��0��˳ʱ��ת450�ȣ�1Ȧ+90�ȣ�
 *
 * 3. ��ȷ��ͷ���ƣ�
 *    float current = IMU_GetExtendedYaw();
 *    servo_set_pd_extended(current + 180.0, true);  // ˳ʱ���ͷ180��
 *    servo_set_pd_extended(current - 180.0, true);  // ��ʱ���ͷ180��
 *
 * 4. �Ƕ����ã�
 *    IMU_ResetExtendedAngles();  // �����ۻ��Ƕ�
 *
 * 5. ���ó�ʼ�Ƕȣ�
 *    IMU_SetExtendedYaw(100.0);  // ���õ�ǰ��չ�Ƕ�Ϊ100��
 */


/*------------------------------------test------------------------------------*/
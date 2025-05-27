/*********************************************************************************************************************
 * TC264 Opensourec Library ����TC264 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
 * Copyright (c) 2022 SEEKFREE ��ɿƼ�
 *
 * ���ļ��� TC264 ��Դ���һ����
 *
 * TC264 ��Դ�� ��������
 * �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
 * �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
 *
 * ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
 * ����û�������������Ի��ʺ��ض���;�ı�֤
 * ����ϸ����μ� GPL
 *
 * ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
 * ���û�У������<https://www.gnu.org/licenses/>
 *
 * ����ע����
 * ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
 * �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
 * ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
 * ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
 *
 * �ļ�����          cpu0_main
 * ��˾����          �ɶ���ɿƼ����޹�˾
 * �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
 * ��������          ADS v1.9.4
 * ����ƽ̨          TC264D
 * ��������          https://seekfree.taobao.com/
 *
 * �޸ļ�¼
 * ����              ����                ��ע
 * 2022-09-15       pudding            first version
 ********************************************************************************************************************/
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��

// ******************************* HFUT-CITA-CODE ************************************
//      _      __       _          ___ _ _                ___          _            //
//     | |__  / _|_   _| |_       / __(_) |_ __ _        / __\___   __| | ___       //
//     | '_ \| |_| | | | __|____ / /  | | __/ _` |_____ / /  / _ \ / _` |/ _ \      //
//     | | | |  _| |_| | ||_____/ /___| | || (_| |_____/ /__| (_) | (_| |  __/      //
//     |_| |_|_|  \__,_|\__|    \____/|_|\__\__,_|     \____/\___/ \__,_|\___|      //
//                                                                                  //
// ******************************* HFUT-CITA-CODE ************************************

// *********************************** �������� ***************************************
int core0_main(void)
{
    clock_init();   // ��ȡʱ��Ƶ��<��ر���>
    debug_init();   // ��ʼ��Ĭ�ϵ��Դ���
    // �˴���д�û����� ���������ʼ�������
// 10000hz����sbusң�ؽ���
// #if SBUS_PIT_START_FLAG
//    pit_us_init(SBUS_PIT, 100);
// #endif

// IMUƵ����ߵ�1000Hz
#if IMU_PIT_START_FLAG
    pit_us_init(IMU_PIT, 1000);   // 100Hz (10ms)
#endif
#if ENCODER_PIT_START_FLAG
    // ������Ƶ���޸�Ϊ50Hz
    pit_ms_init(ENCODER_PIT, 20);   // 50Hz (20ms)
#endif
    // ���Ƶ����ߵ�300Hz
#if SERVO_PIT_START_FLAG
    pit_us_init(SERVO_PIT, 3333);   // 300Hz (3.33ms)
#endif
    // GPS����ʹ���жϣ�ֱ������ѭ���д���
    // #if GPS_PIT_START_FLAG
    //    pit_ms_init(GPS_PIT, 100);
    // #endif

    // ң���������ж�������CPU1��������CPU0�г�ʼ��
    // �˴���д�û����� ���������ʼ�������


    all_init();
    cpu_wait_event_ready();   // �ȴ����к��ĳ�ʼ�����
    system_start();
    // int            iterations = 1000000;   // ���ӵ��������Ի�ø��ȶ��Ľ��
    // float          input_val;
    // unsigned long  start_time, end_time;
    // float          cpu_time_used;
    // volatile float result=1;
    // start_time = system_getval_ms();
    // for (int i = 0; i < iterations; i++)
    // {
    //     // �����������ֵ
    //     input_val = 1.001f;
    //     result    *=input_val ;
    //     result/=(input_val+0.0001f);
    // }
    // end_time      = system_getval_ms();
    // cpu_time_used = (float)(end_time - start_time);
    // printf("func_sin time: %f us, result: %f\n", cpu_time_used, result);
    // test_gps_math_functions();
    while (TRUE)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���
        // double_test();
        // servo_test();
        // drv8701_motor_test();
        // collect_and_store_points();

        for (int i = 0; i < handlers_count; i++)
        {
            // printf("Handler %d: name=%s, enabled=%d\r\n", i, handlers[i].name, handlers[i].is_enabled);
            if (handlers[i].is_enabled)
            {
                // timer_enable();
                handlers[i].proc();
                // timer_disable();
            }
        }
        // GPS_INIT_FLAG
        //   float meas_acc[3];
        //   float meas_gyro[3];
        //   float temp_degc;
        //   bsp_IcmGetRawData(meas_acc, meas_gyro, &temp_degc);
        //   printf("meas_acc: %f, %f, %f\n", meas_acc[0], meas_acc[1], meas_acc[2]);
        //   printf("meas_gyro: %f, %f, %f\n", meas_gyro[0], meas_gyro[1], meas_gyro[2]);
        // //  // printf("temp_degc: %f\n", temp_degc);
        //   system_delay_ms(100);

        //         if (1 == uart_receiver.finsh_flag)
        //         {
        //             sbus_proc();
        //         }
        // #if IMU_INIT_FLAG
        //         if (imu_pit_state)
        //         {
        //             imu_mahony_proc();
        //         }
        // #endif // IMU_INIT_FLAG
        // #if MOTOR_INIT_FLAG
        //         if (motor_encoder_pit_state)
        //         {
        //             motor_encoder_proc();
        //         }
        // #endif // MOTOR_INIT_FLAG
        // #if GPS_INIT_FLAG
        //         if (gps_pit_state)
        //         {
        //             gps_proc();
        //         }
        // #endif // GPS_INIT_FLAG
        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}

#pragma section all restore
// *********************************** �������� ***************************************
// *****************************  ����Ϊ��ѧ�����ܲ��� *********************************
// int             iterations = 10000;   // ���ӵ��������Ի�ø��ȶ��Ľ��
// double          input_val;
// int             pow_val;
// double          y_val;
// double          x_val;
// unsigned long   start_time, end_time;
// double          cpu_time_used;
// volatile double result;

// // ��ʼ�����������
// srand(system_getval_ms());

// // ���� pow ����
// start_time = system_getval_ms();
// for (int i = 0; i < iterations; i++)
// {
//     // �����������ֵ
//     input_val = (double)rand() / RAND_MAX * 10.0;   // 0.0 �� 10.0 ֮��������
//     pow_val   = rand() % 5;                         // 0 �� 4 ֮����������
//     result    = pow(input_val, pow_val);
// }
// end_time      = system_getval_ms();
// cpu_time_used = (double)(end_time - start_time);
// printf("math pow time: %f us, result: %f\n", cpu_time_used, result);

// start_time = system_getval_ms();
// for (int i = 0; i < iterations; i++)
// {
//     // �����������ֵ
//     input_val = (double)rand() / RAND_MAX * 10.0;   // 0.0 �� 10.0 ֮��������
//     pow_val   = rand() % 5;                         // 0 �� 4 ֮����������
//     result    = func_pow(input_val, pow_val);
// }
// end_time      = system_getval_ms();
// cpu_time_used = (double)(end_time - start_time);
// printf("func_pow time: %f us, result: %f\n", cpu_time_used, result);

// // ���� sqrt ����
// start_time = system_getval_ms();
// for (int i = 0; i < iterations; i++)
// {
//     // �����������ֵ
//     input_val = (double)rand() / RAND_MAX * 10.0;   // 0.0 �� 10.0 ֮��������
//     result    = sqrt(input_val);
// }
// end_time      = system_getval_ms();
// cpu_time_used = (double)(end_time - start_time);
// printf("math sqrt time: %f us, result: %f\n", cpu_time_used, result);

// start_time = system_getval_ms();
// for (int i = 0; i < iterations; i++)
// {
//     // �����������ֵ
//     input_val = (double)rand() / RAND_MAX * 10.0;   // 0.0 �� 10.0 ֮��������
//     result    = func_sqrt(input_val);
// }
// end_time      = system_getval_ms();
// cpu_time_used = (double)(end_time - start_time);
// printf("func_sqrt time: %f us, result: %f\n", cpu_time_used, result);

// // ���� cos ����
// start_time = system_getval_ms();
// for (int i = 0; i < iterations; i++)
// {
//     // �����������ֵ
//     input_val = (double)rand() / RAND_MAX * 2 * MY_PI;   // 0.0 �� 2*PI ֮��������
//     result    = cos(input_val);
// }
// end_time      = system_getval_ms();
// cpu_time_used = (double)(end_time - start_time);
// printf("math cos time: %f us, result: %f\n", cpu_time_used, result);

// start_time = system_getval_ms();
// for (int i = 0; i < iterations; i++)
// {
//     // �����������ֵ
//     input_val = (double)rand() / RAND_MAX * 2 * MY_PI;   // 0.0 �� 2*PI ֮��������
//     result    = func_cos(input_val);
// }
// end_time      = system_getval_ms();
// cpu_time_used = (double)(end_time - start_time);
// printf("func_cos time: %f us, result: %f\n", cpu_time_used, result);

// // ���� sin ����
// start_time = system_getval_ms();
// for (int i = 0; i < iterations; i++)
// {
//     // �����������ֵ
//     input_val = (double)rand() / RAND_MAX * 2 * MY_PI;   // 0.0 �� 2*PI ֮��������
//     result    = sin(input_val);
// }
// end_time      = system_getval_ms();
// cpu_time_used = (double)(end_time - start_time);
// printf("math sin time: %f us, result: %f\n", cpu_time_used, result);

// start_time = system_getval_ms();
// for (int i = 0; i < iterations; i++)
// {
//     // �����������ֵ
//     input_val = (double)rand() / RAND_MAX * 2 * MY_PI;   // 0.0 �� 2*PI ֮��������
//     result    = func_sin(input_val);
// }
// end_time      = system_getval_ms();
// cpu_time_used = (double)(end_time - start_time);
// printf("func_sin time: %f us, result: %f\n", cpu_time_used, result);

// // ���� atan2 ����
// start_time = system_getval_ms();
// for (int i = 0; i < iterations; i++)
// {
//     // �����������ֵ
//     y_val  = (double)rand() / RAND_MAX * 2.0 - 1.0;   // -1.0 �� 1.0 ֮��������
//     x_val  = (double)rand() / RAND_MAX * 2.0 - 1.0;   // -1.0 �� 1.0 ֮��������
//     result = atan2(y_val, x_val);
// }
// end_time      = system_getval_ms();
// cpu_time_used = (double)(end_time - start_time);
// printf("math atan2 time: %f us, result: %f\n", cpu_time_used, result);

// start_time = system_getval_ms();
// for (int i = 0; i < iterations; i++)
// {
//     // �����������ֵ
//     y_val  = (double)rand() / RAND_MAX * 2.0 - 1.0;   // -1.0 �� 1.0 ֮��������
//     x_val  = (double)rand() / RAND_MAX * 2.0 - 1.0;   // -1.0 �� 1.0 ֮��������
//     result = fast_atan2(y_val, x_val);
// }
// end_time      = system_getval_ms();
// cpu_time_used = (double)(end_time - start_time);
// printf("fast_atan2 time: %f us, result: %f\n", cpu_time_used, result);

// double temp_pi = 3.14159265358979323846;
// double temp_e  = 2.71828182845904523536;
// double num1    = 1.0;
// for (int i = 0; i < 1000000; i++)
// {
//     num1 *= temp_pi;
//     num1 /= temp_e;
// }
// double result = system_getval_us();
// printf("Result: %f\n", result);

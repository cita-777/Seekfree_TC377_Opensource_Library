/*********************************************************************************************************************
 * TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 TC264 开源库的一部分
 *
 * TC264 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
 * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
 * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
 * 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称          cpu0_main
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          ADS v1.9.4
 * 适用平台          TC264D
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2022-09-15       pudding            first version
 ********************************************************************************************************************/
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// ******************************* HFUT-CITA-CODE ************************************
//      _      __       _          ___ _ _                ___          _            //
//     | |__  / _|_   _| |_       / __(_) |_ __ _        / __\___   __| | ___       //
//     | '_ \| |_| | | | __|____ / /  | | __/ _` |_____ / /  / _ \ / _` |/ _ \      //
//     | | | |  _| |_| | ||_____/ /___| | || (_| |_____/ /__| (_) | (_| |  __/      //
//     |_| |_|_|  \__,_|\__|    \____/|_|\__\__,_|     \____/\___/ \__,_|\___|      //
//                                                                                  //
// ******************************* HFUT-CITA-CODE ************************************

// *********************************** 代码区域 ***************************************
int core0_main(void)
{
    clock_init();   // 获取时钟频率<务必保留>
    debug_init();   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等
// 10000hz进行sbus遥控接受
// #if SBUS_PIT_START_FLAG
//    pit_us_init(SBUS_PIT, 100);
// #endif

// IMU频率提高到1000Hz
#if IMU_PIT_START_FLAG
    pit_us_init(IMU_PIT, 1000);   // 100Hz (10ms)
#endif
#if ENCODER_PIT_START_FLAG
    // 编码器频率修改为50Hz
    pit_ms_init(ENCODER_PIT, 20);   // 50Hz (20ms)
#endif
    // 舵机频率提高到300Hz
#if SERVO_PIT_START_FLAG
    pit_us_init(SERVO_PIT, 3333);   // 300Hz (3.33ms)
#endif
    // GPS不再使用中断，直接在主循环中处理
    // #if GPS_PIT_START_FLAG
    //    pit_ms_init(GPS_PIT, 100);
    // #endif

    // 遥控器串口中断已移至CPU1，不再在CPU0中初始化
    // 此处编写用户代码 例如外设初始化代码等


    all_init();
    cpu_wait_event_ready();   // 等待所有核心初始化完毕
    system_start();
    // int            iterations = 1000000;   // 增加迭代次数以获得更稳定的结果
    // float          input_val;
    // unsigned long  start_time, end_time;
    // float          cpu_time_used;
    // volatile float result=1;
    // start_time = system_getval_ms();
    // for (int i = 0; i < iterations; i++)
    // {
    //     // 生成随机输入值
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
        // 此处编写需要循环执行的代码
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
        // 此处编写需要循环执行的代码
    }
}

#pragma section all restore
// *********************************** 代码区域 ***************************************
// *****************************  以下为数学库性能测试 *********************************
// int             iterations = 10000;   // 增加迭代次数以获得更稳定的结果
// double          input_val;
// int             pow_val;
// double          y_val;
// double          x_val;
// unsigned long   start_time, end_time;
// double          cpu_time_used;
// volatile double result;

// // 初始化随机数种子
// srand(system_getval_ms());

// // 测试 pow 函数
// start_time = system_getval_ms();
// for (int i = 0; i < iterations; i++)
// {
//     // 生成随机输入值
//     input_val = (double)rand() / RAND_MAX * 10.0;   // 0.0 到 10.0 之间的随机数
//     pow_val   = rand() % 5;                         // 0 到 4 之间的随机整数
//     result    = pow(input_val, pow_val);
// }
// end_time      = system_getval_ms();
// cpu_time_used = (double)(end_time - start_time);
// printf("math pow time: %f us, result: %f\n", cpu_time_used, result);

// start_time = system_getval_ms();
// for (int i = 0; i < iterations; i++)
// {
//     // 生成随机输入值
//     input_val = (double)rand() / RAND_MAX * 10.0;   // 0.0 到 10.0 之间的随机数
//     pow_val   = rand() % 5;                         // 0 到 4 之间的随机整数
//     result    = func_pow(input_val, pow_val);
// }
// end_time      = system_getval_ms();
// cpu_time_used = (double)(end_time - start_time);
// printf("func_pow time: %f us, result: %f\n", cpu_time_used, result);

// // 测试 sqrt 函数
// start_time = system_getval_ms();
// for (int i = 0; i < iterations; i++)
// {
//     // 生成随机输入值
//     input_val = (double)rand() / RAND_MAX * 10.0;   // 0.0 到 10.0 之间的随机数
//     result    = sqrt(input_val);
// }
// end_time      = system_getval_ms();
// cpu_time_used = (double)(end_time - start_time);
// printf("math sqrt time: %f us, result: %f\n", cpu_time_used, result);

// start_time = system_getval_ms();
// for (int i = 0; i < iterations; i++)
// {
//     // 生成随机输入值
//     input_val = (double)rand() / RAND_MAX * 10.0;   // 0.0 到 10.0 之间的随机数
//     result    = func_sqrt(input_val);
// }
// end_time      = system_getval_ms();
// cpu_time_used = (double)(end_time - start_time);
// printf("func_sqrt time: %f us, result: %f\n", cpu_time_used, result);

// // 测试 cos 函数
// start_time = system_getval_ms();
// for (int i = 0; i < iterations; i++)
// {
//     // 生成随机输入值
//     input_val = (double)rand() / RAND_MAX * 2 * MY_PI;   // 0.0 到 2*PI 之间的随机数
//     result    = cos(input_val);
// }
// end_time      = system_getval_ms();
// cpu_time_used = (double)(end_time - start_time);
// printf("math cos time: %f us, result: %f\n", cpu_time_used, result);

// start_time = system_getval_ms();
// for (int i = 0; i < iterations; i++)
// {
//     // 生成随机输入值
//     input_val = (double)rand() / RAND_MAX * 2 * MY_PI;   // 0.0 到 2*PI 之间的随机数
//     result    = func_cos(input_val);
// }
// end_time      = system_getval_ms();
// cpu_time_used = (double)(end_time - start_time);
// printf("func_cos time: %f us, result: %f\n", cpu_time_used, result);

// // 测试 sin 函数
// start_time = system_getval_ms();
// for (int i = 0; i < iterations; i++)
// {
//     // 生成随机输入值
//     input_val = (double)rand() / RAND_MAX * 2 * MY_PI;   // 0.0 到 2*PI 之间的随机数
//     result    = sin(input_val);
// }
// end_time      = system_getval_ms();
// cpu_time_used = (double)(end_time - start_time);
// printf("math sin time: %f us, result: %f\n", cpu_time_used, result);

// start_time = system_getval_ms();
// for (int i = 0; i < iterations; i++)
// {
//     // 生成随机输入值
//     input_val = (double)rand() / RAND_MAX * 2 * MY_PI;   // 0.0 到 2*PI 之间的随机数
//     result    = func_sin(input_val);
// }
// end_time      = system_getval_ms();
// cpu_time_used = (double)(end_time - start_time);
// printf("func_sin time: %f us, result: %f\n", cpu_time_used, result);

// // 测试 atan2 函数
// start_time = system_getval_ms();
// for (int i = 0; i < iterations; i++)
// {
//     // 生成随机输入值
//     y_val  = (double)rand() / RAND_MAX * 2.0 - 1.0;   // -1.0 到 1.0 之间的随机数
//     x_val  = (double)rand() / RAND_MAX * 2.0 - 1.0;   // -1.0 到 1.0 之间的随机数
//     result = atan2(y_val, x_val);
// }
// end_time      = system_getval_ms();
// cpu_time_used = (double)(end_time - start_time);
// printf("math atan2 time: %f us, result: %f\n", cpu_time_used, result);

// start_time = system_getval_ms();
// for (int i = 0; i < iterations; i++)
// {
//     // 生成随机输入值
//     y_val  = (double)rand() / RAND_MAX * 2.0 - 1.0;   // -1.0 到 1.0 之间的随机数
//     x_val  = (double)rand() / RAND_MAX * 2.0 - 1.0;   // -1.0 到 1.0 之间的随机数
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

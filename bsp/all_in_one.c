/**
 * *****************************************************************************
 * @file        all_in_one.c
 * @brief       汇总各模块
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
// 全局函数指针表,用于存放各个处理函数,main中调用
ProcHandler handlers[] = {
#if SBUS_INIT_FLAG
    {sbus_proc, 1, "sbus"},   // uart接受完成标识位执行
#endif                        // SBUS_INIT_FLAG
#if MENU_INIT_FLAG
//{MenuTask, 0, "menu"},   // 10hz中断执行
#endif   // MENU_INIT_FLAG
#if IMU_INIT_FLAG
    {imu_proc, 0, "imu"},   // 100hz中断执行
                            // {imu_oscilloscope_isr, 0, "imu_oscilloscope"},   // 100hz中断执行
#endif                      // IMU_INIT_FLAG
#if MOTOR_INIT_FLAG
    {motor_encoder_proc, 0, "motor_encoder"},   // 100hz中断执行
#endif                                          // MOTOR_INIT_FLAG
#if GPS_INIT_FLAG
    {gps_proc, 0, "gps"},                                  // 10hz中断执行
#endif                                                     // GPS_INIT_FLAG
    {navigation_proc, 0, "navigation"},                    // 导航更新函数
    {collect_and_store_points, 0, "gps_get_point_mode"},   // 采点模式
};

const int handlers_count = sizeof(handlers) / sizeof(handlers[0]);
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void all_init()
{
    // 初始化ips
    ips_sbus_init();
    // 显示一个RGB565色彩启动界面图片 原图240*320 显示240*320 低位在前
    ips_show_rgb565_image(0, 0, (const uint16*)gImage_start, 240, 320, 240, 320, 0);
    flash_init();


#if IMU_WIFI_SEND_FLAG || MOTOR_ENCODER_WIFI_SEND_FLAG || GPS_WIFI_SEND_FLAG || PID_WIFI_SEND_FLAG || PD_WIFI_SEND_FLAG
    wifi_init();
#endif   // IMU_WIFI_SEND_FLAG || MOTOR_ENCODER_WIFI_SEND_FLAG || GPS_WIFI_SEND_FLAG || PID_WIFI_SEND_FLAG ||
         // PD_WIFI_SEND_FLAG
#if GPS_INIT_FLAG
    gps_init();
#endif   // GPS_INIT_FLAG
#if KEY_INIT_FLAG
    Key_init();
#endif   // KEY_INIT_FLAG
#if SERVO_INIT_FLAG
    servo_init();
#endif   // SERVO_INIT_FLAG
#if MOTOR_INIT_FLAG
    // drv8701_motor_init();
    bldc_motor_init();
#endif   // MOTOR_INIT_FLAG
    // 初始化imu和spi通讯
#if IMU_INIT_FLAG
    system_delay_ms(1000);   // 等待1秒消除上电抖动对零飘处理的影响
    imu_init();
#endif   // IMU_INIT_FLAG
    // 结束启动界面
    ips_clear();
#if MENU_INIT_FLAG
    MainMenuInit();
#endif   // MENU_INIT_FLAG
    // 蜂鸣器响三声代表初始化过程完成
    Buzzer_init();
    navigation_init();
    Buzzer_check(100, 70);
    Buzzer_check(100, 50);
    Buzzer_check(50, 10);

    // 结束启动界面
    // ips_clear();
}

/**
 * @brief       启动处理函数
 *
 * @param       name
 */
void enable_handler(const char* name)
{
    for (int i = 0; i < handlers_count; i++)
    {
        if (strcmp(handlers[i].name, name) == 0)
        {
            handlers[i].is_enabled = 1;
            break;
        }
    }
}
/**
 * @brief       关闭处理函数
 *
 * @param       name
 */
void disable_handler(const char* name)
{
    for (int i = 0; i < sizeof(handlers) / sizeof(handlers[0]); i++)
    {
        if (strcmp(handlers[i].name, name) == 0)
        {
            handlers[i].is_enabled = 0;
            break;
        }
    }
}
/*------------------------------------test------------------------------------*/

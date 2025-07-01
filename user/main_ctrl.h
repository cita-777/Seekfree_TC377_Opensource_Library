/**
 * *****************************************************************************
 * @file        main_ctrl.h
 * @brief
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-26
 * @copyright   cita
 * *****************************************************************************
 */
#ifndef MAIN_CTRL_H
#define MAIN_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------include-----------------------------------*/

/*-----------------------------------macro------------------------------------*/
#define Main_FLAG (0)   // 主程序开启标志位

#define MENU_INIT_FLAG (1)    // 菜单初始化标志位
#define SBUS_INIT_FLAG (1)    // SBUS初始化标志位
#define IMU_INIT_FLAG (1)     // IMU初始化标志位
#define FLASH_INIT_FLAG (1)   // FLASH数据初始化标志位
#define GPS_INIT_FLAG (1)     // GPS初始化标志位
#define KEY_INIT_FLAG (1)     // 按键初始化标志位
#define SERVO_INIT_FLAG (1)   // 舵机初始化标志位
#define MOTOR_INIT_FLAG (1)   // 电机初始化标志位
#define MENU_INIT_FLAG (1)    // 菜单初始化标志位

#define IMU_WIFI_SEND_FLAG (0)             // IMU使用WIFI发送标志位
#define MOTOR_ENCODER_WIFI_SEND_FLAG (0)   // 编码器使用WIFI发送标志位
#define GPS_WIFI_SEND_FLAG (0)             // GPS使用WIFI发送标志位
#define PID_WIFI_SEND_FLAG (1)             // PID使用WIFI发送标志位
#define PD_WIFI_SEND_FLAG (0)              // PD使用WIFI发送标志位


#define IMU_WIFI_GET_PARAM_FLAG (0)   // IMU使用WIFI获取上位机参数标志位
// #define SBUS_PIT_START_FLAG (1)     //SBUS定时器开启标志位
#define IMU_PIT_START_FLAG (1)   // IMU中断开启标志位

#define ENCODER_PIT_START_FLAG (1)   // 编码器定时器开启标志位
#define SERVO_PIT_START_FLAG (1)     // 舵机定时器开启标志位
// #define GPS_PIT_START_FLAG (1)       // GPS定时器开启标志位

#define GL_GPS_MATH_FLAG (1)        // GPS_Double专用数学库开启标志位
#define DEFAULT_GPS_MATH_FLAG (0)   // 双精度模式开启标志位(GPS数学库条件编译为双精度)
#define Double_Record_FLAG (1)      // 双精度浮点存储开启标志位//可用
/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
// 条件编译
// 为1,可以编译||为0,禁止编译

// 开启标志位-说明：
// 1.判断ADS是单精度模式还是双精度模式(单精度模式下：float和double都按照float定义)
// 2.判断采用何种数学库:Float_FLAG-GPS采用单精度数学库(ADS自带库)||Double_FLAG-GPS采用双精度数学库(自定义库)||Int_FLAG_D-GPS采用整数形参并采用默认数学库
// 3.判断才采用何种FLASH存储方式:Float_Record_FLAG-单精度||Double_Record_FLAG-双精度||Int_Record_FLAG-整数存储

// 固定模式：
// Float_FLAG  (1)---------Float_Record_FLAG  (1)//单精度
// Double_FLAG (1)---------Double_Record_FLAG (1)//双精度
// Int_FLAG_D  (1)---------Int_Record_FLAG    (1)//整数

/*
舵机和电机在一个定时器--7ms

占空比         霍尔
0              0
1000           50
2000           100
3000           140
4000           170
5000           190
6000           210
7000           220
8000           230
9000           240
10000          250
 */
/*------------------------------------test------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* MAIN_CTRL_H */

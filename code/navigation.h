/**
 * *****************************************************************************
 * @file        navigation.h
 * @brief       导航系统头文件，简化版本用于智能车科目导航
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-04-04
 * @copyright   cita
 * *****************************************************************************
 */

#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#include "zf_common_headfile.h"

/*----------------------------------宏定义-----------------------------------*/
// 导航模式定义
#define NAV_MODE_GPS_ONLY 0   // 仅使用GPS模式
#define NAV_MODE_IMU_ONLY 1   // 仅使用IMU模式
#define NAV_MODE_MANUAL 2     // 手动模式

// 科目状态定义
#define COURSE_STATE_IDLE 0        // 未运行
#define COURSE_STATE_RUNNING 1     // 正在运行
#define COURSE_STATE_COMPLETED 2   // 已完成

/*----------------------------------类型定义----------------------------------*/
// 导航系统结构体定义在.c文件中

/*--------------------------------公共接口函数--------------------------------*/
/**
 * @brief 初始化导航系统
 */
void navigation_init(void);

/**
 * @brief 简化导航更新函数
 * @param gps_lat 当前GPS纬度
 * @param gps_lon 当前GPS经度
 * @param gps_valid GPS数据是否有效
 * @param imu_yaw IMU航向角 (度)
 */
void navigation_update(double gps_lat, double gps_lon, uint8_t gps_valid, float imu_yaw);

/**
 * @brief 切换导航模式
 * @param mode 目标模式
 */
void navigation_set_mode(uint8_t mode);

/**
 * @brief 获取当前导航状态
 * @param lat 纬度指针
 * @param lon 经度指针
 * @param yaw 航向角指针
 */
void navigation_get_status(double* lat, double* lon, float* yaw);

/**
 * @brief 开始运行科目一
 */
void navigation_start_course1(void);

/**
 * @brief 停止当前科目
 */
void navigation_stop_course(void);

/**
 * @brief 获取当前科目状态
 * @return 当前科目状态
 */
uint8_t navigation_get_course_state(void);

/**
 * @brief 采集GPS点位
 * @param point_index 点位索引 (0-起点, 1-掉头起点, 2-掉头结束点, 3-终点)
 * @param lat 纬度
 * @param lon 经度
 */
void navigation_collect_point(uint8_t point_index, double lat, double lon);

/**
 * @brief 发送导航数据到上位机
 */
void navigation_send_data(void);

#endif   // _NAVIGATION_H_
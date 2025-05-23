/**
 * *****************************************************************************
 * @file        gps.h
 * @brief       华大北大tau1201 gps模块驱动
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-27
 * @copyright   cita
 * *****************************************************************************
 */

#ifndef GPS_H
#define GPS_H

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------include-----------------------------------*/
#include "main_ctrl.h"
/*-----------------------------------macro------------------------------------*/
#define GPS_TYPE (GN42A)   // GPS模块类型
// #define GPS_PIT (CCU61_CH0)
#define ALL_POINT_MAX 20          // 设置的采集点上限(一个点包含一对经纬度)
#define FLASH_SECTION_INDEX (0)   // 存储数据用的扇区
#define GPS_PAGE_INDEX (11)       // GPS存储页码
/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/
extern uint16 point_num;
// extern uint8 gps_pit_state;
#if Double_Record_FLAG
extern double Target_point[2][ALL_POINT_MAX];
// 用于储存采集的目标点信息，用于后续的位置计算(第一层是纬度，第二层是经度)
extern double Work_target_array[2][ALL_POINT_MAX];
// 用于将从flash中读取的目标点数组的数据复制过来，当这个数组被赋值时，后续的一切和Flash再无关系
#endif
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void gps_proc();
void gps_init();
void collect_and_store_points();
/*------------------------------------test------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* GPS_H */

#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#include "zf_common_typedef.h"

// 科目状态定义
#define COURSE_STATE_IDLE 0        // 空闲
#define COURSE_STATE_RUNNING 1     // 运行中
#define COURSE_STATE_COMPLETED 2   // 已完成
#define COURSE_STATE_ERROR 3       // 错误

// 公共接口函数
void    navigation_init(void);
void    navigation_proc(void);
void    navigation_update(double gps_lat, double gps_lon, uint8_t gps_valid, float imu_yaw);
void    navigation_get_status(double* lat, double* lon, float* yaw);
void    navigation_start_course1(void);
void    navigation_start_course2(void);
void    navigation_start_course3(void);
void    navigation_stop_course(void);
uint8_t navigation_get_course_state(void);
void    navigation_collect_point(uint8_t point_index, double lat, double lon);
void    navigation_send_data(void);

#endif
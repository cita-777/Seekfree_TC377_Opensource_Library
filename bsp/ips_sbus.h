/**
 * *****************************************************************************
 * @file        ips_sbus.h
 * @brief       2寸ips显示屏和sbus遥控器接受模块
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-26
 * @copyright   cita
 * *****************************************************************************
 */

#ifndef IPS_SBUS_H
#define IPS_SBUS_H

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------include-----------------------------------*/

/*-----------------------------------macro------------------------------------*/
#define IPS200_TYPE (IPS200_TYPE_SPI)

// IPS200宏定义
#define ips_show_int ips200_show_int
#define ips_show_uint ips200_show_uint
#define ips_show_float ips200_show_float
#define ips_show_chinese ips200_show_chinese
#define ips_clear ips200_clear
#define ips_init ips200_init
#define ips_show_rgb565_image ips200_show_rgb565_image

// #define SBUS_PIT                 (CCU60_CH1 )
/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/
extern uint8 point_flag;
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void ips_sbus_init(void);
void sbus_proc(void);

void ips_show_string(uint16 x, uint16 y, const char* str);
/*------------------------------------test------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* IPS_SBUS_H */

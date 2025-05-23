/**
 * *****************************************************************************
 * @file        menu_display.h
 * @brief
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-31
 * @copyright   cita
 * *****************************************************************************
 */

#ifndef MENU_DISPLAY_H
#define MENU_DISPLAY_H

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------include-----------------------------------*/

/*-----------------------------------macro------------------------------------*/
/*确定菜单使用的屏幕:TFT OR IPS200*/
/*在使用tft时，一行最多容纳16个字符*/
/*IPS200每个字符宽占8个格,长占16个格子*/
#define USE_IPS200 (1)
// #define USE_TFT

#define RGB565_dianqing 0x4810     // 靛青
#define RGB565_Silver 0XC618       // 银色
#define RGB565_Gray 0X8410         // 灰色
#define RGB565_SandBeige 0XE618    // 沙棕
#define RGB565_Pansy 0X7014        // 三色堇紫
#define RGB565_PeachPuff 0XFED7    // 粉扑桃色
#define RGB565_Periwinkle 0XCE7F   // 长春花色
#define RGB565_Pink 0XFE19         // 粉色
#define RGB565_RED 0xF800          // 红色
#define RGB565_GREEN 0x0400        // 绿色
#define RGB565_BLACK 0x0000        // 黑色
#define RGB565_WHITE 0xFFFF        // 白色
#define RGB565_chahe 0x8a86        // 茶褐色
#define RGB565_konqueBLUE 0x02ad   // 孔雀蓝
#if USE_IPS200

#    define ITEM_H 16                        // 菜单项高度
#    define ITEM_W ips200_width_max - 10     // 菜单项宽度
#    define MENU_H ips200_height_max         // 菜单页高度
#    define MENU_W ips200_width_max          // 菜单页宽度
#    define SCREEN_MARGIN 10                 // 菜单项边距
#    define BACK_COLOR RGB565_Pansy          // 背景颜色
#    define CLEAR_COLOR RGB565_Pansy         // 清屏颜色
#    define SELECT_COLOR RGB565_Periwinkle   // 选中颜色

#endif

#if USE_TFT
#    define ITEM_H 16
#    define ITEM_W TFT_X_MAX
#    define MENU_H TFT_Y_MAX
#    define MENU_W TFT_X_MAX
#    define SCREEN_MARGIN 0
#    define BACK_COLOR BLACK
#    define CLEAR_COLOR BLACK
#    define SELECT_COLOR RED
#endif
/*----------------------------------typedef-----------------------------------*/
/*定义颜色数据类型*/
typedef uint16 COLOR;
/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
extern void MENU_setScreenColor(COLOR color);
extern void MENU_DisplayStringLine(uint8 line, const char *str);
extern void MENU_drawRectagnleFill(uint16 x_0, uint16 y_0, uint16 x_1, uint16 y_1, COLOR color);
/*------------------------------------test------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* MENU_DISPLAY_H */

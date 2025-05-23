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
/*ȷ���˵�ʹ�õ���Ļ:TFT OR IPS200*/
/*��ʹ��tftʱ��һ���������16���ַ�*/
/*IPS200ÿ���ַ���ռ8����,��ռ16������*/
#define USE_IPS200 (1)
// #define USE_TFT

#define RGB565_dianqing 0x4810     // ����
#define RGB565_Silver 0XC618       // ��ɫ
#define RGB565_Gray 0X8410         // ��ɫ
#define RGB565_SandBeige 0XE618    // ɳ��
#define RGB565_Pansy 0X7014        // ��ɫ����
#define RGB565_PeachPuff 0XFED7    // ������ɫ
#define RGB565_Periwinkle 0XCE7F   // ������ɫ
#define RGB565_Pink 0XFE19         // ��ɫ
#define RGB565_RED 0xF800          // ��ɫ
#define RGB565_GREEN 0x0400        // ��ɫ
#define RGB565_BLACK 0x0000        // ��ɫ
#define RGB565_WHITE 0xFFFF        // ��ɫ
#define RGB565_chahe 0x8a86        // ���ɫ
#define RGB565_konqueBLUE 0x02ad   // ��ȸ��
#if USE_IPS200

#    define ITEM_H 16                        // �˵���߶�
#    define ITEM_W ips200_width_max - 10     // �˵�����
#    define MENU_H ips200_height_max         // �˵�ҳ�߶�
#    define MENU_W ips200_width_max          // �˵�ҳ���
#    define SCREEN_MARGIN 10                 // �˵���߾�
#    define BACK_COLOR RGB565_Pansy          // ������ɫ
#    define CLEAR_COLOR RGB565_Pansy         // ������ɫ
#    define SELECT_COLOR RGB565_Periwinkle   // ѡ����ɫ

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
/*������ɫ��������*/
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

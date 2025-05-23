/**
 * *****************************************************************************
 * @file        menu_display.c
 * @brief
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-31
 * @copyright   cita
 * *****************************************************************************
 */

/*----------------------------------include-----------------------------------*/
#include "zf_common_headfile.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
/**
 * @brief:������Ļ��ʾ��ɫ
 */
void MENU_setScreenColor(COLOR color)
{
#ifdef USE_IPS200
    ips200_full(color);
#endif
#ifdef USE_TFT
    lcd_clear(color);
#endif
}

/**
 * @brief:��Ļ��ʾһ����Ϣ
 */
void MENU_DisplayStringLine(uint8 line, const char *str)
{
#ifdef USE_IPS200
    // ��lineǿתΪuint16����ips200_show_string����
    ips200_show_string(SCREEN_MARGIN, line * ITEM_H, str);
#endif
#ifdef USE_TFT
    lcd_showstr(SCREEN_MARGIN, line, str);
#endif
}

/**
 * @brief:��һ����
 */
void MENU_drwaPoint(uint16 x, uint16 y, COLOR color)
{
#ifdef USE_IPS200
    ips200_draw_point(x, y, color);
#endif
#ifdef USE_TFT
    lcd_drawpoint(x, y, color);
#endif
}

/**
 * @brief:��ˮƽ��
 */
void MENU_HLine(uint16 x_0, uint16 y_0, uint16 x_1, COLOR color)
{
    ips200_draw_line(x_0, y_0, x_1, y_0, color);
}
/**
 * @brief:����ֱ��
 */
void MENU_RLine(uint16 x_0, uint16 y_0, uint16 y_1, COLOR color)
{
    ips200_draw_line(x_0, y_0, x_0, y_1, color);
}

/**
 * @brief����һ������
 */
void MENU_drawRectagnleFill(uint16 x_0, uint16 y_0, uint16 x_1, uint16 y_1, COLOR color)
{
#ifdef USE_IPS200
    uint16 temp;

    if (x_0 > x_1)
    {
        temp = x_0;
        x_0  = x_1;
        x_1  = temp;
    }
    if (y_0 > y_1)
    {
        temp = y_0;
        y_0  = y_1;
        y_1  = temp;
    }

    if (y_0 == y_1) MENU_HLine(x_0, y_0, x_1, color);
    if (x_0 == x_1) MENU_RLine(x_0, y_0, y_1, color);

    while (y_1 >= y_0)
    {
        MENU_HLine(x_0, y_0, x_1, color);
        y_0++;
    }
#endif

#ifdef USE_TFT
    lcd_fill(x_0, y_0, x_1, y_1, color);
#endif
}
/*------------------------------------test------------------------------------*/

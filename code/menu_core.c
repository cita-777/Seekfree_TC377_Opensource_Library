/**
 * *****************************************************************************
 * @file        menu_core.c
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
uint8     g_MenuItemIndex     = 0;   // �˵���
uint8     g_nMenuLevel        = 0;   // �˵�ҳ
uint8     g_lastMenuItemIndex = 0;   // �ϴβ˵���
uint8     ItemNumb[MAX_MENU_LEVELS];
tMenuItem psMenuItem, psCurrentMenuItem;   // �˵�ҳ��Ĳ˵����ǰ�˵�ҳ��Ĳ˵���
tMenu     psPrevMenu[MAX_MENU_LEVELS];     //
tMenu     psCurrentMenu;                   // ��ǰ�˵�ҳ
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
/*private functions*/
// �ڴ������˵��ڲ����õĺ���
void IdleFunc(void);
void gps_get_point_mode_open(void);
void gps_get_point_mode_close(void);
void Menu_test(void);
void gps_clear_point_from_flash(void);
void draw_adaptive_curve(void);
void nav_start_course1(void);
void nav_start_course2(void);
void nav_start_course3(void);
void nav_stop_course(void);
void nav_show_status(void);
/*------------------------------ Menu level 3 -------------------------------*/
struct sMenuItem SubMenuItems_1_1[] = {
    /*�˵�������, ȷ�Ϻ�Ҫִ�еĺ������˳���Ҫִ�еĺ�������һ��*/
    {"Open", gps_get_point_mode_open, DisplayMenu, NULL},
    {"Close", gps_get_point_mode_close, DisplayMenu, NULL}};
struct sMenu SubMenu_1_1 = {"     Level 3", SubMenuItems_1_1, COUNT_OF(SubMenuItems_1_1)};

struct sMenuItem SubMenuItems_1_2[] = {
    /*�˵�������, Ҫִ�еĺ�����Ҫִ�еĺ�������һ��*/
    {"Item 1", IdleFunc, IdleFunc, NULL},
    {"Item 2", IdleFunc, IdleFunc, NULL},
    {"Item 3", IdleFunc, IdleFunc, NULL},
    {"Item 4", IdleFunc, IdleFunc, NULL},
    {"Item 5", IdleFunc, IdleFunc, NULL}};
struct sMenu SubMenu_1_2 = {"     Level 3", SubMenuItems_1_2, COUNT_OF(SubMenuItems_1_2)};

/*------------------------------ Menu level 2 -------------------------------*/
struct sMenuItem SubMenuItems_1[] = {
    /*�˵�������, Ҫִ�еĺ�����Ҫִ�еĺ�������һ��*/
    {"Get_Point_Mode", IdleFunc, IdleFunc, &SubMenu_1_1},
    {"Point_Map", draw_adaptive_curve, DisplayMenu, NULL},
    {"Clear_Point_From_Flash", gps_clear_point_from_flash, IdleFunc, NULL},
    {"Item 4", IdleFunc, IdleFunc, NULL},
    {"Item 5", IdleFunc, IdleFunc, NULL}};
struct sMenu SubMenu_1 = {"     Level 2", SubMenuItems_1, COUNT_OF(SubMenuItems_1)};

struct sMenuItem SubMenuItems_2[] = {
    /*�˵�������, Ҫִ�еĺ�������Ӧ�Ĺ��ܺ����������¶���ʱ�Ĺ��ܣ���һ���˵�*/
    {"course1", nav_start_course1, DisplayMenu, NULL},
    {"course2", nav_start_course2, DisplayMenu, NULL},
    {"course3", nav_start_course3, DisplayMenu, NULL},
    {"stop course", nav_stop_course, DisplayMenu, NULL},
    {"now state", nav_show_status, DisplayMenu, NULL}};
struct sMenu SubMenu_2 = {"     Level 2", SubMenuItems_2, COUNT_OF(SubMenuItems_2)};

struct sMenuItem SubMenuItems_3[] = {
    /*�˵�������, Ҫִ�еĺ�������Ӧ�Ĺ��ܺ����������¶���ʱ�Ĺ��ܣ���һ���˵�*/
    {"Item 1", IdleFunc, IdleFunc, NULL},
    {"Item 2", IdleFunc, IdleFunc, NULL},
    {"Item 3", IdleFunc, IdleFunc, NULL},
    {"Item 4", IdleFunc, IdleFunc, NULL},
    {"Item 5", IdleFunc, IdleFunc, NULL}};
struct sMenu SubMenu_3 = {"     Level 2", SubMenuItems_3, COUNT_OF(SubMenuItems_3)};
/*------------------------------ Menu level 1 -------------------------------*/
struct sMenuItem MainMenuItems[] = {
    /*�˵�������, Ҫִ�еĺ�������Ӧ�Ĺ��ܺ����������¶���ʱ�Ĺ��ܣ���һ���˵�*/
    {"GPS", IdleFunc, IdleFunc, &SubMenu_1},
    {"Item Two", IdleFunc, IdleFunc, &SubMenu_2},
    {"Item Three", IdleFunc, IdleFunc, &SubMenu_3},
    {"Item Four", IdleFunc, IdleFunc, NULL},
    {"Item Five", IdleFunc, IdleFunc, NULL}};
struct sMenu MainMenu = {"     Main menu", MainMenuItems, COUNT_OF(MainMenuItems)};
/**
 * @brief ��ʼ���п�Ŀһ
 */
void nav_start_course1(void)
{
    // ����
    ips_clear();

    // ������Ŀһ
    navigation_start_course1();

    ips_show_string(10, 0, "��Ŀһ��ʼ����!");

    // ��ʾʵʱ״̬
    nav_show_status();
}

/**
 * @brief ��ʼ���п�Ŀ��
 */
void nav_start_course2(void)
{
    // ����
    ips_clear();

    ips_show_string(10, 0, "��Ŀ��δʵ��!");
}

/**
 * @brief ��ʼ���п�Ŀ��
 */
void nav_start_course3(void)
{
    // ����
    ips_clear();

    ips_show_string(10, 0, "��Ŀ��δʵ��!");
}

/**
 * @brief ֹͣ��Ŀ����
 */
void nav_stop_course(void)
{
    // ֹͣ��ǰ��Ŀ
    navigation_stop_course();

    ips_show_string(10, 0, "��Ŀ��ֹͣ����!");
}

/**
 * @brief ��ʾ����״̬
 */
void nav_show_status(void)
{
    // ����
    ips_clear();

    // ��ȡ��ǰ����״̬
    double lat, lon;
    float  yaw;
    navigation_get_status(&lat, &lon, &yaw);

    // ��ȡ��ǰ��Ŀ״̬
    uint8_t course_state = navigation_get_course_state();

    // ��ʾ״̬��Ϣ
    ips_show_string(0, 0, "����״̬:");

    // ��ʾ������Ϣ
    ips_show_string(0, 16, "γ��:");
    ips_show_float(90, 16, lat, 6, 9);

    ips_show_string(0, 32, "����:");
    ips_show_float(90, 32, lon, 6, 9);

    ips_show_string(0, 48, "�����:");
    ips_show_float(90, 48, yaw, 2, 6);

    // ��ʾ��Ŀ״̬
    ips_show_string(0, 64, "��Ŀ״̬:");

    char* state_str = "";
    switch (course_state)
    {
    case COURSE_STATE_IDLE: state_str = "δ����"; break;
    case COURSE_STATE_RUNNING: state_str = "������"; break;
    case COURSE_STATE_COMPLETED: state_str = "�����"; break;
    default: state_str = "δ֪"; break;
    }
    ips_show_string(90, 64, state_str);

    // ��ʾ������ʾ
    ips_show_string(10, 90, "�����ؼ��ص��˵�");
}
void gps_get_point_mode_open(void)
{
    // printf("gps_get_point_mode_open\n");
    ips_show_string(10, 0, "gps_get_point_mode_open!");
    enable_handler("gps_get_point_mode");
}
void gps_get_point_mode_close(void)
{
    // printf("gps_get_point_mode_close\n");
    ips_show_string(10, 0, "gps_get_point_mode_close!");
    disable_handler("gps_get_point_mode");
}
void gps_clear_point_from_flash(void)
{
    if (flash_check(FLASH_SECTION_INDEX,
                    GPS_PAGE_INDEX))   // �ж�Flash�Ƿ������� : �����ݷ���1�������ݷ���0
    {
        flash_erase_page(FLASH_SECTION_INDEX, GPS_PAGE_INDEX);
        ips_show_string(10, 0, "flash_erase_success!");
    }
    else
    {
        ips_show_string(10, 0, "no_point_in_flash!");
    }
}
void draw_adaptive_curve(void)
{
    const int    screen_width  = 240;
    const int    screen_height = 320;
    const int    color_point   = RGB565_YELLOW;
    const double epsilon       = 1e-6;

    // ����ģʽ���� - ����Ϊ1���ò���ģʽ������ʹ��ȫ�ֱ���
    int    test_mode          = 1;   // ����Ϊ1���ò���ģʽ��0��ʹ��ȫ�ֱ���
    int    test_points_num    = 4;   // ���Ե�����
    double test_points[2][20] = {
        // ���֧��20�����Ե�
        {30.90660, 30.90660, 30.90568, 30.90568},      // γ��
        {118.71615, 118.71621, 118.71621, 118.71615}   // ����
    };

    // ����ģʽѡ��ʹ���ĸ�����Դ
    int points_num = test_mode ? test_points_num : ALL_POINT_MAX;

    if (points_num < 1) return;
    ips_clear();

    // ���㾭γ�ȷ�Χ
    double min_lat, max_lat, min_lon, max_lon;

    if (test_mode)
    {
        min_lat = max_lat = test_points[0][0];
        min_lon = max_lon = test_points[1][0];
    }
    else
    {
        min_lat = max_lat = Work_target_array[0][0];
        min_lon = max_lon = Work_target_array[1][0];
    }

    for (int i = 1; i < points_num; i++)
    {
        double lat, lon;
        if (test_mode)
        {
            lat = test_points[0][i];
            lon = test_points[1][i];
        }
        else
        {
            lat = Work_target_array[0][i];
            lon = Work_target_array[1][i];
        }

        if (lat < min_lat) min_lat = lat;
        if (lat > max_lat) max_lat = lat;
        if (lon < min_lon) min_lon = lon;
        if (lon > max_lon) max_lon = lon;
    }

    // ǿ����չ��С��Χ
    if (max_lat - min_lat < epsilon)
    {
        max_lat = min_lat + epsilon;
        printf("���棺γ�ȷ�Χ��С�����Զ���չ\n");
    }
    if (max_lon - min_lon < epsilon)
    {
        max_lon = min_lon + epsilon;
        printf("���棺���ȷ�Χ��С�����Զ���չ\n");
    }

    // ������������ʵ�ʷ�Χ
    double delta_lat = max_lat - min_lat;
    double delta_lon = max_lon - min_lon;

    // ����ȱ�����������Ĳ���
    double center_lat       = (min_lat + max_lat) / 2;
    double lon_scale_factor = cos(center_lat * 3.14159265358979323846 / 180.0);

    // �������ʵ�ʵ���Χ
    double actual_width  = delta_lon * lon_scale_factor;
    double actual_height = delta_lat;

    // ����ȱ�������
    double scale;
    int    draw_width, draw_height;
    int    offset_x, offset_y;

    if (actual_width / actual_height > (screen_width - 20) / (double)(screen_height - 20))
    {
        // ��������������Ļ�����Կ��Ϊ��׼
        scale       = (screen_width - 20) / actual_width;
        draw_width  = screen_width - 20;
        draw_height = (int)(actual_height * scale);
        offset_x    = 10;
        offset_y    = (screen_height - draw_height) / 2;
    }
    else
    {
        // ��������������Ļ���ߣ��Ը߶�Ϊ��׼
        scale       = (screen_height - 20) / actual_height;
        draw_height = screen_height - 20;
        draw_width  = (int)(actual_width * scale);
        offset_x    = (screen_width - draw_width) / 2;
        offset_y    = 10;
    }

    // ȷ���߾�������10����
    offset_x = offset_x < 10 ? 10 : offset_x;
    offset_y = offset_y < 10 ? 10 : offset_y;

    // ����ת��
    Point* screen_points = (Point*)malloc(points_num * sizeof(Point));
    if (!screen_points) return;

    for (int i = 0; i < points_num; i++)
    {
        double lat, lon;
        if (test_mode)
        {
            lat = test_points[0][i];
            lon = test_points[1][i];
        }
        else
        {
            lat = Work_target_array[0][i];
            lon = Work_target_array[1][i];
        }

        // ͳһ���ű���ӳ�䵽��Ļ
        int x = offset_x + (int)((lon - min_lon) * lon_scale_factor * scale);
        int y = screen_height - offset_y - (int)((lat - min_lat) * scale);

        // �ϸ��޷�
        x = (x < 0) ? 0 : (x >= screen_width) ? screen_width - 1 : x;
        y = (y < 0) ? 0 : (y >= screen_height) ? screen_height - 1 : y;

        screen_points[i].x = x;
        screen_points[i].y = y;
        printf("ת���� Point %d: x=%d, y=%d\n", i, x, y);
    }

    // ���Ʊ߽�� - ʹ��4���ߴ�����κ���
    int right_x  = offset_x + draw_width;
    int bottom_y = screen_height - offset_y;
    ips200_draw_line(offset_x, offset_y, right_x, offset_y, RGB565_WHITE);    // �ϱ�
    ips200_draw_line(offset_x, bottom_y, right_x, bottom_y, RGB565_WHITE);    // �±�
    ips200_draw_line(offset_x, offset_y, offset_x, bottom_y, RGB565_WHITE);   // ���
    ips200_draw_line(right_x, offset_y, right_x, bottom_y, RGB565_WHITE);     // �ұ�

    // ������ǿ�㣨3x3���ط��飩
    for (int i = 0; i < points_num; i++)
    {
        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                int px = screen_points[i].x + dx;
                int py = screen_points[i].y + dy;
                if (px >= 0 && px < screen_width && py >= 0 && py < screen_height)
                {
                    ips200_draw_point(px, py, color_point);
                }
            }
        }
    }

    free(screen_points);
}
/**
 * @brief ��ʼ���˵�
 */
void Menu_Init(void)
{
    // ��ǰ�˵�ҳ
    psCurrentMenu = &MainMenu;
    // ��һ���˵�
    psPrevMenu[g_nMenuLevel] = psCurrentMenu;
    // ��ǰ�˵�ҳ��Ĳ˵���
    psMenuItem = MainMenuItems;
}

/**
 * @brief ��ʾ�˵�
 */
void DisplayMenu(void)
{
    uint32    line = 0, index = 0;
    tMenuItem psMenuItem2;

    /*��ɫ�����Ļ*/
    MENU_setScreenColor(BACK_COLOR);
    /*��ʾ�˵�����*/
    MENU_DisplayStringLine(line, psCurrentMenu->pszTitle);
    line++;
    // ��ӡline
    // printf("start line = %d\n", line);
    /*��ʾ����ǰ�˵�ҳ���в˵���*/
    while (!(index >= (psCurrentMenu->nItems)))
    {
        psMenuItem2 = &(psCurrentMenu->psItems[index]);
        MENU_DisplayStringLine(line, psMenuItem2->pszTitle);
        line++;
        index++;
        // printf("next line = %d\n", line);
        // printf("next index = %d\n", index);
    }

    /*��ʾѡ�еĲ˵���*/
    MENU_drawRectagnleFill(0, (g_MenuItemIndex + 1) * ITEM_H, ITEM_W, (g_MenuItemIndex + 2) * ITEM_H, SELECT_COLOR);
    psMenuItem = &(psCurrentMenu->psItems[g_MenuItemIndex]);
    MENU_DisplayStringLine((g_MenuItemIndex + 1), psMenuItem->pszTitle);
}

/**
 * @brief ��ʼ���˵������ʩ
 */
void MainMenuInit(void)
{
    /*ips200��Ļ��ʼ��,ע��������Ĳ˵�ʹ�õ���Ļ��Ҫ����display.h�еĺ궨��*/
    // ips200_init();
    /*LCD��Ļ��ʼ��*/
    // lcd_init();
    /*��ʼ���˵��������*/
    Menu_Init();
    /*��ʾ�˵�*/
    DisplayMenu();
}

/**
 * @brief �˵�����
 */
void MenuTask(void)
{
    key_scan();
    if (key1_flag)
    {
        key1_flag = 0;
        UpFunc();
    }
    if (key2_flag)
    {
        key2_flag = 0;
        DownFunc();
    }
    if (key3_flag)
    {
        key3_flag = 0;
        SelFunc();
    }
    if (key4_flag)
    {
        key4_flag = 0;
        ReturnFunc();
    }
    disable_handler("menu");
}

/**
 * @brief None
 */
void IdleFunc(void)
{
    /*Nothing to execute:return*/
    return;
}

/**
 * @brief ѡ�ж���
 */
void SelFunc(void)
{
    /*�õ���ǰ�˵�ҳ�еĲ˵���*/
    psCurrentMenuItem = psMenuItem;

    /*�����Ӳ˵���ת*/
    if (psMenuItem->psSubMenu != NULL)
    {
        g_nMenuLevel++;
        g_MenuItemIndex = ItemNumb[g_nMenuLevel];
        /*�õ���ǰ�˵�����Ӳ˵�*/
        psCurrentMenu = psCurrentMenuItem->psSubMenu;
        psMenuItem    = &(psCurrentMenu->psItems)[g_MenuItemIndex];
        DisplayMenu();
        psPrevMenu[g_nMenuLevel] = psCurrentMenu;
    }
    /*ִ�в˵����*/
    psCurrentMenuItem->pfMenuFunc();
}

/**
 * @brief ���ض���
 */
void ReturnFunc(void)
{
    psMenuItem->pfUpDownMenuFunc();

    if (g_nMenuLevel == 0) g_nMenuLevel++;

    /*�õ���һ���˵�*/
    psCurrentMenu          = psPrevMenu[g_nMenuLevel - 1];
    psMenuItem             = &psCurrentMenu->psItems[0];
    ItemNumb[g_nMenuLevel] = 0;
    g_nMenuLevel--;
    g_MenuItemIndex = ItemNumb[g_nMenuLevel];

    DisplayMenu();
}

/**
 * @brief ����
 */
void UpFunc(void)
{
    /*ȡ��ѡ��״̬*/
    MENU_drawRectagnleFill(0, (g_MenuItemIndex + 1) * ITEM_H, ITEM_W, (g_MenuItemIndex + 2) * ITEM_H, BACK_COLOR);
    psMenuItem = &psCurrentMenu->psItems[g_MenuItemIndex];
    MENU_DisplayStringLine((g_MenuItemIndex + 1), psMenuItem->pszTitle);
    /*�ж��±�״̬*/
    if (g_MenuItemIndex > 0)
        g_MenuItemIndex--;
    else
        g_MenuItemIndex = psCurrentMenu->nItems - 1;
    /*����ѡ��״̬*/
    MENU_drawRectagnleFill(0, (g_MenuItemIndex + 1) * ITEM_H, ITEM_W, (g_MenuItemIndex + 2) * ITEM_H, SELECT_COLOR);
    psMenuItem = &psCurrentMenu->psItems[g_MenuItemIndex];
    MENU_DisplayStringLine((g_MenuItemIndex + 1), psMenuItem->pszTitle);
    /*��ѡ�еĲ˵�*/
    ItemNumb[g_nMenuLevel] = g_MenuItemIndex;
}

/**
 * @brief ����
 */
void DownFunc(void)
{
    /*ȡ��ѡ��״̬*/
    MENU_drawRectagnleFill(0, (g_MenuItemIndex + 1) * ITEM_H, ITEM_W, (g_MenuItemIndex + 2) * ITEM_H, BACK_COLOR);
    psMenuItem = &psCurrentMenu->psItems[g_MenuItemIndex];
    MENU_DisplayStringLine((g_MenuItemIndex + 1), psMenuItem->pszTitle);
    /*�ж��±�״̬*/
    if (g_MenuItemIndex >= ((psCurrentMenu->nItems) - 1))
        g_MenuItemIndex = 0;
    else
        g_MenuItemIndex++;
    /*����ѡ��״̬*/
    MENU_drawRectagnleFill(0, (g_MenuItemIndex + 1) * ITEM_H, ITEM_W, (g_MenuItemIndex + 2) * ITEM_H, SELECT_COLOR);
    psMenuItem = &psCurrentMenu->psItems[g_MenuItemIndex];
    MENU_DisplayStringLine((g_MenuItemIndex + 1), psMenuItem->pszTitle);
    /**/
    ItemNumb[g_nMenuLevel] = g_MenuItemIndex;
}

void Menu_test(void)
{
    MENU_setScreenColor(RGB565_GREEN);
}

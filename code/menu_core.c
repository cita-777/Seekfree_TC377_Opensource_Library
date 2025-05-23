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
uint8     g_MenuItemIndex     = 0;   // 菜单项
uint8     g_nMenuLevel        = 0;   // 菜单页
uint8     g_lastMenuItemIndex = 0;   // 上次菜单项
uint8     ItemNumb[MAX_MENU_LEVELS];
tMenuItem psMenuItem, psCurrentMenuItem;   // 菜单页里的菜单项，当前菜单页里的菜单项
tMenu     psPrevMenu[MAX_MENU_LEVELS];     //
tMenu     psCurrentMenu;                   // 当前菜单页
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
/*private functions*/
// 在此声明菜单内部调用的函数
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
    /*菜单项名称, 确认后要执行的函数，退出后要执行的函数，下一级*/
    {"Open", gps_get_point_mode_open, DisplayMenu, NULL},
    {"Close", gps_get_point_mode_close, DisplayMenu, NULL}};
struct sMenu SubMenu_1_1 = {"     Level 3", SubMenuItems_1_1, COUNT_OF(SubMenuItems_1_1)};

struct sMenuItem SubMenuItems_1_2[] = {
    /*菜单项名称, 要执行的函数，要执行的函数，下一级*/
    {"Item 1", IdleFunc, IdleFunc, NULL},
    {"Item 2", IdleFunc, IdleFunc, NULL},
    {"Item 3", IdleFunc, IdleFunc, NULL},
    {"Item 4", IdleFunc, IdleFunc, NULL},
    {"Item 5", IdleFunc, IdleFunc, NULL}};
struct sMenu SubMenu_1_2 = {"     Level 3", SubMenuItems_1_2, COUNT_OF(SubMenuItems_1_2)};

/*------------------------------ Menu level 2 -------------------------------*/
struct sMenuItem SubMenuItems_1[] = {
    /*菜单项名称, 要执行的函数，要执行的函数，下一级*/
    {"Get_Point_Mode", IdleFunc, IdleFunc, &SubMenu_1_1},
    {"Point_Map", draw_adaptive_curve, DisplayMenu, NULL},
    {"Clear_Point_From_Flash", gps_clear_point_from_flash, IdleFunc, NULL},
    {"Item 4", IdleFunc, IdleFunc, NULL},
    {"Item 5", IdleFunc, IdleFunc, NULL}};
struct sMenu SubMenu_1 = {"     Level 2", SubMenuItems_1, COUNT_OF(SubMenuItems_1)};

struct sMenuItem SubMenuItems_2[] = {
    /*菜单项名称, 要执行的函数（对应的功能函数），上下动作时的功能，下一级菜单*/
    {"course1", nav_start_course1, DisplayMenu, NULL},
    {"course2", nav_start_course2, DisplayMenu, NULL},
    {"course3", nav_start_course3, DisplayMenu, NULL},
    {"stop course", nav_stop_course, DisplayMenu, NULL},
    {"now state", nav_show_status, DisplayMenu, NULL}};
struct sMenu SubMenu_2 = {"     Level 2", SubMenuItems_2, COUNT_OF(SubMenuItems_2)};

struct sMenuItem SubMenuItems_3[] = {
    /*菜单项名称, 要执行的函数（对应的功能函数），上下动作时的功能，下一级菜单*/
    {"Item 1", IdleFunc, IdleFunc, NULL},
    {"Item 2", IdleFunc, IdleFunc, NULL},
    {"Item 3", IdleFunc, IdleFunc, NULL},
    {"Item 4", IdleFunc, IdleFunc, NULL},
    {"Item 5", IdleFunc, IdleFunc, NULL}};
struct sMenu SubMenu_3 = {"     Level 2", SubMenuItems_3, COUNT_OF(SubMenuItems_3)};
/*------------------------------ Menu level 1 -------------------------------*/
struct sMenuItem MainMenuItems[] = {
    /*菜单项名称, 要执行的函数（对应的功能函数），上下动作时的功能，下一级菜单*/
    {"GPS", IdleFunc, IdleFunc, &SubMenu_1},
    {"Item Two", IdleFunc, IdleFunc, &SubMenu_2},
    {"Item Three", IdleFunc, IdleFunc, &SubMenu_3},
    {"Item Four", IdleFunc, IdleFunc, NULL},
    {"Item Five", IdleFunc, IdleFunc, NULL}};
struct sMenu MainMenu = {"     Main menu", MainMenuItems, COUNT_OF(MainMenuItems)};
/**
 * @brief 开始运行科目一
 */
void nav_start_course1(void)
{
    // 清屏
    ips_clear();

    // 启动科目一
    navigation_start_course1();

    ips_show_string(10, 0, "科目一开始运行!");

    // 显示实时状态
    nav_show_status();
}

/**
 * @brief 开始运行科目二
 */
void nav_start_course2(void)
{
    // 清屏
    ips_clear();

    ips_show_string(10, 0, "科目二未实现!");
}

/**
 * @brief 开始运行科目三
 */
void nav_start_course3(void)
{
    // 清屏
    ips_clear();

    ips_show_string(10, 0, "科目三未实现!");
}

/**
 * @brief 停止科目运行
 */
void nav_stop_course(void)
{
    // 停止当前科目
    navigation_stop_course();

    ips_show_string(10, 0, "科目已停止运行!");
}

/**
 * @brief 显示导航状态
 */
void nav_show_status(void)
{
    // 清屏
    ips_clear();

    // 获取当前导航状态
    double lat, lon;
    float  yaw;
    navigation_get_status(&lat, &lon, &yaw);

    // 获取当前科目状态
    uint8_t course_state = navigation_get_course_state();

    // 显示状态信息
    ips_show_string(0, 0, "导航状态:");

    // 显示坐标信息
    ips_show_string(0, 16, "纬度:");
    ips_show_float(90, 16, lat, 6, 9);

    ips_show_string(0, 32, "经度:");
    ips_show_float(90, 32, lon, 6, 9);

    ips_show_string(0, 48, "航向角:");
    ips_show_float(90, 48, yaw, 2, 6);

    // 显示科目状态
    ips_show_string(0, 64, "科目状态:");

    char* state_str = "";
    switch (course_state)
    {
    case COURSE_STATE_IDLE: state_str = "未运行"; break;
    case COURSE_STATE_RUNNING: state_str = "运行中"; break;
    case COURSE_STATE_COMPLETED: state_str = "已完成"; break;
    default: state_str = "未知"; break;
    }
    ips_show_string(90, 64, state_str);

    // 显示返回提示
    ips_show_string(10, 90, "按返回键回到菜单");
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
                    GPS_PAGE_INDEX))   // 判断Flash是否有数据 : 有数据返回1，无数据返回0
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

    // 测试模式数据 - 设置为1启用测试模式，否则使用全局变量
    int    test_mode          = 1;   // 设置为1启用测试模式，0则使用全局变量
    int    test_points_num    = 4;   // 测试点数量
    double test_points[2][20] = {
        // 最多支持20个测试点
        {30.90660, 30.90660, 30.90568, 30.90568},      // 纬度
        {118.71615, 118.71621, 118.71621, 118.71615}   // 经度
    };

    // 根据模式选择使用哪个数据源
    int points_num = test_mode ? test_points_num : ALL_POINT_MAX;

    if (points_num < 1) return;
    ips_clear();

    // 计算经纬度范围
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

    // 强制扩展极小范围
    if (max_lat - min_lat < epsilon)
    {
        max_lat = min_lat + epsilon;
        printf("警告：纬度范围过小，已自动扩展\n");
    }
    if (max_lon - min_lon < epsilon)
    {
        max_lon = min_lon + epsilon;
        printf("警告：经度范围过小，已自动扩展\n");
    }

    // 计算地理坐标的实际范围
    double delta_lat = max_lat - min_lat;
    double delta_lon = max_lon - min_lon;

    // 计算等比例缩放所需的参数
    double center_lat       = (min_lat + max_lat) / 2;
    double lon_scale_factor = cos(center_lat * 3.14159265358979323846 / 180.0);

    // 调整后的实际地理范围
    double actual_width  = delta_lon * lon_scale_factor;
    double actual_height = delta_lat;

    // 计算等比例缩放
    double scale;
    int    draw_width, draw_height;
    int    offset_x, offset_y;

    if (actual_width / actual_height > (screen_width - 20) / (double)(screen_height - 20))
    {
        // 如果地理区域比屏幕更宽，以宽度为基准
        scale       = (screen_width - 20) / actual_width;
        draw_width  = screen_width - 20;
        draw_height = (int)(actual_height * scale);
        offset_x    = 10;
        offset_y    = (screen_height - draw_height) / 2;
    }
    else
    {
        // 如果地理区域比屏幕更高，以高度为基准
        scale       = (screen_height - 20) / actual_height;
        draw_height = screen_height - 20;
        draw_width  = (int)(actual_width * scale);
        offset_x    = (screen_width - draw_width) / 2;
        offset_y    = 10;
    }

    // 确保边距至少有10像素
    offset_x = offset_x < 10 ? 10 : offset_x;
    offset_y = offset_y < 10 ? 10 : offset_y;

    // 坐标转换
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

        // 统一缩放比例映射到屏幕
        int x = offset_x + (int)((lon - min_lon) * lon_scale_factor * scale);
        int y = screen_height - offset_y - (int)((lat - min_lat) * scale);

        // 严格限幅
        x = (x < 0) ? 0 : (x >= screen_width) ? screen_width - 1 : x;
        y = (y < 0) ? 0 : (y >= screen_height) ? screen_height - 1 : y;

        screen_points[i].x = x;
        screen_points[i].y = y;
        printf("转换后 Point %d: x=%d, y=%d\n", i, x, y);
    }

    // 绘制边界框 - 使用4条线代替矩形函数
    int right_x  = offset_x + draw_width;
    int bottom_y = screen_height - offset_y;
    ips200_draw_line(offset_x, offset_y, right_x, offset_y, RGB565_WHITE);    // 上边
    ips200_draw_line(offset_x, bottom_y, right_x, bottom_y, RGB565_WHITE);    // 下边
    ips200_draw_line(offset_x, offset_y, offset_x, bottom_y, RGB565_WHITE);   // 左边
    ips200_draw_line(right_x, offset_y, right_x, bottom_y, RGB565_WHITE);     // 右边

    // 绘制增强点（3x3像素方块）
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
 * @brief 初始化菜单
 */
void Menu_Init(void)
{
    // 当前菜单页
    psCurrentMenu = &MainMenu;
    // 上一级菜单
    psPrevMenu[g_nMenuLevel] = psCurrentMenu;
    // 当前菜单页里的菜单项
    psMenuItem = MainMenuItems;
}

/**
 * @brief 显示菜单
 */
void DisplayMenu(void)
{
    uint32    line = 0, index = 0;
    tMenuItem psMenuItem2;

    /*黑色填充屏幕*/
    MENU_setScreenColor(BACK_COLOR);
    /*显示菜单标题*/
    MENU_DisplayStringLine(line, psCurrentMenu->pszTitle);
    line++;
    // 打印line
    // printf("start line = %d\n", line);
    /*显示出当前菜单页所有菜单项*/
    while (!(index >= (psCurrentMenu->nItems)))
    {
        psMenuItem2 = &(psCurrentMenu->psItems[index]);
        MENU_DisplayStringLine(line, psMenuItem2->pszTitle);
        line++;
        index++;
        // printf("next line = %d\n", line);
        // printf("next index = %d\n", index);
    }

    /*显示选中的菜单项*/
    MENU_drawRectagnleFill(0, (g_MenuItemIndex + 1) * ITEM_H, ITEM_W, (g_MenuItemIndex + 2) * ITEM_H, SELECT_COLOR);
    psMenuItem = &(psCurrentMenu->psItems[g_MenuItemIndex]);
    MENU_DisplayStringLine((g_MenuItemIndex + 1), psMenuItem->pszTitle);
}

/**
 * @brief 初始化菜单相关设施
 */
void MainMenuInit(void)
{
    /*ips200屏幕初始化,注意这里更改菜单使用的屏幕后，要更改display.h中的宏定义*/
    // ips200_init();
    /*LCD屏幕初始化*/
    // lcd_init();
    /*初始化菜单相关配置*/
    Menu_Init();
    /*显示菜单*/
    DisplayMenu();
}

/**
 * @brief 菜单任务
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
 * @brief 选中动作
 */
void SelFunc(void)
{
    /*得到当前菜单页中的菜单项*/
    psCurrentMenuItem = psMenuItem;

    /*存在子菜单跳转*/
    if (psMenuItem->psSubMenu != NULL)
    {
        g_nMenuLevel++;
        g_MenuItemIndex = ItemNumb[g_nMenuLevel];
        /*得到当前菜单项的子菜单*/
        psCurrentMenu = psCurrentMenuItem->psSubMenu;
        psMenuItem    = &(psCurrentMenu->psItems)[g_MenuItemIndex];
        DisplayMenu();
        psPrevMenu[g_nMenuLevel] = psCurrentMenu;
    }
    /*执行菜单项函数*/
    psCurrentMenuItem->pfMenuFunc();
}

/**
 * @brief 返回动作
 */
void ReturnFunc(void)
{
    psMenuItem->pfUpDownMenuFunc();

    if (g_nMenuLevel == 0) g_nMenuLevel++;

    /*得到上一级菜单*/
    psCurrentMenu          = psPrevMenu[g_nMenuLevel - 1];
    psMenuItem             = &psCurrentMenu->psItems[0];
    ItemNumb[g_nMenuLevel] = 0;
    g_nMenuLevel--;
    g_MenuItemIndex = ItemNumb[g_nMenuLevel];

    DisplayMenu();
}

/**
 * @brief 向上
 */
void UpFunc(void)
{
    /*取消选中状态*/
    MENU_drawRectagnleFill(0, (g_MenuItemIndex + 1) * ITEM_H, ITEM_W, (g_MenuItemIndex + 2) * ITEM_H, BACK_COLOR);
    psMenuItem = &psCurrentMenu->psItems[g_MenuItemIndex];
    MENU_DisplayStringLine((g_MenuItemIndex + 1), psMenuItem->pszTitle);
    /*判断下标状态*/
    if (g_MenuItemIndex > 0)
        g_MenuItemIndex--;
    else
        g_MenuItemIndex = psCurrentMenu->nItems - 1;
    /*更新选中状态*/
    MENU_drawRectagnleFill(0, (g_MenuItemIndex + 1) * ITEM_H, ITEM_W, (g_MenuItemIndex + 2) * ITEM_H, SELECT_COLOR);
    psMenuItem = &psCurrentMenu->psItems[g_MenuItemIndex];
    MENU_DisplayStringLine((g_MenuItemIndex + 1), psMenuItem->pszTitle);
    /*被选中的菜单*/
    ItemNumb[g_nMenuLevel] = g_MenuItemIndex;
}

/**
 * @brief 向下
 */
void DownFunc(void)
{
    /*取消选中状态*/
    MENU_drawRectagnleFill(0, (g_MenuItemIndex + 1) * ITEM_H, ITEM_W, (g_MenuItemIndex + 2) * ITEM_H, BACK_COLOR);
    psMenuItem = &psCurrentMenu->psItems[g_MenuItemIndex];
    MENU_DisplayStringLine((g_MenuItemIndex + 1), psMenuItem->pszTitle);
    /*判断下标状态*/
    if (g_MenuItemIndex >= ((psCurrentMenu->nItems) - 1))
        g_MenuItemIndex = 0;
    else
        g_MenuItemIndex++;
    /*更新选中状态*/
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

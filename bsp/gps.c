/**
 * *****************************************************************************
 * @file        gps.c
 * @brief       华大北大tau1201 gps模块驱动
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-27
 * @copyright   cita
 * *****************************************************************************
 */

/*----------------------------------include-----------------------------------*/
#include "zf_common_headfile.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/
uint16 point_num = 0;

// uint8 gps_pit_state = 0;

int point_index = 0;   // 用于记录当前采集点的索引
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
#if Double_Record_FLAG
double Target_point[2][ALL_POINT_MAX];   // 用于储存采集的目标点信息，用于后续的位置计算(第一层是纬度，第二层是经度)
double Work_target_array
    [2]
    [ALL_POINT_MAX];   // 用于将从flash中读取的目标点数组的数据复制过来，当这个数组被赋值时，后续的一切和Flash再无关系

void collect_and_store_points()
{
    if (point_flag == 1)
    {
        point_flag = 0;
        if (point_index < ALL_POINT_MAX)
        {
            Target_point[0][point_index] = gnss.latitude;
            Target_point[1][point_index] = gnss.longitude;
            // printf("get a point");
            ips_show_string(10, 0, "point_num:0");
            point_index++;
            ips200_show_uint(90, 0, point_index, 2);
            if (point_index == ALL_POINT_MAX)
            {
                ips_show_string(10, 0, "write_to_flash_success!");
                printf("写入 Flash，共 %d 个点\n", ALL_POINT_MAX);
                // 将二维数组转为连续内存写入
                Flash_Write_Data(FLASH_SECTION_INDEX,
                                 GPS_PAGE_INDEX,
                                 (double*)Target_point,   // 直接转为一维数组指针
                                 ALL_POINT_MAX * 2,       // 总数据量（纬度+经度）
                                 FLASH_DOUBLE);
                point_index = 0;
            }
        }
    }
}

void read_points_from_flash()
{
    double recovered_data_double[2 * ALL_POINT_MAX];   // 每个点包含纬度和经度两个 double

    // 从 Flash 读取所有数据（总长度 2*ALL_POINT_MAX）
    Flash_Read_Data(FLASH_SECTION_INDEX, GPS_PAGE_INDEX, recovered_data_double, 2 * ALL_POINT_MAX, FLASH_DOUBLE);

    // 拆分数据到二维数组
    for (int i = 0; i < ALL_POINT_MAX; i++)
    {
        Target_point[0][i] = recovered_data_double[i];                   // 前 ALL_POINT_MAX 为纬度
        Target_point[1][i] = recovered_data_double[ALL_POINT_MAX + i];   // 后 ALL_POINT_MAX 为经度
    }
}

#endif
void gps_work_array()
{
    for (int NUM = 0; NUM < ALL_POINT_MAX; NUM++)   // 修改循环次数为完整点数
    {
        Work_target_array[0][NUM] = Target_point[0][NUM];
        Work_target_array[1][NUM] = Target_point[1][NUM];

        printf("\r\n组数-%d, 纬度:%.6lf, 经度:%.6lf", NUM + 1, Work_target_array[0][NUM], Work_target_array[1][NUM]);
        system_delay_ms(50);
    }
}

void gps_init()   // GPS参数初始化
{
    gnss_init(GPS_TYPE);                                    // GN42A 为GPS模块 GN43RFA 为RTK模块
    if (flash_check(FLASH_SECTION_INDEX, GPS_PAGE_INDEX))   // 判断Flash是否有数据 : 有数据返回1，无数据返回0
    {
        read_points_from_flash();   // 将GPS_FLASH的数据重新读回缓冲区并赋值给数组
        gps_work_array();           // 将目标数组的数据复制一份到工作目标数组(从这里之后GPS采点彻底结束,用以隔离FLASH)
        printf("GPS读取点位成功！\r\n");
    }
    else
    {
        printf("\r\nGPS为程序设定值(原始值)");
    }
}

void gps_show()   // GPS信息显示
{
    // 每个字符宽占8个格,长占16个格子
    ips_show_string(0, 16 * 0, "satl num:");
    ips_show_uint(90, 16 * 0, gnss.satellite_used, 1);
    ips_show_string(0, 16 * 1, "direction:");
    ips_show_float(90, 16 * 1, gnss.direction, 4, 6);
    ips_show_string(0, 16 * 2, "latitude:");
    ips_show_float(90, 16 * 2, gnss.latitude, 4, 6);
    ips_show_string(0, 16 * 3, "longitude:");
    ips_show_float(90, 16 * 3, gnss.longitude, 4, 6);
    ips_show_string(0, 16 * 4, "speed:");
    ips_show_float(90, 16 * 4, gnss.speed, 4, 6);
    ips_show_string(0, 16 * 5, "yaw:");
    // ips_show_float(90, 16 * 5, g_imu_angle.yaw, 4, 6);
    //  printf("Show yaw angle: %.6f\n", g_imu_angle.yaw);
}

void gps_proc()
{
    // gps_pit_state = 0;
    // timer_enable();
    disable_handler("gps");
    gnss_flag = 0;
    // 开始解析数据
    if (0 == gnss_data_parse())
    {
        // 解析成功
        // 以下是解析后的数据
        // gnss.time.year
        // gnss.time.month
        // gnss.time.day
        // gnss.time.hour
        // gnss.time.minute
        // gnss.time.second
        // gnss.state
        // gnss.latitude
        // gnss.longitude
        // gnss.speed
        // gnss.direction
        // gnss.height
        // gnss.satellite_used

#if GPS_WIFI_SEND_FLAG
        wifi_spi_send();
#endif   // GPS_WIFI_SEND_FLAG
        // gps_show();
    }
    else
    {
        // 解析失败
        zf_log(0, "gnss data parse error.");
    }
    // timer_disable();
}
/*------------------------------------test------------------------------------*/

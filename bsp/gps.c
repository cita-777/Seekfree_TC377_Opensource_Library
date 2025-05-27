/**
 * *****************************************************************************
 * @file        gps.c
 * @brief       ���󱱴�tau1201 gpsģ������
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

int point_index = 0;   // ���ڼ�¼��ǰ�ɼ��������
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
#if Double_Record_FLAG
double Target_point[2][ALL_POINT_MAX];   // ���ڴ���ɼ���Ŀ�����Ϣ�����ں�����λ�ü���(��һ����γ�ȣ��ڶ����Ǿ���)
double Work_target_array
    [2]
    [ALL_POINT_MAX];   // ���ڽ���flash�ж�ȡ��Ŀ�����������ݸ��ƹ�������������鱻��ֵʱ��������һ�к�Flash���޹�ϵ

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
                printf("д�� Flash���� %d ����\n", ALL_POINT_MAX);
                // ����ά����תΪ�����ڴ�д��
                Flash_Write_Data(FLASH_SECTION_INDEX,
                                 GPS_PAGE_INDEX,
                                 (double*)Target_point,   // ֱ��תΪһά����ָ��
                                 ALL_POINT_MAX * 2,       // ����������γ��+���ȣ�
                                 FLASH_DOUBLE);
                point_index = 0;
            }
        }
    }
}

void read_points_from_flash()
{
    double recovered_data_double[2 * ALL_POINT_MAX];   // ÿ�������γ�Ⱥ;������� double

    // �� Flash ��ȡ�������ݣ��ܳ��� 2*ALL_POINT_MAX��
    Flash_Read_Data(FLASH_SECTION_INDEX, GPS_PAGE_INDEX, recovered_data_double, 2 * ALL_POINT_MAX, FLASH_DOUBLE);

    // ������ݵ���ά����
    for (int i = 0; i < ALL_POINT_MAX; i++)
    {
        Target_point[0][i] = recovered_data_double[i];                   // ǰ ALL_POINT_MAX Ϊγ��
        Target_point[1][i] = recovered_data_double[ALL_POINT_MAX + i];   // �� ALL_POINT_MAX Ϊ����
    }
}

#endif
void gps_work_array()
{
    for (int NUM = 0; NUM < ALL_POINT_MAX; NUM++)   // �޸�ѭ������Ϊ��������
    {
        Work_target_array[0][NUM] = Target_point[0][NUM];
        Work_target_array[1][NUM] = Target_point[1][NUM];

        printf("\r\n����-%d, γ��:%.6lf, ����:%.6lf", NUM + 1, Work_target_array[0][NUM], Work_target_array[1][NUM]);
        system_delay_ms(50);
    }
}

void gps_init()   // GPS������ʼ��
{
    gnss_init(GPS_TYPE);                                    // GN42A ΪGPSģ�� GN43RFA ΪRTKģ��
    if (flash_check(FLASH_SECTION_INDEX, GPS_PAGE_INDEX))   // �ж�Flash�Ƿ������� : �����ݷ���1�������ݷ���0
    {
        read_points_from_flash();   // ��GPS_FLASH���������¶��ػ���������ֵ������
        gps_work_array();           // ��Ŀ����������ݸ���һ�ݵ�����Ŀ������(������֮��GPS�ɵ㳹�׽���,���Ը���FLASH)
        printf("GPS��ȡ��λ�ɹ���\r\n");
    }
    else
    {
        printf("\r\nGPSΪ�����趨ֵ(ԭʼֵ)");
    }
}

void gps_show()   // GPS��Ϣ��ʾ
{
    // ÿ���ַ���ռ8����,��ռ16������
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
    // ��ʼ��������
    if (0 == gnss_data_parse())
    {
        // �����ɹ�
        // �����ǽ����������
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
        // ����ʧ��
        zf_log(0, "gnss data parse error.");
    }
    // timer_disable();
}
/*------------------------------------test------------------------------------*/

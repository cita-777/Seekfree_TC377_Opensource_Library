/**
 * *****************************************************************************
 * @file        imu.c
 * @brief       �Ż���� IMU �� WIFI SPI ģ��ʵ��
 *              �ڲ����ݽṹIMU_Private_Data�����˾����˲�������ľ�ȷ�Ƕȣ�
 *              ��ԭʼ����̬���ݣ�pose����Ϊ˽�г�Ա���������С�
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-24
 * *****************************************************************************
 */

/*----------------------------------include-----------------------------------*/
#include "zf_common_headfile.h"

/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/
/**
 * @brief �ڲ�˽�����ݽṹ�����ڴ��IMU�ɼ����ݡ��˲������Լ������ĽǶ�
 */
typedef struct
{
    /* ԭʼ���������ݣ�������Ҫ������չ���棩 */
    float raw_acc[3];
    float raw_gyro[3];
    float raw_mag[3];

    /* ������ͨ�˲���Ĵ��������� */
    float acc_filtered[3];
    float gyro_filtered[3];
    float mag_filtered[3];

    /* �ڲ�˽�е���̬���ݣ�����ԭʼ�������ݣ�δ�����ģ� */
    Pose_Module pose;

    /* ������һ�������Ͳ�����ĽǶȣ����ⲿ��ѯ */
    float yaw;
    float pitch;
    float roll;
} IMU_Private_Data;
// ����ȫ�ֽǶ����ݽṹ
IMU_Angle g_imu_angle = {0};
/*----------------------------------variable----------------------------------*/
/* �ڲ�IMU���ݣ����ڵ�ǰ�ļ���ʹ�� */
IMU_Private_Data imu_private;

/* ���ڰ����ַ��ͨ�˲��Ļ���Ͳ��� */
static Butter_BufferData accel_filter_buf[3];
static Butter_BufferData gyro_filter_buf[3];
static Butter_Parameter  Accel_Parameter;
static Butter_Parameter  Gyro_Parameter;

/* RLS�˲���״̬������У��������ƫ�ã����ֱ��Ӧ������ */
static RLSState rls_acc[3] = {{0.0f, 1000.0f, 0.98f}, {0.0f, 1000.0f, 0.98f}, {0.0f, 1000.0f, 0.98f}};

static RLSState rls_gyro[3] = {{0.0f, 1000.0f, 0.98f}, {0.0f, 1000.0f, 0.98f}, {0.0f, 1000.0f, 0.98f}};

/* �ڲ�ƫ�ã�����õ��������������������� */
float offset_acce[3] = {0.0f, 0.0f, 0.0f};
float offset_gyro[3] = {0.0f, 0.0f, 0.0f};

/* �ڲ����������������Ư������ */
// static uint32 x_counter  = 0;
// static double zero_error = 0.0;

/* ���Ʋ�����־����У׼�׶�ʹ�ã� */
uint8 offset_flag = 0;
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/

/**
 * @brief ��ʼ��IMU�豸���˲������ڲ����ݽṹ
 */
void imu_init(void)
{
    // ��ʼ��IMU�豸
#if IMU_USE_963RA
    if (imu963ra_init())
    {
        printf("\r\nIMU963RA init error.");
    }
#endif   // IMU_USE_963RA
#if IMU_USE_ICM45686
    if (icm45686_init(1, 1, 1))
    {
        printf("\r\nICM45686 init error.");
    }
    // ����㹻�����ȶ���ʱ
    system_delay_ms(100);   // ���ӳ�ʼ�������ʱ

#endif   // IMU_USE_ICM45686

    // ���õ�ͨ�˲�����ֹƵ��
    Set_Cutoff_Frequency(1000, 200, &Accel_Parameter);
    Set_Cutoff_Frequency(1000, 100, &Gyro_Parameter);

    /* У׼�׶Σ��ɼ�������ݸ��´�����ƫ�� */
    int sample_count = 100;
    int i            = 0;
    while (i < sample_count)
    {
        if (offset_flag == 1)
        {   // ��������־����ʱ
            offset_flag = 0;
/* ��ȡIMUԭʼ���� */
#if IMU_USE_963RA
            imu963ra_get_acc();
            imu963ra_get_gyro();
            imu963ra_get_mag();

            // ��ȡԭʼ���ݣ�����������ת���ӿڣ�
            float meas_acc[3];
            meas_acc[0] =
                LPButterworth(imu963ra_acc_transition(imu963ra_acc_x), &accel_filter_buf[0], &Accel_Parameter);
            meas_acc[1] =
                LPButterworth(imu963ra_acc_transition(imu963ra_acc_y), &accel_filter_buf[1], &Accel_Parameter);
            meas_acc[2] =
                LPButterworth(imu963ra_acc_transition(imu963ra_acc_z), &accel_filter_buf[2], &Accel_Parameter);

            float meas_gyro[3];
            meas_gyro[0] =
                LPButterworth(imu963ra_gyro_transition(imu963ra_gyro_x), &gyro_filter_buf[0], &Gyro_Parameter);
            meas_gyro[1] =
                LPButterworth(imu963ra_gyro_transition(imu963ra_gyro_y), &gyro_filter_buf[1], &Gyro_Parameter);
            meas_gyro[2] =
                LPButterworth(imu963ra_gyro_transition(imu963ra_gyro_z), &gyro_filter_buf[2], &Gyro_Parameter);
#endif   // IMU_USE_963RA
#if IMU_USE_ICM45686
            // ��ȡ���ݺ����LPButeerworth����bsp_IcmGetRawData
            float meas_acc[3];
            float meas_gyro[3];
            float temp_degc;
            bsp_IcmGetRawData(meas_acc, meas_gyro, &temp_degc);

            // ʹ��LPButterworth�Լ��ٶ����ݽ����˲�����
            meas_acc[0] = LPButterworth(meas_acc[0], &accel_filter_buf[0], &Accel_Parameter);
            meas_acc[1] = LPButterworth(meas_acc[1], &accel_filter_buf[1], &Accel_Parameter);
            meas_acc[2] = LPButterworth(meas_acc[2], &accel_filter_buf[2], &Accel_Parameter);

            // ʹ��LPButterworth�����������ݽ����˲�����
            meas_gyro[0] = LPButterworth(meas_gyro[0], &gyro_filter_buf[0], &Gyro_Parameter);
            meas_gyro[1] = LPButterworth(meas_gyro[1], &gyro_filter_buf[1], &Gyro_Parameter);
            meas_gyro[2] = LPButterworth(meas_gyro[2], &gyro_filter_buf[2], &Gyro_Parameter);
#endif   // IMU_USE_ICM45686
         // ��RLS�㷨����ƫ��
            for (int j = 0; j < 3; j++)
            {
                RLS_Update(&rls_acc[j], meas_acc[j]);
                RLS_Update(&rls_gyro[j], meas_gyro[j]);
            }
            i++;
        }
    }

    // ���������󣬱������չ��Ƶ�ƫ��
    for (int j = 0; j < 3; j++)
    {
        offset_acce[j] = rls_acc[j].bias;
        offset_gyro[j] = rls_gyro[j].bias;
    }

    // ��ʼ���ڲ�˽�е�pose�ṹ
    initPose_Module(&imu_private.pose);

    // ��ʼ���ڲ��Ƕ�����Ϊ0
    imu_private.yaw = imu_private.pitch = imu_private.roll = 0.0f;
}

/**
 * @brief ͨ��madgwick��mahony�㷨����IMU���ݣ��ɼ����ݡ��˲���λ�˽��㲢�����ⲿ��ѯ�Ƕ�
 */
void imu_proc(void)
{
    // timer_enable();
#if IMU_WIFI_GET_PARAM_FLAG
    // �������λ�����Σ������λ�˽���������˴�ֱ�Ӵ��ݲ��������еĲ������ݣ�
    imu_private.pose.parameter.error_ki   = seekfree_assistant_parameter[5];
    imu_private.pose.parameter.error_kp   = seekfree_assistant_parameter[6];
    imu_private.pose.parameter.correct_kp = seekfree_assistant_parameter[7];
#endif

    /* ��ȡIMUԭʼ���� */
#if IMU_USE_963RA
    imu963ra_get_acc();
    imu963ra_get_gyro();
    imu963ra_get_mag();
    /* ����ת����ƫ��������ͬʱ����ԭʼ���ݣ�������Ҫ�� */
    imu_private.raw_acc[0] = imu963ra_acc_transition(imu963ra_acc_x);
    imu_private.raw_acc[1] = imu963ra_acc_transition(imu963ra_acc_y);
    imu_private.raw_acc[2] = imu963ra_acc_transition(imu963ra_acc_z);

    imu_private.raw_gyro[0] = imu963ra_gyro_transition(imu963ra_gyro_x);
    imu_private.raw_gyro[1] = imu963ra_gyro_transition(imu963ra_gyro_y);
    imu_private.raw_gyro[2] = imu963ra_gyro_transition(imu963ra_gyro_z);

    imu_private.raw_mag[0] = imu963ra_mag_transition(imu963ra_mag_x);
    imu_private.raw_mag[1] = imu963ra_mag_transition(imu963ra_mag_y);
    imu_private.raw_mag[2] = imu963ra_mag_transition(imu963ra_mag_z);
#endif   // IMU_USE_963RA
#if IMU_USE_ICM45686
    float temp_degc;
    bsp_IcmGetRawData(imu_private.raw_acc, imu_private.raw_gyro, &temp_degc);
#endif   // IMU_USE_ICM45686
    /* ��λת����ƫ������
       ���ٶȣ�g -> cm/s2��1gԼ=980.665 cm/s2��
       �����ǣ���λ����Ϊ ��/s
       ��ע���ɸ���Ӳ����װ�������� */
    float acc[3];
    acc[0] = -(imu_private.raw_acc[0] - offset_acce[0]) * 980.665f;
    acc[1] = (imu_private.raw_acc[1] - offset_acce[1]) * 980.665f;
    acc[2] = (imu_private.raw_acc[2] - offset_acce[2] + 1.0f) * 980.665f;

    // float acc[3];
    // acc[0] = -(imu_private.raw_acc[0] - offset_acce[0]);
    // acc[1] = (imu_private.raw_acc[1] - offset_acce[1]);
    // acc[2] = (imu_private.raw_acc[2] - offset_acce[2] + 1.0f);

    float gyro[3];
    gyro[0] = -(imu_private.raw_gyro[0] - offset_gyro[0]);
    gyro[1] = (imu_private.raw_gyro[1] - offset_gyro[1]);
    gyro[2] = (imu_private.raw_gyro[2] - offset_gyro[2]);

    float mag[3];
    mag[0] = -imu_private.raw_mag[0];
    mag[1] = imu_private.raw_mag[1];
    mag[2] = imu_private.raw_mag[2];

    /* �Լ��ٶȺ����������ݽ��е�ͨ�˲� */
    for (int j = 0; j < 3; j++)
    {
        imu_private.acc_filtered[j]  = LPButterworth(acc[j], &accel_filter_buf[j], &Accel_Parameter);
        imu_private.gyro_filtered[j] = LPButterworth(gyro[j], &gyro_filter_buf[j], &Gyro_Parameter);
    }
    /* ��������������Ҫ�˲��������ƴ����˴�ֱ�Ӹ�ֵ */
    for (int j = 0; j < 3; j++)
    {
        imu_private.mag_filtered[j] = mag[j];
    }

    /* ���˲�������ݴ��ݸ��ڲ�˽�е�pose�ṹ
       ע�⣺������Ķ�Ӧ��ϵ����ݾ��尲װ������� */
    imu_private.pose.interface.data.a_x = &imu_private.acc_filtered[1];
    imu_private.pose.interface.data.g_x = &imu_private.gyro_filtered[1];
    imu_private.pose.interface.data.m_x = &imu_private.mag_filtered[1];
    imu_private.pose.interface.data.a_y = &imu_private.acc_filtered[0];
    imu_private.pose.interface.data.g_y = &imu_private.gyro_filtered[0];
    imu_private.pose.interface.data.m_y = &imu_private.mag_filtered[0];
    imu_private.pose.interface.data.a_z = &imu_private.acc_filtered[2];
    imu_private.pose.interface.data.g_z = &imu_private.gyro_filtered[2];
    imu_private.pose.interface.data.m_z = &imu_private.mag_filtered[2];

    /* ����λ�˽��㺯��������Mahony�㷨���������ڲ�pose���� */
    calculatePose_Module(&imu_private.pose, 0.001f);

    /* ������Ҫ�������Ư������ */
    // x_counter++;
    // zero_error = 4.68411239e-09 * Power2(x_counter) - 8.86316947e-04 * x_counter
    // + 2.26171105e-02;
    //  �˴�����yaw�����������磺imu_private.yaw = imu_private.pose.data.yaw - zero_error;
    //  ʾ��:imu_private.yaw   = imu_private.pose.data.yaw - zero_error;
    imu_private.yaw   = imu_private.pose.data.yaw;
    imu_private.pitch = imu_private.pose.data.pit;
    imu_private.roll  = imu_private.pose.data.rol;

    g_imu_angle.yaw   = imu_private.pose.data.yaw;
    g_imu_angle.pitch = imu_private.pose.data.pit;
    g_imu_angle.roll  = imu_private.pose.data.rol;
    // static uint32_t display_counter = 0;
    // if (++display_counter >= 1)   // ÿ50ms��ʾһ��(Լ20Hz)
    // {
    //     display_counter = 0;

    //     // IPS��ʾ����
    //     ips_show_string(0, 0, "Pose:");
    //     ips_show_float(90, 0, imu_private.pose.data.yaw, 2, 6);

    //     ips_show_string(0, 16, "IMU:");
    //     ips_show_float(90, 16, imu_private.yaw, 2, 6);

    //     ips_show_string(0, 32, "Angle:");
    //     ips_show_float(90, 32, g_imu_angle.yaw, 2, 6);

    //     // // ʹ��sprintf����ַ���,һ�������
    //     // char debug_str[100];
    //     // sprintf(debug_str,
    //     //         "\r\nDebug - [%u] Pose:%.2f IMU:%.2f Angle:%.2f",
    //     //         display_counter,
    //     //         imu_private.pose.data.yaw,
    //     //         imu_private.yaw,
    //     //         g_imu_angle.yaw);
    //     // printf("%s", debug_str);
    // }

    // // ��ӵ����������֤��ֵ
    // printf("\r\nDebug - Pose:%.2f IMU:%.2f Angle:%.2f", imu_private.pose.data.yaw, imu_private.yaw, g_imu_angle.yaw);
#if IMU_WIFI_SEND_FLAG
    wifi_spi_send();
#endif
    disable_handler("imu");
    // timer_disable();
}

/**
 * @brief ���ⲿ��ѯ�����˲���������ĽǶ�����
 */
IMU_Angle* IMU_GetAngle(void)
{
    return &g_imu_angle;
}

/**
 * @brief ��ʼ��WIFIģ�鼰��������
 */
void wifi_init(void)
{
    while (wifi_spi_init(MY_WIFI_SSID, MY_WIFI_PASSWORD))
    {
        printf("\r\n connect wifi failed. \r\n");
        system_delay_ms(100);
    }

    printf("\r\n module version:%s", wifi_spi_version);
    printf("\r\n module mac    :%s", wifi_spi_mac_addr);
    printf("\r\n module ip     :%s", wifi_spi_ip_addr_port);

    if (1 != WIFI_SPI_AUTO_CONNECT)
    {
        while (wifi_spi_socket_connect("TCP", MY_TCP_TARGET_IP, MY_TCP_TARGET_PORT, MY_WIFI_LOCAL_PORT))
        {
            printf("\r\n Connect TCP Servers error, try again.");
            system_delay_ms(100);
        }
    }
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
}

/**
 * @brief ͨ��WIFI�������ݣ�ʾ���з��Ͳ��ֽǶ��봫�������ݣ�
 */
void wifi_spi_send(void)
{
#if IMU_WIFI_SEND_FLAG
    // ����ʾ�������ڲ��˲���ĽǶȺͲ��ִ���������
    seekfree_assistant_oscilloscope_data.data[0]     = imu_private.acc_filtered[0];
    seekfree_assistant_oscilloscope_data.data[1]     = imu_private.acc_filtered[1];
    seekfree_assistant_oscilloscope_data.data[2]     = imu_private.acc_filtered[2];
    seekfree_assistant_oscilloscope_data.data[3]     = imu_private.gyro_filtered[0];
    seekfree_assistant_oscilloscope_data.data[4]     = imu_private.gyro_filtered[1];
    seekfree_assistant_oscilloscope_data.data[5]     = imu_private.gyro_filtered[2];
    seekfree_assistant_oscilloscope_data.data[6]     = imu_private.yaw;
    seekfree_assistant_oscilloscope_data.data[7]     = imu_private.roll;
    seekfree_assistant_oscilloscope_data.channel_num = 8;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);

    seekfree_assistant_data_analysis();
#endif   // IMU_WIFI_SEND_FLAG
#if MOTOR_ENCODER_WIFI_SEND_FLAG
    seekfree_assistant_oscilloscope_data.data[0] = encoder_data_1;
    seekfree_assistant_oscilloscope_data.data[1] = cumulative_encoder_data_1;
    seekfree_assistant_oscilloscope_data.data[2] = motor_duty;
    // seekfree_assistant_oscilloscope_data.data[3] = ;
    // seekfree_assistant_oscilloscope_data.data[4] = ;
    // seekfree_assistant_oscilloscope_data.data[5] = 100;
    // seekfree_assistant_oscilloscope_data.data[6] = 1000;
    // seekfree_assistant_oscilloscope_data.data[7] = 10000;
    //  ���ñ�����Ҫ���ͼ���ͨ��������
    seekfree_assistant_oscilloscope_data.channel_num = 3;

    // �����������6��ͨ�������ݣ����֧��8ͨ��
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
    // �п��ܻ��������������Ͽ������θ��²���������������Ϊʹ��WIFI�в�ȷ�����ӳٵ��µ�

    // ������λ�����͹����Ĳ���
    seekfree_assistant_data_analysis();
#endif   // ENCODER_WIFI_SEND_FLAG
#if GPS_WIFI_SEND_FLAG
    seekfree_assistant_oscilloscope_data.data[0] = gnss.latitude;
    seekfree_assistant_oscilloscope_data.data[1] = gnss.longitude;
    seekfree_assistant_oscilloscope_data.data[2] = gnss.direction;
    seekfree_assistant_oscilloscope_data.data[3] = gnss.speed;
    seekfree_assistant_oscilloscope_data.data[4] = gnss.satellite_used;
    seekfree_assistant_oscilloscope_data.data[5] = yaw;
    // seekfree_assistant_oscilloscope_data.data[6] = 1000;
    // seekfree_assistant_oscilloscope_data.data[7] = 10000;
    //  ���ñ�����Ҫ���ͼ���ͨ��������
    seekfree_assistant_oscilloscope_data.channel_num = 6;

    // �����������6��ͨ�������ݣ����֧��8ͨ��
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
    // �п��ܻ��������������Ͽ������θ��²���������������Ϊʹ��WIFI�в�ȷ�����ӳٵ��µ�

    // ������λ�����͹����Ĳ���
    seekfree_assistant_data_analysis();
#endif   // GPS_WIFI_SEND_FLAG
}

/**
 * @brief ����ר�ã��жϷ�����������ԭʼIMU���ݵ����Թ���
 */
void imu_oscilloscope_isr(void)
{
    imu963ra_get_acc();
    imu963ra_get_gyro();
    imu963ra_get_mag();

    /* ֱ�ӷ���ԭʼ���ݹ����� */
    seekfree_assistant_oscilloscope_data.data[0]     = imu963ra_acc_x;
    seekfree_assistant_oscilloscope_data.data[1]     = imu963ra_acc_y;
    seekfree_assistant_oscilloscope_data.data[2]     = imu963ra_acc_z;
    seekfree_assistant_oscilloscope_data.data[3]     = imu963ra_gyro_x;
    seekfree_assistant_oscilloscope_data.data[4]     = imu963ra_gyro_y;
    seekfree_assistant_oscilloscope_data.data[5]     = imu963ra_gyro_z;
    seekfree_assistant_oscilloscope_data.channel_num = 6;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
    seekfree_assistant_data_analysis();

    disable_handler("imu_oscilloscope");
}
/**
 * @brief ����ר�ã��жϷ�����������ԭʼIMU���ݵ����Թ���
 */
void icm45686_oscilloscope_isr(void)
{
    float meas_acc[3];
    float meas_gyro[3];
    float temp_degc;
    bsp_IcmGetRawData(meas_acc, meas_gyro, &temp_degc);

    /* ֱ�ӷ���ԭʼ���ݹ����� */
    seekfree_assistant_oscilloscope_data.data[0]     = meas_acc[0];
    seekfree_assistant_oscilloscope_data.data[1]     = meas_acc[1];
    seekfree_assistant_oscilloscope_data.data[2]     = meas_acc[2];
    seekfree_assistant_oscilloscope_data.data[3]     = meas_gyro[0];
    seekfree_assistant_oscilloscope_data.data[4]     = meas_gyro[1];
    seekfree_assistant_oscilloscope_data.data[5]     = meas_gyro[2];
    seekfree_assistant_oscilloscope_data.data[5]     = temp_degc;
    seekfree_assistant_oscilloscope_data.channel_num = 7;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
    seekfree_assistant_data_analysis();

    disable_handler("imu_oscilloscope");
}

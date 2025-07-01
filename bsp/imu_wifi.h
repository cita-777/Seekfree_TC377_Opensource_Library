/**
 * *****************************************************************************
 * @file        imu_wifi.h
 * @brief       IMU_Angle���ⲿ��ѯʹ��
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-24
 * *****************************************************************************
 */

/**
 *  ��ʽ1��ֱ�ӷ���
    float current_yaw = g_imu_angle.yaw;

    ��ʽ2��ͨ�������ӿ�
    IMU_Angle* angle = IMU_GetAngle();
    float current_pitch = angle->pitch;
 */
#ifndef IMU_WIFI_H
#define IMU_WIFI_H

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------include-----------------------------------*/

/*-----------------------------------macro------------------------------------*/
#define IMU_USE_963RA (0)      // ����ʹ�� 963RA IMU
#define IMU_USE_ICM45686 (1)   // ����ʹ�� ICM45686 IMU

// ѡ��WiFi����
#define WIFI_CONFIG 3   // 1: CMCC-nAHP, 2: cita, 3: NEWX

#if WIFI_CONFIG == 1
#    define MY_WIFI_SSID "CMCC-nAHP"
#    define MY_WIFI_PASSWORD "18065890955"
#    define MY_TCP_TARGET_IP "192.168.1.5"   // ����Ŀ��� IP
#elif WIFI_CONFIG == 2
#    define MY_WIFI_SSID "cita"
#    define MY_WIFI_PASSWORD "88888888"
#    define MY_TCP_TARGET_IP "192.168.*.*"
#elif WIFI_CONFIG == 3
#    define MY_WIFI_SSID "NEWX"
#    define MY_WIFI_PASSWORD "dxswlwhgzs"
#    define MY_TCP_TARGET_IP "192.168.5.5"   // ����Ŀ��� IP
#elif WIFI_CONFIG == 4
#    define MY_WIFI_SSID "xingyao14"
#    define MY_WIFI_PASSWORD "cjk20158"
#    define MY_TCP_TARGET_IP "192.168.137.1"   // ����Ŀ��� IP
#else
#    error "Invalid WIFI_CONFIG value. Must be 1, 2, or 3."
#endif

#define MY_TCP_TARGET_PORT "8086"   // ����Ŀ��Ķ˿�
#define MY_WIFI_LOCAL_PORT "6666"   // �����Ķ˿� 0�����  �����÷�Χ2048-65535  Ĭ�� 6666

#define IMU_PIT (CCU60_CH0)

/*----------------------------------typedef-----------------------------------*/
/**
 * @brief �����ⲿ��ѯ�ľ�ȷ�Ƕ�����
 */
typedef struct
{
    float yaw;     // ����ǣ���λ�㣨�����˲���������[-180, 180]
    float pitch;   // �����ǣ���λ��
    float roll;    // ����ǣ���λ��

    // ��չ�Ƕȱ�����֧��-���+���Χ
    float yaw_extended;     // ��չ����ǣ����Գ�����180�㷶Χ
    float pitch_extended;   // ��չ������
    float roll_extended;    // ��չ�����
} IMU_Angle;
// �����ⲿ�ɷ��ʵĽǶ�����
extern IMU_Angle g_imu_angle;
/*----------------------------------variable----------------------------------*/
extern uint8 offset_flag;
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
/**
 * @brief ��ʼ��IMU�豸���˲������ڲ����ݽṹ
 */
void imu_init(void);

/**
 * @brief ͨ��Madgwick��Mahony�㷨����IMU���ݣ��ɼ����˲���λ�˼��㣩
 */
void imu_proc(void);

/**
 * @brief ��ѯ�����˲���������ĽǶ�����
 */
IMU_Angle* IMU_GetAngle(void);

/**
 * @brief ��ʼ��WIFIģ�鼰��������
 */
void wifi_init(void);

/**
 * @brief ͨ��WIFI��������
 */
void wifi_spi_send(void);

/**
 * @brief ����ר�ã��жϷ�����������ԭʼ���ݵ����Թ���
 */
void imu_oscilloscope_isr(void);
void icm45686_oscilloscope_isr(void);
/*------------------------------------test------------------------------------*/
/**
 * @brief ��ȡ��չ�Ƕ�ֵ�������Ʒ�Χ��
 */
float IMU_GetExtendedYaw(void);

/**
 * @brief ������չ�Ƕ��ۻ�ֵ
 */
void IMU_ResetExtendedAngles(void);

/**
 * @brief ������չ�Ƕȵĳ�ʼֵ
 */
void IMU_SetExtendedYaw(float yaw);

#ifdef __cplusplus
}
#endif

#endif /* IMU_WIFI_H */

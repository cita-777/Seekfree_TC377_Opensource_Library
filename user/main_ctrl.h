/**
 * *****************************************************************************
 * @file        main_ctrl.h
 * @brief
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-26
 * @copyright   cita
 * *****************************************************************************
 */
#ifndef MAIN_CTRL_H
#define MAIN_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------include-----------------------------------*/

/*-----------------------------------macro------------------------------------*/
#define Main_FLAG (0)   // ����������־λ

#define MENU_INIT_FLAG (1)    // �˵���ʼ����־λ
#define SBUS_INIT_FLAG (1)    // SBUS��ʼ����־λ
#define IMU_INIT_FLAG (1)     // IMU��ʼ����־λ
#define FLASH_INIT_FLAG (1)   // FLASH���ݳ�ʼ����־λ
#define GPS_INIT_FLAG (1)     // GPS��ʼ����־λ
#define KEY_INIT_FLAG (1)     // ������ʼ����־λ
#define SERVO_INIT_FLAG (1)   // �����ʼ����־λ
#define MOTOR_INIT_FLAG (1)   // �����ʼ����־λ
#define MENU_INIT_FLAG (1)    // �˵���ʼ����־λ

#define IMU_WIFI_SEND_FLAG (0)             // IMUʹ��WIFI���ͱ�־λ
#define MOTOR_ENCODER_WIFI_SEND_FLAG (0)   // ������ʹ��WIFI���ͱ�־λ
#define GPS_WIFI_SEND_FLAG (0)             // GPSʹ��WIFI���ͱ�־λ
#define PID_WIFI_SEND_FLAG (1)             // PIDʹ��WIFI���ͱ�־λ
#define PD_WIFI_SEND_FLAG (0)              // PDʹ��WIFI���ͱ�־λ


#define IMU_WIFI_GET_PARAM_FLAG (0)   // IMUʹ��WIFI��ȡ��λ��������־λ
// #define SBUS_PIT_START_FLAG (1)     //SBUS��ʱ��������־λ
#define IMU_PIT_START_FLAG (1)   // IMU�жϿ�����־λ

#define ENCODER_PIT_START_FLAG (1)   // ��������ʱ��������־λ
#define SERVO_PIT_START_FLAG (1)     // �����ʱ��������־λ
// #define GPS_PIT_START_FLAG (1)       // GPS��ʱ��������־λ

#define GL_GPS_MATH_FLAG (1)        // GPS_Doubleר����ѧ�⿪����־λ
#define DEFAULT_GPS_MATH_FLAG (0)   // ˫����ģʽ������־λ(GPS��ѧ����������Ϊ˫����)
#define Double_Record_FLAG (1)      // ˫���ȸ���洢������־λ//����
/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
// ��������
// Ϊ1,���Ա���||Ϊ0,��ֹ����

// ������־λ-˵����
// 1.�ж�ADS�ǵ�����ģʽ����˫����ģʽ(������ģʽ�£�float��double������float����)
// 2.�жϲ��ú�����ѧ��:Float_FLAG-GPS���õ�������ѧ��(ADS�Դ���)||Double_FLAG-GPS����˫������ѧ��(�Զ����)||Int_FLAG_D-GPS���������ββ�����Ĭ����ѧ��
// 3.�жϲŲ��ú���FLASH�洢��ʽ:Float_Record_FLAG-������||Double_Record_FLAG-˫����||Int_Record_FLAG-�����洢

// �̶�ģʽ��
// Float_FLAG  (1)---------Float_Record_FLAG  (1)//������
// Double_FLAG (1)---------Double_Record_FLAG (1)//˫����
// Int_FLAG_D  (1)---------Int_Record_FLAG    (1)//����

/*
����͵����һ����ʱ��--7ms

ռ�ձ�         ����
0              0
1000           50
2000           100
3000           140
4000           170
5000           190
6000           210
7000           220
8000           230
9000           240
10000          250
 */
/*------------------------------------test------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* MAIN_CTRL_H */

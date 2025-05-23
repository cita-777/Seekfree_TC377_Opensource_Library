/**
 * *****************************************************************************
 * @file        pose.h
 * @brief       Mahony����ṹ��
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-25
 * *****************************************************************************
 */

#ifndef POSE_H
#define POSE_H

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------include-----------------------------------*/

/*-----------------------------------macro------------------------------------*/
#ifndef u8
#    define u8 unsigned char
#endif

#ifndef s8
#    define s8 char
#endif

/**
 * @brief ��ά����
 * @details ���ڴ洢��ά���ݣ�x, y, z����֧�ֲ�ͬ���ȣ����㡢���ͣ�
 */
#ifndef XYZ_Data
#    define XYZ_Data

typedef struct XYZ_Data_f
{
    float x;
    float y;
    float z;
} XYZ_Data_f;

typedef struct XYZ_Data_s32
{
    long x;
    long y;
    long z;
} XYZ_Data_s32;

typedef struct XYZ_Data_s16
{
    short x;
    short y;
    short z;
} XYZ_Data_s16;

typedef struct XYZ_Data_s8
{
    s8 x;
    s8 y;
    s8 z;
} XYZ_Data_s8;
#endif
/*----------------------------------typedef-----------------------------------*/

/**
 * @brief ��̬��־λ
 * @details ��־λ������ģ���Ƿ����У�run�����Ƿ�ʹ�ô����ƣ�use_mag����
 */
typedef struct Pose_Flag
{
    u8 run;
    u8 use_mag;
    u8 algorithm;
} Pose_Flag;

/**
 * @brief ��̬���ݽӿ�
 * @details ���������ݽӿڣ�ָ����ٶȡ������ǡ������Ƶ�ԭʼ���ݡ�
 */
typedef struct Pose_DInterface
{
    float *a_x;
    float *a_y;
    float *a_z;

    float *g_x;
    float *g_y;
    float *g_z;

    float *m_x;
    float *m_y;
    float *m_z;
} Pose_DInterface;

/**
 * @brief ��̬���ݽӿ�
 * @details ��װPose_DInterface��һ����������̬ģ������ݴ��ݡ�
 */
typedef struct Pose_Interface
{
    Pose_DInterface data;
} Pose_Interface;

/**
 * @brief ��̬����
 * @details �洢��������̬���ݣ�����ŷ���ǡ���ת����У����Ĵ��������ݵȡ�
 */
typedef struct Pose_Data
{
    float yaw;
    float rol;
    float pit;

    float rotate_matrix[3][3];   // ��ת����

    XYZ_Data_f acc_world;   // ��������ϵ�µļ��ٶ�
    XYZ_Data_f mag_world;   // ��������ϵ�µĴų�ǿ��   --  ֻ�����˻�λ���йص���

    XYZ_Data_f acc_correct;    // ��������ϵ�µļ��ٶ�    --  �����˸���������ļ��ٶ�
    XYZ_Data_f mag_correct;    // ��������ϵ�µĴų�ǿ��   --  �����˸���������Ĵų�ǿ��
    XYZ_Data_f gyro_correct;   // �ںϼ��ٶȺʹ��������ݣ��������������ֵ
} Pose_Data;

/**
 * @brief ��̬����
 * @details �м�������������Ԫ���������֡�������ƫ���ǡ�
 */
typedef struct Pose_Process
{
    float      mag_yaw;          // �����Ƶ�ƫ����
    float      mag_yaw_bias;     // �����ƽ�����ƫ����ƫ��
    float      quaternion[4];    // ��Ԫ��
    XYZ_Data_f error;            // �ɼ��ٶȼ����Ч������ƫ��
    XYZ_Data_f error_integral;   // ������
} Pose_Process;

/**
 * @brief ��̬����
 * @details �㷨�����������ı�����kp���ͻ��֣�ki��ϵ����
 */
typedef struct Pose_Parameter
{
    float correct_kp;
    float error_kp;
    float error_ki;

    float beta;
} Pose_Parameter;

/**
 * @brief ��̬ģ��
 * @details �������нṹ�壬����̬����ĺ���ģ�顣
 */
typedef struct Pose_Module
{
    Pose_Flag      flag;
    Pose_Interface interface;
    Pose_Process   process;
    Pose_Data      data;
    Pose_Parameter parameter;
} Pose_Module;
/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
/**
 * @brief      ��ʼ����̬ģ��
 * @details    ��ʼ����̬ģ�飬���ó�ʼ����������Ԫ����ʼ��Ϊ��λ��Ԫ�������������ݡ�
 * @param       pose
 */
void initPose_Module(Pose_Module *pose);
/**
 * @brief      ��̬����
 * @details    ��̬���㣬ͨ����Ԫ�����㣬������̬�Ƕȡ���ת����У����Ĵ��������ݡ�
 * @param       pose
 * @param       cycle
 */
void calculatePose_Module(Pose_Module *pose, float cycle);
/*------------------------------------test------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* POSE_H */

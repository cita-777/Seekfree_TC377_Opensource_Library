/**
 * *****************************************************************************
 * @file        pose.c
 * @brief       Mahony����ṹ��
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-25
 * *****************************************************************************
 */
#include "zf_common_headfile.h"
// ��̬��������
void MadgwickIMUUpdate(Pose_Module *pose, float cycle, float gx, float gy, float gz, float ax, float ay, float az);
const float RAD2DEG = 180.0f / MY_PI;
//@����           ��tou_zi
//@��дʱ�� ��2019��4��6��
//@�޸�ʱ�� ��2019��4��6��
//@������      ��initPose_Module
//@�������� ����ʼ����̬����ģ��
//@����1      ��*pose ��̬����ṹ��ָ��
//@����ֵ      ����
void initPose_Module(Pose_Module *pose)
{
    // ��־λ��ʼ��
    pose->flag.run     = 1;   // ��������
    pose->flag.use_mag = 0;   // ʹ�õ�������
    // �ӿڳ�ʼ��
    pose->interface.data.a_x = 0;
    pose->interface.data.a_y = 0;
    pose->interface.data.a_z = 0;
    pose->interface.data.g_x = 0;
    pose->interface.data.g_y = 0;
    pose->interface.data.g_z = 0;
    pose->interface.data.m_x = 0;
    pose->interface.data.m_y = 0;
    pose->interface.data.m_z = 0;
    // ������ʼ��
    pose->parameter.error_ki   = 0.00f;
    pose->parameter.error_kp   = 0.35f;
    pose->parameter.correct_kp = 0.6f;
    // pose->parameter.error_kp = 0.0f;
    // pose->parameter.correct_kp = 0.0f;
    // �м�������
    pose->process.error.x          = 0;
    pose->process.error.y          = 0;
    pose->process.error.z          = 0;
    pose->process.error_integral.x = 0;
    pose->process.error_integral.y = 0;
    pose->process.error_integral.z = 0;

    pose->process.quaternion[0] = 1;
    pose->process.quaternion[1] = 0;
    pose->process.quaternion[2] = 0;
    pose->process.quaternion[3] = 0;
    // ���ݳ�ʼ��
    pose->data.rotate_matrix[0][0] = 0;
    pose->data.rotate_matrix[0][1] = 0;
    pose->data.rotate_matrix[0][2] = 0;
    pose->data.rotate_matrix[1][0] = 0;
    pose->data.rotate_matrix[1][1] = 0;
    pose->data.rotate_matrix[1][2] = 0;
    pose->data.rotate_matrix[2][0] = 0;
    pose->data.rotate_matrix[2][1] = 0;
    pose->data.rotate_matrix[2][2] = 0;

    pose->data.mag_world.x = 0;
    pose->data.mag_world.y = 0;
    pose->data.mag_world.z = 0;

    pose->data.acc_world.x = 0;
    pose->data.acc_world.y = 0;
    pose->data.acc_world.z = 0;

    pose->data.mag_correct.x = 0;
    pose->data.mag_correct.y = 0;
    pose->data.mag_correct.z = 0;

    pose->data.acc_correct.x = 0;
    pose->data.acc_correct.y = 0;
    pose->data.acc_correct.z = 0;

    pose->data.gyro_correct.x = 0;
    pose->data.gyro_correct.y = 0;
    pose->data.gyro_correct.z = 0;

    pose->data.pit = 0;
    pose->data.rol = 0;
    pose->data.yaw = 0;

    // ��ʼ��madgwick�㷨����
    pose->parameter.beta = 0.00001f;   // Madgwick�˲�������ϵ��
    pose->flag.algorithm = 1;          // Ĭ��ʹ��Madgwick�㷨
}

void simple_3d_trans(XYZ_Data_f *ref, XYZ_Data_f *in, XYZ_Data_f *out)   // С��Χ����ȷ��
{
    static s8    pn;
    static float h_tmp_x, h_tmp_y;

    h_tmp_x = my_sqrt(ref->z * ref->z + ref->y * ref->y);
    h_tmp_y = my_sqrt(ref->z * ref->z + ref->x * ref->x);

    pn = ref->z < 0 ? -1 : 1;

    out->x = (h_tmp_x * in->x - pn * ref->x * in->z);
    out->y = (pn * h_tmp_y * in->y - ref->y * in->z);
    out->z = ref->x * in->x + ref->y * in->y + ref->z * in->z;
}

//@����           ��tou_zi
//@��дʱ�� ��2019��4��6��
//@�޸�ʱ�� ��2019��4��6��
//@������      ��calculatePose_Module
//@�������� ����̬����   --  ��Ԫ������
//@����1      ��*pose ��̬����ṹ��ָ��
//@����2      ��cycle ����
//@����ֵ      ����
void calculatePose_Module(Pose_Module *pose, float cycle)
{
    if (pose->flag.algorithm == 1)
    {
        /* ԭʼ���������ݣ�ע�ⵥλת��������ϵת���� */
        // ���ٶ�ԭʼ���ݣ�cm/s? -> m/s?��
        float ax_raw = *pose->interface.data.a_x / 100.0f;
        float ay_raw = *pose->interface.data.a_y / 100.0f;
        float az_raw = *pose->interface.data.a_z / 100.0f;

        // ������ԭʼ���ݣ���/s -> rad/s��
        float gx_raw = *pose->interface.data.g_x * (MY_PI / 180.0f);
        float gy_raw = *pose->interface.data.g_y * (MY_PI / 180.0f);
        float gz_raw = *pose->interface.data.g_z * (MY_PI / 180.0f);

        /* FLU����ϵת�� */
        // ���ٶȼƣ�
        // X+��ǰ�򣨱��֣�
        // Y+�����򣨱��֣�
        // Z+��������Ҫȡ������Ϊ��ֹʱ������ԭʼZ�����£�
        float ax = ax_raw;
        float ay = ay_raw;
        float az = az_raw;   // ת����FLU����ϵ

        // �����ǣ�
        // X+���Ҳ෭�������֣�
        // Y+����ͷ������¸��������֣�
        // Z+����ʱ��ƫ�������֣�
        float gx = gx_raw;
        float gy = gy_raw;
        float gz = gz_raw;

        // ִ��Madgwick�㷨����
        MadgwickIMUUpdate(pose, cycle, gx, gy, gz, ax, ay, az);

        // ����Ԫ������ŷ����
        float q0 = pose->process.quaternion[0];
        float q1 = pose->process.quaternion[1];
        float q2 = pose->process.quaternion[2];
        float q3 = pose->process.quaternion[3];

        /* FLU����ϵ�µ�ŷ���Ƕ��壺
           - �����roll������X����ת���Ҳ�Ϊ����
           - ������pitch������Y����ת����ͷ����Ϊ����
           - ƫ����yaw������Z����ת����ʱ��Ϊ���� */

        // �������ǣ���X�ᣬ�Ҳ�Ϊ����
        float sinr     = 2.0f * (q0 * q1 + q3 * q2);
        float cosr     = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
        pose->data.rol = my_arctan2_rad(cosr, sinr) * RAD2DEG;

        // ���㸩���ǣ���Y�ᣬǰ��Ϊ����
        float sinp = 2.0f * (q0 * q2 - q3 * q1);
        if (my_fabs(sinp) >= 1.0f)
            pose->data.pit = my_copysignf(90.0f, sinp);
        else
            pose->data.pit = my_arcsin_rad(sinp) * RAD2DEG;

        // ����ƫ���ǣ���Z�ᣬ��ʱ��Ϊ����
        float siny     = 2.0f * (q0 * q3 + q1 * q2);
        float cosy     = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
        pose->data.yaw = my_arctan2_rad(cosy, siny) * RAD2DEG;

        // �Ƕȹ�һ����[-180, 180]
        // pose->data.yaw = my_translateAngleRad(pose->data.yaw);
        // pose->data.pit = my_translateAngleRad(pose->data.pit);
        // pose->data.rol = my_translateAngleRad(pose->data.rol);
    }
    else
    {
        float      length;
        XYZ_Data_f acc_tmp;
        XYZ_Data_f error;

        if (pose->flag.run == 0) return;

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // �������̴���
        if (pose->flag.use_mag == 1)
        {
            // ���õ������̼���yaw
            length = my_sqrt(pose->data.mag_correct.x * pose->data.mag_correct.x +
                             pose->data.mag_correct.y * pose->data.mag_correct.y);
            if (pose->data.mag_correct.x != 0 && pose->data.mag_correct.y != 0 && pose->data.mag_correct.z != 0 &&
                length != 0)
            {
                pose->process.mag_yaw =
                    my_arctan2(pose->data.mag_correct.y / length, pose->data.mag_correct.x / length);
            }

            // ����yawƫ��
            if (pose->data.rotate_matrix[2][2] > 0.0f)
            {
                pose->process.mag_yaw_bias =
                    pose->parameter.correct_kp * my_translateAngle(pose->data.yaw - pose->process.mag_yaw);
                // ����ֵ���� -- ����ֵ����
                if (pose->process.mag_yaw_bias > 360 || pose->process.mag_yaw_bias < -360)
                {
                    pose->process.mag_yaw_bias = 0;
                }
            }

            else
            {
                pose->process.mag_yaw_bias = 0;   // �Ƕȹ���ֹͣ������������Ŀ��ֵ���ܲ���ȷ
            }
        }

        else
        {
            pose->process.mag_yaw_bias = 0;
        }
        /////////////////////////////////////////////////////////////////////////////////////////////////
        // ���ٶȼƴ���
        length = my_sqrt(*(pose->interface.data.a_x) * *(pose->interface.data.a_x) +
                         *(pose->interface.data.a_y) * *(pose->interface.data.a_y) +
                         *(pose->interface.data.a_z) * *(pose->interface.data.a_z));

        if (ABS(*(pose->interface.data.a_x)) < 1050.0f && ABS(*(pose->interface.data.a_y)) < 1050.0f &&
            ABS(*(pose->interface.data.a_z)) < 1050.0f)
        {
            // ���ٶȼƹ�һ��
            acc_tmp.x = *(pose->interface.data.a_x) / length;
            acc_tmp.y = *(pose->interface.data.a_y) / length;
            acc_tmp.z = *(pose->interface.data.a_z) / length;

            // ��˼���ƫ��    --  �����˻��ȶ�ʱ���в���
            if (800.0f < length && length < 1200.0f)
            {
                /* ��˵õ���� */
                error.x = (acc_tmp.y * pose->data.rotate_matrix[2][2] - acc_tmp.z * pose->data.rotate_matrix[1][2]);
                error.y = (acc_tmp.z * pose->data.rotate_matrix[0][2] - acc_tmp.x * pose->data.rotate_matrix[2][2]);
                error.z = (acc_tmp.x * pose->data.rotate_matrix[1][2] - acc_tmp.y * pose->data.rotate_matrix[0][2]);

                /* ����ͨ */
                pose->process.error.x += 1.0f * 3.14f * cycle * (error.x - pose->process.error.x);
                pose->process.error.y += 1.0f * 3.14f * cycle * (error.y - pose->process.error.y);
                pose->process.error.z += 1.0f * 3.14f * cycle * (error.z - pose->process.error.z);
            }
        }
        else
        {
            pose->process.error.x = 0;
            pose->process.error.y = 0;
            pose->process.error.z = 0;
        }

        // ������
        pose->process.error_integral.x += pose->process.error.x * pose->parameter.error_ki * cycle;
        pose->process.error_integral.y += pose->process.error.y * pose->parameter.error_ki * cycle;
        pose->process.error_integral.z += pose->process.error.z * pose->parameter.error_ki * cycle;

        // �����޷� -- 2������
        pose->process.error_integral.x = LIMIT(pose->process.error_integral.x, -0.035f, 0.035f);
        pose->process.error_integral.y = LIMIT(pose->process.error_integral.y, -0.035f, 0.035f);
        pose->process.error_integral.z = LIMIT(pose->process.error_integral.z, -0.035f, 0.035f);

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // ��ʼ����������ֵ
        pose->data.gyro_correct.x =
            (*(pose->interface.data.g_x) - pose->data.rotate_matrix[0][2] * pose->process.mag_yaw_bias) * 0.01745329f +
            (pose->parameter.error_kp * pose->process.error.x + pose->process.error_integral.x);
        pose->data.gyro_correct.y =
            (*(pose->interface.data.g_y) - pose->data.rotate_matrix[1][2] * pose->process.mag_yaw_bias) * 0.01745329f +
            (pose->parameter.error_kp * pose->process.error.y + pose->process.error_integral.y);
        pose->data.gyro_correct.z =
            (*(pose->interface.data.g_z) - pose->data.rotate_matrix[2][2] * pose->process.mag_yaw_bias) * 0.01745329f +
            (pose->parameter.error_kp * pose->process.error.z + pose->process.error_integral.z);

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // һ���������������Ԫ��ֵ
        pose->process.quaternion[0] += (-pose->process.quaternion[1] * pose->data.gyro_correct.x -
                                        pose->process.quaternion[2] * pose->data.gyro_correct.y -
                                        pose->process.quaternion[3] * pose->data.gyro_correct.z) *
                                       cycle / 2.0f;
        pose->process.quaternion[1] += (pose->process.quaternion[0] * pose->data.gyro_correct.x +
                                        pose->process.quaternion[2] * pose->data.gyro_correct.z -
                                        pose->process.quaternion[3] * pose->data.gyro_correct.y) *
                                       cycle / 2.0f;
        pose->process.quaternion[2] += (pose->process.quaternion[0] * pose->data.gyro_correct.y -
                                        pose->process.quaternion[1] * pose->data.gyro_correct.z +
                                        pose->process.quaternion[3] * pose->data.gyro_correct.x) *
                                       cycle / 2.0f;
        pose->process.quaternion[3] += (pose->process.quaternion[0] * pose->data.gyro_correct.z +
                                        pose->process.quaternion[1] * pose->data.gyro_correct.y -
                                        pose->process.quaternion[2] * pose->data.gyro_correct.x) *
                                       cycle / 2.0f;

        // ��Ԫ����һ��
        length = my_sqrt(pose->process.quaternion[0] * pose->process.quaternion[0] +
                         pose->process.quaternion[1] * pose->process.quaternion[1] +
                         pose->process.quaternion[2] * pose->process.quaternion[2] +
                         pose->process.quaternion[3] * pose->process.quaternion[3]);

        if (length != 0)
        {
            pose->process.quaternion[0] /= length;
            pose->process.quaternion[1] /= length;
            pose->process.quaternion[2] /= length;
            pose->process.quaternion[3] /= length;
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////
        // ������ת����
        pose->data.rotate_matrix[0][0] = pose->process.quaternion[0] * pose->process.quaternion[0] +
                                         pose->process.quaternion[1] * pose->process.quaternion[1] -
                                         pose->process.quaternion[2] * pose->process.quaternion[2] -
                                         pose->process.quaternion[3] * pose->process.quaternion[3];
        pose->data.rotate_matrix[0][1] = 2 * (pose->process.quaternion[1] * pose->process.quaternion[2] +
                                              pose->process.quaternion[0] * pose->process.quaternion[3]);
        pose->data.rotate_matrix[0][2] = 2 * (pose->process.quaternion[1] * pose->process.quaternion[3] -
                                              pose->process.quaternion[0] * pose->process.quaternion[2]);

        pose->data.rotate_matrix[1][0] = 2 * (pose->process.quaternion[1] * pose->process.quaternion[2] -
                                              pose->process.quaternion[0] * pose->process.quaternion[3]);
        pose->data.rotate_matrix[1][1] = pose->process.quaternion[0] * pose->process.quaternion[0] -
                                         pose->process.quaternion[1] * pose->process.quaternion[1] +
                                         pose->process.quaternion[2] * pose->process.quaternion[2] -
                                         pose->process.quaternion[3] * pose->process.quaternion[3];
        pose->data.rotate_matrix[1][2] = 2 * (pose->process.quaternion[2] * pose->process.quaternion[3] +
                                              pose->process.quaternion[0] * pose->process.quaternion[1]);

        pose->data.rotate_matrix[2][0] = 2 * (pose->process.quaternion[1] * pose->process.quaternion[3] +
                                              pose->process.quaternion[0] * pose->process.quaternion[2]);
        pose->data.rotate_matrix[2][1] = 2 * (pose->process.quaternion[2] * pose->process.quaternion[3] -
                                              pose->process.quaternion[0] * pose->process.quaternion[1]);
        pose->data.rotate_matrix[2][2] = pose->process.quaternion[0] * pose->process.quaternion[0] -
                                         pose->process.quaternion[1] * pose->process.quaternion[1] -
                                         pose->process.quaternion[2] * pose->process.quaternion[2] +
                                         pose->process.quaternion[3] * pose->process.quaternion[3];

        // ������������ϵ�µĴ�����ֵ
        if (pose->flag.use_mag == 1)
        {
            pose->data.mag_world.x = pose->data.rotate_matrix[0][0] * *(pose->interface.data.m_x) +
                                     pose->data.rotate_matrix[1][0] * *(pose->interface.data.m_y) +
                                     pose->data.rotate_matrix[2][0] * *(pose->interface.data.m_z);

            pose->data.mag_world.y = pose->data.rotate_matrix[0][1] * *(pose->interface.data.m_x) +
                                     pose->data.rotate_matrix[1][1] * *(pose->interface.data.m_y) +
                                     pose->data.rotate_matrix[2][1] * *(pose->interface.data.m_z);

            pose->data.mag_world.z = pose->data.rotate_matrix[0][2] * *(pose->interface.data.m_x) +
                                     pose->data.rotate_matrix[1][2] * *(pose->interface.data.m_y) +
                                     pose->data.rotate_matrix[2][2] * *(pose->interface.data.m_z);
        }

        // ������������ϵ�µļ��ٶ�ֵ
        pose->data.acc_world.x = pose->data.rotate_matrix[0][0] * *(pose->interface.data.a_x) +
                                 pose->data.rotate_matrix[1][0] * *(pose->interface.data.a_y) +
                                 pose->data.rotate_matrix[2][0] * *(pose->interface.data.a_z);

        pose->data.acc_world.y = pose->data.rotate_matrix[0][1] * *(pose->interface.data.a_x) +
                                 pose->data.rotate_matrix[1][1] * *(pose->interface.data.a_y) +
                                 pose->data.rotate_matrix[2][1] * *(pose->interface.data.a_z);

        pose->data.acc_world.z = pose->data.rotate_matrix[0][2] * *(pose->interface.data.a_x) +
                                 pose->data.rotate_matrix[1][2] * *(pose->interface.data.a_y) +
                                 pose->data.rotate_matrix[2][2] * *(pose->interface.data.a_z);

        // ���ŷ����
        pose->data.rol = my_arctan2(pose->data.rotate_matrix[2][2], pose->data.rotate_matrix[1][2]);
        pose->data.pit = -my_arcsin(pose->data.rotate_matrix[0][2]);
        pose->data.yaw = my_arctan2(pose->data.rotate_matrix[0][0], pose->data.rotate_matrix[0][1]);

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // �����������ϵ������ļ��ٶ�--���ܸ����ͷ���Ӱ��
        pose->data.acc_correct.x =
            pose->data.acc_world.x * my_cos(pose->data.yaw) + pose->data.acc_world.y * my_sin(pose->data.yaw);
        pose->data.acc_correct.y =
            -pose->data.acc_world.x * my_sin(pose->data.yaw) + pose->data.acc_world.y * my_cos(pose->data.yaw);
        pose->data.acc_correct.z = pose->data.acc_world.z;

        // �����������ϵ������Ĵų�--���ܸ����ͷ���Ӱ��
        if (pose->flag.use_mag == 1)
        {
            XYZ_Data_f ref_v = (XYZ_Data_f){
                pose->data.rotate_matrix[0][2], pose->data.rotate_matrix[1][2], pose->data.rotate_matrix[2][2]};
            XYZ_Data_f mag_tmp =
                (XYZ_Data_f){*pose->interface.data.m_x, *pose->interface.data.m_y, *pose->interface.data.m_z};

            length = my_sqrt(*(pose->interface.data.m_x) * *(pose->interface.data.m_x) +
                             *(pose->interface.data.m_y) * *(pose->interface.data.m_y) +
                             *(pose->interface.data.m_z) * *(pose->interface.data.m_z));

            //      length *= sqrt( pose->data.rotate_matrix[0][2] * pose->data.rotate_matrix[0][2]
            //      +
            //                                      pose->data.rotate_matrix[1][2] *
            //                                      pose->data.rotate_matrix[1][2] +
            //                                      pose->data.rotate_matrix[2][2] *
            //                                      pose->data.rotate_matrix[2][2] );
            //
            if (length != 0)
            {
                simple_3d_trans(&ref_v, &mag_tmp, &pose->data.mag_correct);
                //          pose->data.mag_correct.z = pose->data.mag_world.z;
                //          pose->data.mag_correct.x = sqrt(1 - (pose->data.mag_correct.z / length)
                //          * (pose->data.mag_correct.z / length)) * *(pose->interface.data.m_x);
                //          pose->data.mag_correct.y = sqrt(1 - (pose->data.mag_correct.z / length)
                //          * (pose->data.mag_correct.z / length)) * *(pose->interface.data.m_y);
            }
        }
    }
}
/* Madgwick IMU�����㷨��FLU����ϵ�棩 */
void MadgwickIMUUpdate(Pose_Module *pose, float cycle, float gx, float gy, float gz, float ax, float ay, float az)
{
    float  recipNorm;
    float  s0, s1, s2, s3;
    float  qDot1, qDot2, qDot3, qDot4;
    float  _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
    float  q0q0, q1q1, q2q2, q3q3;
    float *q = pose->process.quaternion;

    /* �����ǻ��֣�FLU����ϵ�� */
    qDot1 = 0.5f * (-q[1] * gx - q[2] * gy - q[3] * gz);
    qDot2 = 0.5f * (q[0] * gx + q[2] * gz - q[3] * gy);
    qDot3 = 0.5f * (q[0] * gy - q[1] * gz + q[3] * gx);
    qDot4 = 0.5f * (q[0] * gz + q[1] * gy - q[2] * gx);

    /* ���ٶȼ�У�� */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        /* ��һ�����ٶȼ����� */
        recipNorm = my_invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        // ��ӡax ay az
        // printf("ax:%.2f ay:%.2f az:%.2f\n", ax, ay, az);
        /* ���㸨����������ӦFLU����ϵ�� */
        _2q0 = 2.0f * q[0];
        _2q1 = 2.0f * q[1];
        _2q2 = 2.0f * q[2];
        _2q3 = 2.0f * q[3];
        _4q0 = 4.0f * q[0];
        _4q1 = 4.0f * q[1];
        _4q2 = 4.0f * q[2];
        _8q1 = 8.0f * q[1];
        _8q2 = 8.0f * q[2];
        q0q0 = q[0] * q[0];
        q1q1 = q[1] * q[1];
        q2q2 = q[2] * q[2];
        q3q3 = q[3] * q[3];

        /* �ݶ��½���У����������������ΪZ+�� */
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q[1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q[3] - _2q1 * ax + 4.0f * q2q2 * q[3] - _2q2 * ay;

        /* ��һ��У���� */
        recipNorm = my_invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        /* Ӧ�÷��� */
        qDot1 -= pose->parameter.beta * s0;
        qDot2 -= pose->parameter.beta * s1;
        qDot3 -= pose->parameter.beta * s2;
        qDot4 -= pose->parameter.beta * s3;
    }

    /* ������Ԫ�� */
    q[0] += qDot1 * cycle;
    q[1] += qDot2 * cycle;
    q[2] += qDot3 * cycle;
    q[3] += qDot4 * cycle;

    /* ��Ԫ����һ�� */
    recipNorm = my_invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] *= recipNorm;
    q[1] *= recipNorm;
    q[2] *= recipNorm;
    q[3] *= recipNorm;
}

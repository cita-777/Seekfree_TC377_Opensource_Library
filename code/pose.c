/**
 * *****************************************************************************
 * @file        pose.c
 * @brief       Mahony解算结构体
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-25
 * *****************************************************************************
 */
#include "zf_common_headfile.h"
// 静态函数声明
void MadgwickIMUUpdate(Pose_Module *pose, float cycle, float gx, float gy, float gz, float ax, float ay, float az);
const float RAD2DEG = 180.0f / MY_PI;
//@作者           ：tou_zi
//@编写时间 ：2019年4月6日
//@修改时间 ：2019年4月6日
//@函数名      ：initPose_Module
//@函数功能 ：初始化姿态解算模块
//@输入1      ：*pose 姿态解算结构体指针
//@返回值      ：无
void initPose_Module(Pose_Module *pose)
{
    // 标志位初始化
    pose->flag.run     = 1;   // 开启计算
    pose->flag.use_mag = 0;   // 使用电子罗盘
    // 接口初始化
    pose->interface.data.a_x = 0;
    pose->interface.data.a_y = 0;
    pose->interface.data.a_z = 0;
    pose->interface.data.g_x = 0;
    pose->interface.data.g_y = 0;
    pose->interface.data.g_z = 0;
    pose->interface.data.m_x = 0;
    pose->interface.data.m_y = 0;
    pose->interface.data.m_z = 0;
    // 参数初始化
    pose->parameter.error_ki   = 0.00f;
    pose->parameter.error_kp   = 0.35f;
    pose->parameter.correct_kp = 0.6f;
    // pose->parameter.error_kp = 0.0f;
    // pose->parameter.correct_kp = 0.0f;
    // 中间变量清空
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
    // 数据初始化
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

    // 初始化madgwick算法参数
    pose->parameter.beta = 0.00001f;   // Madgwick滤波器增益系数
    pose->flag.algorithm = 1;          // 默认使用Madgwick算法
}

void simple_3d_trans(XYZ_Data_f *ref, XYZ_Data_f *in, XYZ_Data_f *out)   // 小范围内正确。
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

//@作者           ：tou_zi
//@编写时间 ：2019年4月6日
//@修改时间 ：2019年4月6日
//@函数名      ：calculatePose_Module
//@函数功能 ：姿态解算   --  四元数解算
//@输入1      ：*pose 姿态解算结构体指针
//@输入2      ：cycle 周期
//@返回值      ：无
void calculatePose_Module(Pose_Module *pose, float cycle)
{
    if (pose->flag.algorithm == 1)
    {
        /* 原始传感器数据（注意单位转换和坐标系转换） */
        // 加速度原始数据（cm/s? -> m/s?）
        float ax_raw = *pose->interface.data.a_x / 100.0f;
        float ay_raw = *pose->interface.data.a_y / 100.0f;
        float az_raw = *pose->interface.data.a_z / 100.0f;

        // 陀螺仪原始数据（°/s -> rad/s）
        float gx_raw = *pose->interface.data.g_x * (MY_PI / 180.0f);
        float gy_raw = *pose->interface.data.g_y * (MY_PI / 180.0f);
        float gz_raw = *pose->interface.data.g_z * (MY_PI / 180.0f);

        /* FLU坐标系转换 */
        // 加速度计：
        // X+：前向（保持）
        // Y+：左向（保持）
        // Z+：上向（需要取反，因为静止时传感器原始Z轴向下）
        float ax = ax_raw;
        float ay = ay_raw;
        float az = az_raw;   // 转换到FLU坐标系

        // 陀螺仪：
        // X+：右侧翻滚（保持）
        // Y+：机头左侧向下俯仰（保持）
        // Z+：逆时针偏航（保持）
        float gx = gx_raw;
        float gy = gy_raw;
        float gz = gz_raw;

        // 执行Madgwick算法更新
        MadgwickIMUUpdate(pose, cycle, gx, gy, gz, ax, ay, az);

        // 从四元数计算欧拉角
        float q0 = pose->process.quaternion[0];
        float q1 = pose->process.quaternion[1];
        float q2 = pose->process.quaternion[2];
        float q3 = pose->process.quaternion[3];

        /* FLU坐标系下的欧拉角定义：
           - 横滚（roll）：绕X轴旋转（右侧为正）
           - 俯仰（pitch）：绕Y轴旋转（机头向下为正）
           - 偏航（yaw）：绕Z轴旋转（逆时针为正） */

        // 计算横滚角（绕X轴，右侧为正）
        float sinr     = 2.0f * (q0 * q1 + q3 * q2);
        float cosr     = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
        pose->data.rol = my_arctan2_rad(cosr, sinr) * RAD2DEG;

        // 计算俯仰角（绕Y轴，前倾为正）
        float sinp = 2.0f * (q0 * q2 - q3 * q1);
        if (my_fabs(sinp) >= 1.0f)
            pose->data.pit = my_copysignf(90.0f, sinp);
        else
            pose->data.pit = my_arcsin_rad(sinp) * RAD2DEG;

        // 计算偏航角（绕Z轴，逆时针为正）
        float siny     = 2.0f * (q0 * q3 + q1 * q2);
        float cosy     = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
        pose->data.yaw = my_arctan2_rad(cosy, siny) * RAD2DEG;

        // 角度归一化到[-180, 180]
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
        // 电子罗盘处理
        if (pose->flag.use_mag == 1)
        {
            // 利用电子罗盘计算yaw
            length = my_sqrt(pose->data.mag_correct.x * pose->data.mag_correct.x +
                             pose->data.mag_correct.y * pose->data.mag_correct.y);
            if (pose->data.mag_correct.x != 0 && pose->data.mag_correct.y != 0 && pose->data.mag_correct.z != 0 &&
                length != 0)
            {
                pose->process.mag_yaw =
                    my_arctan2(pose->data.mag_correct.y / length, pose->data.mag_correct.x / length);
            }

            // 计算yaw偏差
            if (pose->data.rotate_matrix[2][2] > 0.0f)
            {
                pose->process.mag_yaw_bias =
                    pose->parameter.correct_kp * my_translateAngle(pose->data.yaw - pose->process.mag_yaw);
                // 矫正值过大 -- 矫正值错误
                if (pose->process.mag_yaw_bias > 360 || pose->process.mag_yaw_bias < -360)
                {
                    pose->process.mag_yaw_bias = 0;
                }
            }

            else
            {
                pose->process.mag_yaw_bias = 0;   // 角度过大，停止修正，修正的目标值可能不正确
            }
        }

        else
        {
            pose->process.mag_yaw_bias = 0;
        }
        /////////////////////////////////////////////////////////////////////////////////////////////////
        // 加速度计处理
        length = my_sqrt(*(pose->interface.data.a_x) * *(pose->interface.data.a_x) +
                         *(pose->interface.data.a_y) * *(pose->interface.data.a_y) +
                         *(pose->interface.data.a_z) * *(pose->interface.data.a_z));

        if (ABS(*(pose->interface.data.a_x)) < 1050.0f && ABS(*(pose->interface.data.a_y)) < 1050.0f &&
            ABS(*(pose->interface.data.a_z)) < 1050.0f)
        {
            // 加速度计归一化
            acc_tmp.x = *(pose->interface.data.a_x) / length;
            acc_tmp.y = *(pose->interface.data.a_y) / length;
            acc_tmp.z = *(pose->interface.data.a_z) / length;

            // 叉乘计算偏差    --  在无人机稳定时进行补偿
            if (800.0f < length && length < 1200.0f)
            {
                /* 叉乘得到误差 */
                error.x = (acc_tmp.y * pose->data.rotate_matrix[2][2] - acc_tmp.z * pose->data.rotate_matrix[1][2]);
                error.y = (acc_tmp.z * pose->data.rotate_matrix[0][2] - acc_tmp.x * pose->data.rotate_matrix[2][2]);
                error.z = (acc_tmp.x * pose->data.rotate_matrix[1][2] - acc_tmp.y * pose->data.rotate_matrix[0][2]);

                /* 误差低通 */
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

        // 误差积分
        pose->process.error_integral.x += pose->process.error.x * pose->parameter.error_ki * cycle;
        pose->process.error_integral.y += pose->process.error.y * pose->parameter.error_ki * cycle;
        pose->process.error_integral.z += pose->process.error.z * pose->parameter.error_ki * cycle;

        // 积分限幅 -- 2°以内
        pose->process.error_integral.x = LIMIT(pose->process.error_integral.x, -0.035f, 0.035f);
        pose->process.error_integral.y = LIMIT(pose->process.error_integral.y, -0.035f, 0.035f);
        pose->process.error_integral.z = LIMIT(pose->process.error_integral.z, -0.035f, 0.035f);

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // 开始修正陀螺仪值
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
        // 一阶龙格库塔更新四元数值
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

        // 四元数归一化
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
        // 计算旋转矩阵
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

        // 计算世界坐标系下的磁力计值
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

        // 计算世界坐标系下的加速度值
        pose->data.acc_world.x = pose->data.rotate_matrix[0][0] * *(pose->interface.data.a_x) +
                                 pose->data.rotate_matrix[1][0] * *(pose->interface.data.a_y) +
                                 pose->data.rotate_matrix[2][0] * *(pose->interface.data.a_z);

        pose->data.acc_world.y = pose->data.rotate_matrix[0][1] * *(pose->interface.data.a_x) +
                                 pose->data.rotate_matrix[1][1] * *(pose->interface.data.a_y) +
                                 pose->data.rotate_matrix[2][1] * *(pose->interface.data.a_z);

        pose->data.acc_world.z = pose->data.rotate_matrix[0][2] * *(pose->interface.data.a_x) +
                                 pose->data.rotate_matrix[1][2] * *(pose->interface.data.a_y) +
                                 pose->data.rotate_matrix[2][2] * *(pose->interface.data.a_z);

        // 求解欧拉角
        pose->data.rol = my_arctan2(pose->data.rotate_matrix[2][2], pose->data.rotate_matrix[1][2]);
        pose->data.pit = -my_arcsin(pose->data.rotate_matrix[0][2]);
        pose->data.yaw = my_arctan2(pose->data.rotate_matrix[0][0], pose->data.rotate_matrix[0][1]);

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // 计算机体坐标系矫正后的加速度--不受俯仰和翻滚影响
        pose->data.acc_correct.x =
            pose->data.acc_world.x * my_cos(pose->data.yaw) + pose->data.acc_world.y * my_sin(pose->data.yaw);
        pose->data.acc_correct.y =
            -pose->data.acc_world.x * my_sin(pose->data.yaw) + pose->data.acc_world.y * my_cos(pose->data.yaw);
        pose->data.acc_correct.z = pose->data.acc_world.z;

        // 计算机体坐标系矫正后的磁场--不受俯仰和翻滚影响
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
/* Madgwick IMU更新算法（FLU坐标系版） */
void MadgwickIMUUpdate(Pose_Module *pose, float cycle, float gx, float gy, float gz, float ax, float ay, float az)
{
    float  recipNorm;
    float  s0, s1, s2, s3;
    float  qDot1, qDot2, qDot3, qDot4;
    float  _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
    float  q0q0, q1q1, q2q2, q3q3;
    float *q = pose->process.quaternion;

    /* 陀螺仪积分（FLU坐标系） */
    qDot1 = 0.5f * (-q[1] * gx - q[2] * gy - q[3] * gz);
    qDot2 = 0.5f * (q[0] * gx + q[2] * gz - q[3] * gy);
    qDot3 = 0.5f * (q[0] * gy - q[1] * gz + q[3] * gx);
    qDot4 = 0.5f * (q[0] * gz + q[1] * gy - q[2] * gx);

    /* 加速度计校正 */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        /* 归一化加速度计数据 */
        recipNorm = my_invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        // 打印ax ay az
        // printf("ax:%.2f ay:%.2f az:%.2f\n", ax, ay, az);
        /* 计算辅助变量（适应FLU坐标系） */
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

        /* 梯度下降法校正（调整重力方向为Z+） */
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q[1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q[3] - _2q1 * ax + 4.0f * q2q2 * q[3] - _2q2 * ay;

        /* 归一化校正量 */
        recipNorm = my_invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        /* 应用反馈 */
        qDot1 -= pose->parameter.beta * s0;
        qDot2 -= pose->parameter.beta * s1;
        qDot3 -= pose->parameter.beta * s2;
        qDot4 -= pose->parameter.beta * s3;
    }

    /* 积分四元数 */
    q[0] += qDot1 * cycle;
    q[1] += qDot2 * cycle;
    q[2] += qDot3 * cycle;
    q[3] += qDot4 * cycle;

    /* 四元数归一化 */
    recipNorm = my_invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] *= recipNorm;
    q[1] *= recipNorm;
    q[2] *= recipNorm;
    q[3] *= recipNorm;
}

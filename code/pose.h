/**
 * *****************************************************************************
 * @file        pose.h
 * @brief       Mahony解算结构体
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
 * @brief 三维数据
 * @details 用于存储三维数据（x, y, z），支持不同精度（浮点、整型）
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
 * @brief 姿态标志位
 * @details 标志位，控制模块是否运行（run）和是否使用磁力计（use_mag）。
 */
typedef struct Pose_Flag
{
    u8 run;
    u8 use_mag;
    u8 algorithm;
} Pose_Flag;

/**
 * @brief 姿态数据接口
 * @details 传感器数据接口，指向加速度、陀螺仪、磁力计的原始数据。
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
 * @brief 姿态数据接口
 * @details 封装Pose_DInterface成一个，用于姿态模块的数据传递。
 */
typedef struct Pose_Interface
{
    Pose_DInterface data;
} Pose_Interface;

/**
 * @brief 姿态数据
 * @details 存储解算后的姿态数据，包括欧拉角、旋转矩阵、校正后的传感器数据等。
 */
typedef struct Pose_Data
{
    float yaw;
    float rol;
    float pit;

    float rotate_matrix[3][3];   // 旋转矩阵

    XYZ_Data_f acc_world;   // 世界坐标系下的加速度
    XYZ_Data_f mag_world;   // 世界坐标系下的磁场强度   --  只与无人机位置有关的量

    XYZ_Data_f acc_correct;    // 机体坐标系下的加速度    --  矫正了俯仰翻滚后的加速度
    XYZ_Data_f mag_correct;    // 机体坐标系下的磁场强度   --  矫正了俯仰翻滚后的磁场强度
    XYZ_Data_f gyro_correct;   // 融合加速度和磁力计数据，矫正后的陀螺仪值
} Pose_Data;

/**
 * @brief 姿态处理
 * @details 中间计算变量，如四元数、误差积分、磁力计偏航角。
 */
typedef struct Pose_Process
{
    float      mag_yaw;          // 磁力计的偏航角
    float      mag_yaw_bias;     // 磁力计矫正的偏航角偏差
    float      quaternion[4];    // 四元数
    XYZ_Data_f error;            // 由加速度计与等效重力的偏差
    XYZ_Data_f error_integral;   // 误差积分
} Pose_Process;

/**
 * @brief 姿态参数
 * @details 算法参数，误差补偿的比例（kp）和积分（ki）系数。
 */
typedef struct Pose_Parameter
{
    float correct_kp;
    float error_kp;
    float error_ki;

    float beta;
} Pose_Parameter;

/**
 * @brief 姿态模块
 * @details 整合所有结构体，是姿态解算的核心模块。
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
 * @brief      初始化姿态模块
 * @details    初始化姿态模块，设置初始参数（如四元数初始化为单位四元数）、清零数据。
 * @param       pose
 */
void initPose_Module(Pose_Module *pose);
/**
 * @brief      姿态解算
 * @details    姿态解算，通过四元数解算，计算姿态角度、旋转矩阵、校正后的传感器数据。
 * @param       pose
 * @param       cycle
 */
void calculatePose_Module(Pose_Module *pose, float cycle);
/*------------------------------------test------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* POSE_H */

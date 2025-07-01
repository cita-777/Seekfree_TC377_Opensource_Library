/**
 * *****************************************************************************
 * @file        imu_wifi.h
 * @brief       IMU_Angle供外部查询使用
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-24
 * *****************************************************************************
 */

/**
 *  方式1：直接访问
    float current_yaw = g_imu_angle.yaw;

    方式2：通过函数接口
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
#define IMU_USE_963RA (0)      // 定义使用 963RA IMU
#define IMU_USE_ICM45686 (1)   // 定义使用 ICM45686 IMU

// 选择WiFi配置
#define WIFI_CONFIG 3   // 1: CMCC-nAHP, 2: cita, 3: NEWX

#if WIFI_CONFIG == 1
#    define MY_WIFI_SSID "CMCC-nAHP"
#    define MY_WIFI_PASSWORD "18065890955"
#    define MY_TCP_TARGET_IP "192.168.1.5"   // 连接目标的 IP
#elif WIFI_CONFIG == 2
#    define MY_WIFI_SSID "cita"
#    define MY_WIFI_PASSWORD "88888888"
#    define MY_TCP_TARGET_IP "192.168.*.*"
#elif WIFI_CONFIG == 3
#    define MY_WIFI_SSID "NEWX"
#    define MY_WIFI_PASSWORD "dxswlwhgzs"
#    define MY_TCP_TARGET_IP "192.168.5.5"   // 连接目标的 IP
#elif WIFI_CONFIG == 4
#    define MY_WIFI_SSID "xingyao14"
#    define MY_WIFI_PASSWORD "cjk20158"
#    define MY_TCP_TARGET_IP "192.168.137.1"   // 连接目标的 IP
#else
#    error "Invalid WIFI_CONFIG value. Must be 1, 2, or 3."
#endif

#define MY_TCP_TARGET_PORT "8086"   // 连接目标的端口
#define MY_WIFI_LOCAL_PORT "6666"   // 本机的端口 0：随机  可设置范围2048-65535  默认 6666

#define IMU_PIT (CCU60_CH0)

/*----------------------------------typedef-----------------------------------*/
/**
 * @brief 用于外部查询的精确角度数据
 */
typedef struct
{
    float yaw;     // 航向角，单位°（经过滤波和修正）[-180, 180]
    float pitch;   // 俯仰角，单位°
    float roll;    // 横滚角，单位°

    // 扩展角度变量，支持-无穷到+无穷范围
    float yaw_extended;     // 扩展航向角，可以超出±180°范围
    float pitch_extended;   // 扩展俯仰角
    float roll_extended;    // 扩展横滚角
} IMU_Angle;
// 声明外部可访问的角度数据
extern IMU_Angle g_imu_angle;
/*----------------------------------variable----------------------------------*/
extern uint8 offset_flag;
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
/**
 * @brief 初始化IMU设备、滤波器和内部数据结构
 */
void imu_init(void);

/**
 * @brief 通过Madgwick或Mahony算法处理IMU数据（采集、滤波、位姿计算）
 */
void imu_proc(void);

/**
 * @brief 查询经过滤波和修正后的角度数据
 */
IMU_Angle* IMU_GetAngle(void);

/**
 * @brief 初始化WIFI模块及建立连接
 */
void wifi_init(void);

/**
 * @brief 通过WIFI发送数据
 */
void wifi_spi_send(void);

/**
 * @brief 调试专用：中断服务函数，发送原始数据到调试工具
 */
void imu_oscilloscope_isr(void);
void icm45686_oscilloscope_isr(void);
/*------------------------------------test------------------------------------*/
/**
 * @brief 获取扩展角度值（无限制范围）
 */
float IMU_GetExtendedYaw(void);

/**
 * @brief 重置扩展角度累积值
 */
void IMU_ResetExtendedAngles(void);

/**
 * @brief 设置扩展角度的初始值
 */
void IMU_SetExtendedYaw(float yaw);

#ifdef __cplusplus
}
#endif

#endif /* IMU_WIFI_H */

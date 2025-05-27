/**
 * *****************************************************************************
 * @file        imu.c
 * @brief       优化后的 IMU 与 WIFI SPI 模块实现
 *              内部数据结构IMU_Private_Data保存了经过滤波与解算后的精确角度，
 *              而原始的姿态数据（pose）作为私有成员存在于其中。
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-24
 * *****************************************************************************
 */

/*----------------------------------include-----------------------------------*/
#include "zf_common_headfile.h"

/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/
/**
 * @brief 内部私有数据结构，用于存放IMU采集数据、滤波数据以及计算后的角度
 */
typedef struct
{
    /* 原始传感器数据（如有需要，可扩展保存） */
    float raw_acc[3];
    float raw_gyro[3];
    float raw_mag[3];

    /* 经过低通滤波后的传感器数据 */
    float acc_filtered[3];
    float gyro_filtered[3];
    float mag_filtered[3];

    /* 内部私有的姿态数据，保存原始解算数据（未修正的） */
    Pose_Module pose;

    /* 经过进一步修正和补偿后的角度，供外部查询 */
    float yaw;
    float pitch;
    float roll;
} IMU_Private_Data;
// 定义全局角度数据结构
IMU_Angle g_imu_angle = {0};
/*----------------------------------variable----------------------------------*/
/* 内部IMU数据，仅在当前文件中使用 */
IMU_Private_Data imu_private;

/* 用于巴特沃夫低通滤波的缓存和参数 */
static Butter_BufferData accel_filter_buf[3];
static Butter_BufferData gyro_filter_buf[3];
static Butter_Parameter  Accel_Parameter;
static Butter_Parameter  Gyro_Parameter;

/* RLS滤波器状态（用于校正传感器偏置），分别对应三个轴 */
static RLSState rls_acc[3] = {{0.0f, 1000.0f, 0.98f}, {0.0f, 1000.0f, 0.98f}, {0.0f, 1000.0f, 0.98f}};

static RLSState rls_gyro[3] = {{0.0f, 1000.0f, 0.98f}, {0.0f, 1000.0f, 0.98f}, {0.0f, 1000.0f, 0.98f}};

/* 内部偏置，计算得到，用于修正传感器数据 */
float offset_acce[3] = {0.0f, 0.0f, 0.0f};
float offset_gyro[3] = {0.0f, 0.0f, 0.0f};

/* 内部计数器，用于零点漂移修正 */
// static uint32 x_counter  = 0;
// static double zero_error = 0.0;

/* 控制采样标志（在校准阶段使用） */
uint8 offset_flag = 0;
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/

/**
 * @brief 初始化IMU设备、滤波器和内部数据结构
 */
void imu_init(void)
{
    // 初始化IMU设备
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
    // 添加足够长的稳定延时
    system_delay_ms(100);   // 增加初始化后的延时

#endif   // IMU_USE_ICM45686

    // 设置低通滤波器截止频率
    Set_Cutoff_Frequency(1000, 200, &Accel_Parameter);
    Set_Cutoff_Frequency(1000, 100, &Gyro_Parameter);

    /* 校准阶段：采集多次数据更新传感器偏置 */
    int sample_count = 100;
    int i            = 0;
    while (i < sample_count)
    {
        if (offset_flag == 1)
        {   // 当采样标志触发时
            offset_flag = 0;
/* 获取IMU原始数据 */
#if IMU_USE_963RA
            imu963ra_get_acc();
            imu963ra_get_gyro();
            imu963ra_get_mag();

            // 获取原始数据（仅调用数据转换接口）
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
            // 获取数据后加入LPButeerworth，绑定bsp_IcmGetRawData
            float meas_acc[3];
            float meas_gyro[3];
            float temp_degc;
            bsp_IcmGetRawData(meas_acc, meas_gyro, &temp_degc);

            // 使用LPButterworth对加速度数据进行滤波处理
            meas_acc[0] = LPButterworth(meas_acc[0], &accel_filter_buf[0], &Accel_Parameter);
            meas_acc[1] = LPButterworth(meas_acc[1], &accel_filter_buf[1], &Accel_Parameter);
            meas_acc[2] = LPButterworth(meas_acc[2], &accel_filter_buf[2], &Accel_Parameter);

            // 使用LPButterworth对陀螺仪数据进行滤波处理
            meas_gyro[0] = LPButterworth(meas_gyro[0], &gyro_filter_buf[0], &Gyro_Parameter);
            meas_gyro[1] = LPButterworth(meas_gyro[1], &gyro_filter_buf[1], &Gyro_Parameter);
            meas_gyro[2] = LPButterworth(meas_gyro[2], &gyro_filter_buf[2], &Gyro_Parameter);
#endif   // IMU_USE_ICM45686
         // 用RLS算法更新偏置
            for (int j = 0; j < 3; j++)
            {
                RLS_Update(&rls_acc[j], meas_acc[j]);
                RLS_Update(&rls_gyro[j], meas_gyro[j]);
            }
            i++;
        }
    }

    // 采样结束后，保存最终估计的偏置
    for (int j = 0; j < 3; j++)
    {
        offset_acce[j] = rls_acc[j].bias;
        offset_gyro[j] = rls_gyro[j].bias;
    }

    // 初始化内部私有的pose结构
    initPose_Module(&imu_private.pose);

    // 初始化内部角度数据为0
    imu_private.yaw = imu_private.pitch = imu_private.roll = 0.0f;
}

/**
 * @brief 通过madgwick或mahony算法处理IMU数据：采集数据、滤波、位姿解算并更新外部查询角度
 */
void imu_proc(void)
{
    // timer_enable();
#if IMU_WIFI_GET_PARAM_FLAG
    // 如果有上位机传参，则更新位姿解算参数（此处直接传递参数数组中的部分数据）
    imu_private.pose.parameter.error_ki   = seekfree_assistant_parameter[5];
    imu_private.pose.parameter.error_kp   = seekfree_assistant_parameter[6];
    imu_private.pose.parameter.correct_kp = seekfree_assistant_parameter[7];
#endif

    /* 获取IMU原始数据 */
#if IMU_USE_963RA
    imu963ra_get_acc();
    imu963ra_get_gyro();
    imu963ra_get_mag();
    /* 数据转换及偏置修正，同时保存原始数据（如有需要） */
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
    /* 单位转换及偏置修正
       加速度：g -> cm/s2（1g约=980.665 cm/s2）
       陀螺仪：单位保持为 °/s
       备注：可根据硬件安装调整符号 */
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

    /* 对加速度和陀螺仪数据进行低通滤波 */
    for (int j = 0; j < 3; j++)
    {
        imu_private.acc_filtered[j]  = LPButterworth(acc[j], &accel_filter_buf[j], &Accel_Parameter);
        imu_private.gyro_filtered[j] = LPButterworth(gyro[j], &gyro_filter_buf[j], &Gyro_Parameter);
    }
    /* 磁力计数据若需要滤波，可类似处理，此处直接赋值 */
    for (int j = 0; j < 3; j++)
    {
        imu_private.mag_filtered[j] = mag[j];
    }

    /* 将滤波后的数据传递给内部私有的pose结构
       注意：数据轴的对应关系请根据具体安装情况调整 */
    imu_private.pose.interface.data.a_x = &imu_private.acc_filtered[1];
    imu_private.pose.interface.data.g_x = &imu_private.gyro_filtered[1];
    imu_private.pose.interface.data.m_x = &imu_private.mag_filtered[1];
    imu_private.pose.interface.data.a_y = &imu_private.acc_filtered[0];
    imu_private.pose.interface.data.g_y = &imu_private.gyro_filtered[0];
    imu_private.pose.interface.data.m_y = &imu_private.mag_filtered[0];
    imu_private.pose.interface.data.a_z = &imu_private.acc_filtered[2];
    imu_private.pose.interface.data.g_z = &imu_private.gyro_filtered[2];
    imu_private.pose.interface.data.m_z = &imu_private.mag_filtered[2];

    /* 调用位姿解算函数（例如Mahony算法），更新内部pose数据 */
    calculatePose_Module(&imu_private.pose, 0.001f);

    /* 根据需要进行零点漂移修正 */
    // x_counter++;
    // zero_error = 4.68411239e-09 * Power2(x_counter) - 8.86316947e-04 * x_counter
    // + 2.26171105e-02;
    //  此处可在yaw上做修正，如：imu_private.yaw = imu_private.pose.data.yaw - zero_error;
    //  示例:imu_private.yaw   = imu_private.pose.data.yaw - zero_error;
    imu_private.yaw   = imu_private.pose.data.yaw;
    imu_private.pitch = imu_private.pose.data.pit;
    imu_private.roll  = imu_private.pose.data.rol;

    g_imu_angle.yaw   = imu_private.pose.data.yaw;
    g_imu_angle.pitch = imu_private.pose.data.pit;
    g_imu_angle.roll  = imu_private.pose.data.rol;
    // static uint32_t display_counter = 0;
    // if (++display_counter >= 1)   // 每50ms显示一次(约20Hz)
    // {
    //     display_counter = 0;

    //     // IPS显示更新
    //     ips_show_string(0, 0, "Pose:");
    //     ips_show_float(90, 0, imu_private.pose.data.yaw, 2, 6);

    //     ips_show_string(0, 16, "IMU:");
    //     ips_show_float(90, 16, imu_private.yaw, 2, 6);

    //     ips_show_string(0, 32, "Angle:");
    //     ips_show_float(90, 32, g_imu_angle.yaw, 2, 6);

    //     // // 使用sprintf组合字符串,一次性输出
    //     // char debug_str[100];
    //     // sprintf(debug_str,
    //     //         "\r\nDebug - [%u] Pose:%.2f IMU:%.2f Angle:%.2f",
    //     //         display_counter,
    //     //         imu_private.pose.data.yaw,
    //     //         imu_private.yaw,
    //     //         g_imu_angle.yaw);
    //     // printf("%s", debug_str);
    // }

    // // 添加调试输出以验证数值
    // printf("\r\nDebug - Pose:%.2f IMU:%.2f Angle:%.2f", imu_private.pose.data.yaw, imu_private.yaw, g_imu_angle.yaw);
#if IMU_WIFI_SEND_FLAG
    wifi_spi_send();
#endif
    disable_handler("imu");
    // timer_disable();
}

/**
 * @brief 供外部查询经过滤波与修正后的角度数据
 */
IMU_Angle* IMU_GetAngle(void)
{
    return &g_imu_angle;
}

/**
 * @brief 初始化WIFI模块及建立连接
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
 * @brief 通过WIFI发送数据（示例中发送部分角度与传感器数据）
 */
void wifi_spi_send(void)
{
#if IMU_WIFI_SEND_FLAG
    // 这里示例发送内部滤波后的角度和部分传感器数据
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
    //  设置本次需要发送几个通道的数据
    seekfree_assistant_oscilloscope_data.channel_num = 3;

    // 这里进发送了6个通道的数据，最大支持8通道
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
    // 有可能会在逐飞助手软件上看到波形更新不够连续，这是因为使用WIFI有不确定的延迟导致的

    // 解析上位机发送过来的参数
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
    //  设置本次需要发送几个通道的数据
    seekfree_assistant_oscilloscope_data.channel_num = 6;

    // 这里进发送了6个通道的数据，最大支持8通道
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
    // 有可能会在逐飞助手软件上看到波形更新不够连续，这是因为使用WIFI有不确定的延迟导致的

    // 解析上位机发送过来的参数
    seekfree_assistant_data_analysis();
#endif   // GPS_WIFI_SEND_FLAG
}

/**
 * @brief 调试专用：中断服务函数，发送原始IMU数据到调试工具
 */
void imu_oscilloscope_isr(void)
{
    imu963ra_get_acc();
    imu963ra_get_gyro();
    imu963ra_get_mag();

    /* 直接发送原始数据供调试 */
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
 * @brief 调试专用：中断服务函数，发送原始IMU数据到调试工具
 */
void icm45686_oscilloscope_isr(void)
{
    float meas_acc[3];
    float meas_gyro[3];
    float temp_degc;
    bsp_IcmGetRawData(meas_acc, meas_gyro, &temp_degc);

    /* 直接发送原始数据供调试 */
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

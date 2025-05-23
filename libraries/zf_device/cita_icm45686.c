/**
 * *****************************************************************************
 * @file        cita_icm45686.c
 * @brief       Implementation file for ICM-45686 device interface
 * @author      ciat-777 (juricek.chen@gmail.com)
 * @date        2025-02-25
 * @copyright   cita
 * *****************************************************************************
 */

#include "cita_icm45686.h"
#include "zf_common_debug.h"
#include "zf_driver_delay.h"
#include "zf_driver_soft_iic.h"
#include "zf_driver_spi.h"

/*----------------------------------variable----------------------------------*/   // 温度数据

static inv_imu_device_t imu_dev; /* Driver structure */

static uint8_t discard_accel_samples; /* Indicates how many accel samples should be discarded */
static uint8_t discard_gyro_samples;  /* Indicates how many gyro samples should be discarded */

#if ICM45686_USE_SOFT_IIC
static soft_iic_info_struct icm45686_iic_struct;
#endif


/*----------------------------------function----------------------------------*/
int si_print_error_if_any(int rc);
#define SI_CHECK_RC(rc)                                                          \
    do {                                                                         \
        if (si_print_error_if_any(rc))                                           \
        {                                                                        \
            INV_MSG(INV_MSG_LEVEL_ERROR, "At %s (line %d)", __FILE__, __LINE__); \
            system_delay_ms(100);                                                \
            return rc;                                                           \
        }                                                                        \
    } while (0)
/*
 * 错误代码处理
 */
int si_print_error_if_any(int rc)
{
    if (rc != 0)
    {
        switch (rc)
        {
        case INV_IMU_ERROR: printf("未指定的错误 (%d)", rc); break;
        case INV_IMU_ERROR_TRANSPORT: printf("传输层发生错误 (%d)", rc); break;
        case INV_IMU_ERROR_TIMEOUT: printf("操作未在预期时间内完成 (%d)", rc); break;
        case INV_IMU_ERROR_BAD_ARG: printf("提供的参数无效 (%d)", rc); break;
        case INV_IMU_ERROR_EDMP_BUF_EMPTY: printf("EDMP缓冲区为空 (%d)", rc); break;
        default: printf("未知错误 (%d)", rc); break;
        }
    }

    return rc;
}

/*******************************************************************************
 * 函数名称： icm45686_read_regs
 * 功能描述： 连续读取多个寄存器的值
 * 入口参数： reg: 起始寄存器地址 *buf: 数据存储指针, uint16_t len: 数据长度
 * 出口参数： 无
 * 作    者： Baxiange
 * 创建日期： 2024-07-25
 * 备    注： 使用SPI读取寄存器时要注意: 最高位为读写位，详见datasheet第50页
 *******************************************************************************/
static int icm45686_read_regs(uint8_t reg, uint8_t* buf, uint32_t len)
{
#if ICM45686_USE_HARD_SPI
    ICM45686_CS(0);
    spi_read_8bit_registers(ICM45686_SPI, reg | 0x80, buf, len);
    ICM45686_CS(1);
    return 0;
#elif ICM45686_USE_SOFT_IIC
    soft_iic_read_8bit_registers(&icm45686_iic_struct, reg, buf, len);
    return 0;
#else
    return -1;   // Indicate error if neither SPI nor IIC is used
#endif
}

/*******************************************************************************
 * 函数名称： icm45686_write_regs
 * 功能描述： 连续写入多个寄存器的值
 * 入口参数： reg: 起始寄存器地址 *buf: 数据存储指针, uint16_t len: 数据长度
 * 出口参数： 无
 * 作    者： Baxiange
 * 创建日期： 2024-07-25
 * 备    注： 使用SPI写入寄存器时要注意: 最高位为读写位，详见datasheet第50页
 *******************************************************************************/
static int icm45686_write_regs(uint8_t reg, const uint8_t* buf, uint32_t len)
{
#if ICM45686_USE_HARD_SPI
    ICM45686_CS(0);
    spi_write_8bit_registers(ICM45686_SPI, reg, buf, len);
    ICM45686_CS(1);
    return 0;
#elif ICM45686_USE_SOFT_IIC
    // I2C implementation for writing multiple registers
    for (uint32_t i = 0; i < len; i++)
    {
        soft_iic_write_8bit_register(&icm45686_iic_struct, reg + i, buf[i]);
    }
    return 0;
#else
    return -1;   // Indicate error if neither SPI nor IIC is used
#endif
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 ICM45686
// 参数说明     *config         ICM45686 配置信息 根据需要修改
// 返回参数     uint8           1-初始化失败 0-初始化成功
// 使用示例     icm45686_init(1,1,1);低噪声高功耗模式，使能加速度计和陀螺仪
// 备注信息     返回0x00表示初始化成功，返回0x01表示初始化失败
//  * @date        2025-02-25
//-------------------------------------------------------------------------------------------------------------------
uint8 icm45686_init(int use_ln, int accel_en, int gyro_en)
{
    int                      rc     = 0;
    uint8_t                  whoami = 0;       // 设备ID
    inv_imu_int_pin_config_t int_pin_config;   // 中断配置
    inv_imu_int_state_t      int_config;       // 中断状态配置
    imu_dev.transport.read_reg   = icm45686_read_regs;
    imu_dev.transport.write_reg  = icm45686_write_regs;
    imu_dev.transport.serif_type = UI_SPI4;
    imu_dev.transport.sleep_us   = system_delay_us;


#if ICM45686_USE_HARD_SPI
    ICM45686_CS(1);   // SPI不选中
    spi_init(
        ICM45686_SPI, SPI_MODE0, ICM45686_SPI_SPEED, ICM45686_SPC_PIN, ICM45686_SDI_PIN, ICM45686_SDO_PIN, SPI_CS_NULL);
    gpio_init(ICM45686_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
#elif ICM45686_USE_SOFT_IIC
    soft_iic_init(&icm45686_iic_struct, ICM45686_DEV_ADDR, ICM45686_SOFT_IIC_DELAY, ICM45686_SCL_PIN, ICM45686_SDA_PIN);
#endif
    system_delay_ms(100);   // 上电延时
    // 配置SPI的slew-rate，防止总线干扰
    if (imu_dev.transport.serif_type == UI_SPI3 || imu_dev.transport.serif_type == UI_SPI4)
    {
        drive_config0_t drive_config0;
        drive_config0.pads_spi_slew = DRIVE_CONFIG0_PADS_SPI_SLEW_TYP_10NS;
        rc |= inv_imu_write_reg(&imu_dev, DRIVE_CONFIG0, 1, (uint8_t*)&drive_config0);
        SI_CHECK_RC(rc);
        system_delay_us(2);   // 注册生效延时
    }
    // 检查设备ID（WHOAMI）
    rc |= inv_imu_get_who_am_i(&imu_dev, &whoami);
    SI_CHECK_RC(rc);
    if (whoami != INV_IMU_WHOAMI)
    {
        INV_MSG(INV_MSG_LEVEL_ERROR, "错误的WHOAMI值.", 0);
        INV_MSG(INV_MSG_LEVEL_ERROR, "  - 读取值 0x%02x", whoami);
        INV_MSG(INV_MSG_LEVEL_ERROR, "  - 期望值 0x%02x", INV_IMU_WHOAMI);
        return -1;
    }

    // 软件复位IMU
    rc |= inv_imu_soft_reset(&imu_dev);
    SI_CHECK_RC(rc);

    // 配置中断引脚：极性高，脉冲模式，推挽输出
    int_pin_config.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH;
    int_pin_config.int_mode     = INTX_CONFIG2_INTX_MODE_PULSE;
    int_pin_config.int_drive    = INTX_CONFIG2_INTX_DRIVE_PP;
    rc |= inv_imu_set_pin_config_int(&imu_dev, INV_IMU_INT1, &int_pin_config);
    SI_CHECK_RC(rc);

    // 配置中断：使能数据准备中断
    memset(&int_config, INV_IMU_DISABLE, sizeof(int_config));
    int_config.INV_UI_DRDY = INV_IMU_ENABLE;
    rc |= inv_imu_set_config_int(&imu_dev, INV_IMU_INT1, &int_config);
    SI_CHECK_RC(rc);

    // 设置加速度计和陀螺仪的量程
    rc |= inv_imu_set_accel_fsr(&imu_dev, ACCEL_CONFIG0_ACCEL_UI_FS_SEL_2_G);
    rc |= inv_imu_set_gyro_fsr(&imu_dev, GYRO_CONFIG0_GYRO_UI_FS_SEL_500_DPS);
    SI_CHECK_RC(rc);

    // 设置数据输出速率（ODR）
    rc |= inv_imu_set_accel_frequency(&imu_dev, ACCEL_CONFIG0_ACCEL_ODR_6400_HZ);
    rc |= inv_imu_set_gyro_frequency(&imu_dev, GYRO_CONFIG0_GYRO_ODR_6400_HZ);
    SI_CHECK_RC(rc);

    // 设置带宽 = 输出速率 / 4
    rc |= inv_imu_set_accel_ln_bw(&imu_dev, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_32);
    rc |= inv_imu_set_gyro_ln_bw(&imu_dev, IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_32);
    SI_CHECK_RC(rc);

    // 设置低功耗模式使用RCOSC时钟
    rc |= inv_imu_select_accel_lp_clk(&imu_dev, SMC_CONTROL_0_ACCEL_LP_CLK_RCOSC);
    SI_CHECK_RC(rc);

    // 设置功率模式
    if (use_ln)
    {
        if (accel_en) rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_LN);
        if (gyro_en) rc |= inv_imu_set_gyro_mode(&imu_dev, PWR_MGMT0_GYRO_MODE_LN);
    }
    else
    {
        if (accel_en) rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_LP);
        if (gyro_en) rc |= inv_imu_set_gyro_mode(&imu_dev, PWR_MGMT0_GYRO_MODE_LP);
    }

    // 丢弃启动时的样本
    if (accel_en) discard_accel_samples = (ACC_STARTUP_TIME_US / 20000) + 1;
    if (gyro_en) discard_gyro_samples = (GYR_STARTUP_TIME_US / 20000) + 1;

    SI_CHECK_RC(rc);
    return rc;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ICM45686 获取原始数据
// 参数说明     accel_mg        加速度计数据
// 参数说明     gyro_dps        陀螺仪数据
// 参数说明     temp_degc       温度数据
// 返回参数     int             0-成功 非0-失败
// 使用示例     float accel_mg[3], gyro_dps[3], temp_degc; bsp_IcmGetRawData(accel_mg, gyro_dps, &temp_degc);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
int bsp_IcmGetRawData(float accel_ms2[3], float gyro_dps[3], float* temp_degc)
{
    int                   rc = 0;
    inv_imu_sensor_data_t d;

    // 获取传感器数据
    rc |= inv_imu_get_register_data(&imu_dev, &d);
    SI_CHECK_RC(rc);

    // 将原始数据转换为实际单位
    accel_ms2[0] = (float)((d.accel_data[0] * 4 /* mg */) / 65536.0);
    accel_ms2[1] = (float)((d.accel_data[1] * 4 /* mg */) / 65536.0);
    accel_ms2[2] = (float)((d.accel_data[2] * 4 /* mg */) / 65536.0);
    gyro_dps[0]  = (float)((d.gyro_data[0] * 1000 /* dps */) / 65536.0);
    gyro_dps[1]  = (float)((d.gyro_data[1] * 1000 /* dps */) / 65536.0);
    gyro_dps[2]  = (float)((d.gyro_data[2] * 1000 /* dps */) / 65536.0);
    *temp_degc   = (float)(25 + (d.temp_data / 128.0));   // 温度转换为摄氏度
    return 0;

    // int                   rc = 0;
    // inv_imu_sensor_data_t d;

    // // 获取传感器数据
    // rc |= inv_imu_get_register_data(&imu_dev, &d);
    // SI_CHECK_RC(rc);

    // // 将原始加速度数据转换为 m/s?
    // // 原始公式：(d.accel_data * 4) / 32.768 得到的是 mg，
    // // 转换为 m/s? 使用：(mg / 1000) * 9.80665，即整体乘因子 (4*9.80665)/(32.768*1000)
    // const float acc_conversion = (4.0f) / (32.768f);
    // accel_ms2[0]               = d.accel_data[0] * acc_conversion;
    // accel_ms2[1]               = d.accel_data[1] * acc_conversion;
    // accel_ms2[2]               = d.accel_data[2] * acc_conversion;

    // // 陀螺仪数据转换保持为度每秒
    // gyro_dps[0] = (float)((d.gyro_data[0] * 1000) / 32768.0);
    // gyro_dps[1] = (float)((d.gyro_data[1] * 1000) / 32768.0);
    // gyro_dps[2] = (float)((d.gyro_data[2] * 1000) / 32768.0);

    // *temp_degc = (float)(25 + (d.temp_data / 128.0));   // 温度转换为摄氏度
    // return 0;
}
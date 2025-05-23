/**
 * *****************************************************************************
 * @file        cita_icm45686.h
 * @brief       Header file for ICM-45686 device interface
 * @author      ciat-777 (juricek.chen@gmail.com)
 * @date        2025-02-25
 * @copyright   cita
 * *****************************************************************************
 */

#ifndef CITA_ICM45686_H
#define CITA_ICM45686_H

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------include-----------------------------------*/
#include "inv_imu_driver.h"
#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"
#include "zf_driver_soft_iic.h"
#include "zf_driver_spi.h"

/*-----------------------------------macro------------------------------------*/
// ICM45686 SPI �� IIC �ӿڶ���
#define ICM45686_USE_HARD_SPI (1)   // ����ʹ��Ӳ�� SPI ģʽ
// #define ICM45686_USE_SOFT_IIC        (1)                 // ����ʹ����� IIC ģʽ

#if ICM45686_USE_HARD_SPI                           // Ӳ�� SPI ģʽ
#    define UI_SPI4 1                               /**< identifies 4-wire SPI interface. */
#    define ICM45686_SPI_SPEED (10 * 1000 * 1000)   // Ӳ�� SPI ����
#    define ICM45686_SPI (SPI_0)                    // Ӳ�� SPI ��
#    define ICM45686_SPC_PIN (SPI0_SCLK_P20_11)     // Ӳ�� SPI SCK ����
#    define ICM45686_SDI_PIN (SPI0_MOSI_P20_14)     // Ӳ�� SPI MOSI ����
#    define ICM45686_SDO_PIN (SPI0_MISO_P20_12)     // Ӳ�� SPI MISO ����
#    define ICM45686_CS_PIN (P20_13)                // Ӳ�� SPI CS Ƭѡ����
#    define ICM45686_CS(x) ((x) ? (gpio_high(ICM45686_CS_PIN)) : (gpio_low(ICM45686_CS_PIN)))
#elif ICM45686_USE_SOFT_IIC            // ��� IIC ģʽ
#    define ICM45686_DEV_ADDR (0x69)   // IIC ������ַ(SA0��ַλ��0����Ϊ0x68 ��1��Ϊ0x69)
#    define ICM45686_SCL_PIN (P07_7)   // ��� IIC SCL ����
#    define ICM45686_SDA_PIN (P07_6)   // ��� IIC SDA ����
#else
#    error "ICM45686 needs to select interface method"
#endif

#define INV_MSG(level, msg, ...) printf("%d," msg "\r\n", __LINE__, ##__VA_ARGS__)
static inv_imu_device_t imu_dev; /* Driver structure */

static uint8_t discard_accel_samples; /* Indicates how many accel samples should be discarded */
static uint8_t discard_gyro_samples;  /* Indicates how many gyro samples should be discarded */


/*----------------------------------typedef-----------------------------------*/


/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
int   bsp_IcmGetRawData(float accel_mg[3], float gyro_dps[3], float* temp_degc);
uint8 icm45686_init(int use_ln, int accel_en, int gyro_en);   // ��ʼ�� ICM45686

/*------------------------------------test------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* CITA_ICM45686_H */
/**
 * *****************************************************************************
 * @file        better_flash.h
 * @brief
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-27
 * @copyright   cita
 * *****************************************************************************
 */

#ifndef BETTER_FLASH_H
#define BETTER_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------include-----------------------------------*/
#include "stdbool.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"

#include "IfxCcu6_Timer.h"
#include "IfxScuEru.h"
#include "SysSe/Bsp/Bsp.h"
#include "ifxAsclin_reg.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/
// 支持的联合体类型枚举
typedef enum
{
    FLASH_DOUBLE,   // double类型（64位，占用2个单元）
    FLASH_FLOAT,    // float类型（32位）
    FLASH_UINT32,   // uint32类型（32位）
    FLASH_INT32,    // int32类型（32位）
    FLASH_UINT16,   // uint16类型（低16位，高16位清零）
    FLASH_INT16,    // int16类型（低16位，符号扩展）
    FLASH_UINT8,    // uint8类型（低8位，高24位清零）
    FLASH_INT8      // int8类型（低8位，符号扩展）
} FlashDataType;
/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void flash_init(void);
void Flash_Write_Data(uint32 section, uint32 page, const void *data, uint32 count, FlashDataType type);
void Flash_Read_Data(uint32 section, uint32 page, void *data, uint32 count, FlashDataType type);
void Flash_Print_Data(const void *data, uint32 count, FlashDataType type);
/*------------------------------------test------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* BETTER_FLASH_H */

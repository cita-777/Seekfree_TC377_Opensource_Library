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
// ֧�ֵ�����������ö��
typedef enum
{
    FLASH_DOUBLE,   // double���ͣ�64λ��ռ��2����Ԫ��
    FLASH_FLOAT,    // float���ͣ�32λ��
    FLASH_UINT32,   // uint32���ͣ�32λ��
    FLASH_INT32,    // int32���ͣ�32λ��
    FLASH_UINT16,   // uint16���ͣ���16λ����16λ���㣩
    FLASH_INT16,    // int16���ͣ���16λ��������չ��
    FLASH_UINT8,    // uint8���ͣ���8λ����24λ���㣩
    FLASH_INT8      // int8���ͣ���8λ��������չ��
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

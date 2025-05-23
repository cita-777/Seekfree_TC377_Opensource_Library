/**
 * *****************************************************************************
 * @file        better_flash.c
 * @brief
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-27
 * @copyright   cita
 * *****************************************************************************
 */

/*----------------------------------include-----------------------------------*/
#include "zf_common_headfile.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void flash_init(void)
{
    // 读取flash数据
}
/**
 * @brief      写入数据到Flash缓冲区（适配全部类型）
 *
 * @param       section  扇区
 * @param       page     页
 * @param       data     数据
 * @param       count    数据个数
 * @param       type     数据类型
 * @note        仅支持小端模式
 * @note        仅支持连续写入
 *
 */
void Flash_Write_Data(uint32 section, uint32 page, const void* data, uint32 count, FlashDataType type)
{
    flash_buffer_clear();   // 清空缓冲区（假设存在此函数）

    for (uint32 i = 0; i < count; i++)
    {
        switch (type)
        {
        case FLASH_DOUBLE:
        {
            // 将 double 拆分为两个 uint32（小端模式）
            const double* dbl_ptr = (const double*)data + i;
            const uint32* parts   = (const uint32*)dbl_ptr;

            // 占用两个连续的 flash_data_union 单元
            flash_union_buffer[2 * i].uint32_type     = parts[0];   // 低32位
            flash_union_buffer[2 * i + 1].uint32_type = parts[1];   // 高32位
            break;
        }
        case FLASH_FLOAT: flash_union_buffer[i].float_type = ((const float*)data)[i]; break;
        case FLASH_UINT32: flash_union_buffer[i].uint32_type = ((const uint32*)data)[i]; break;
        case FLASH_INT32: flash_union_buffer[i].int32_type = ((const int32*)data)[i]; break;
        case FLASH_UINT16: flash_union_buffer[i].uint16_type = ((const uint16*)data)[i]; break;
        case FLASH_INT16: flash_union_buffer[i].int16_type = ((const int16*)data)[i]; break;
        case FLASH_UINT8: flash_union_buffer[i].uint8_type = ((const uint8*)data)[i]; break;
        case FLASH_INT8: flash_union_buffer[i].int8_type = ((const int8*)data)[i]; break;
        }
    }

    flash_write_page_from_buffer(section, page);   // 写入Flash（假设存在此函数）
    // system_delay_ms(200);
    //  flash_buffer_clear();
}

/**
 * @brief      从Flash缓冲区读取数据（适配全部类型）
 *
 * @param       section  扇区
 * @param       page     页
 * @param       data     数据
 * @param       count    数据个数
 * @param       type     数据类型
 * @note        仅支持小端模式
 * @note        仅支持连续读取
 *
 */
void Flash_Read_Data(uint32 section, uint32 page, void* data, uint32 count, FlashDataType type)
{
    flash_buffer_clear();
    flash_read_page_to_buffer(section, page);   // 读取到缓冲区（假设存在此函数）

    for (uint32 i = 0; i < count; i++)
    {
        switch (type)
        {
        case FLASH_DOUBLE:
        {
            // 从两个连续的单元恢复 double
            uint32* parts = (uint32*)((double*)data + i);
            parts[0]      = flash_union_buffer[2 * i].uint32_type;       // 低32位
            parts[1]      = flash_union_buffer[2 * i + 1].uint32_type;   // 高32位
            break;
        }
        case FLASH_FLOAT: ((float*)data)[i] = flash_union_buffer[i].float_type; break;
        case FLASH_UINT32: ((uint32*)data)[i] = flash_union_buffer[i].uint32_type; break;
        case FLASH_INT32: ((int32*)data)[i] = flash_union_buffer[i].int32_type; break;
        case FLASH_UINT16: ((uint16*)data)[i] = flash_union_buffer[i].uint16_type; break;
        case FLASH_INT16: ((int16*)data)[i] = flash_union_buffer[i].int16_type; break;
        case FLASH_UINT8: ((uint8*)data)[i] = flash_union_buffer[i].uint8_type; break;
        case FLASH_INT8: ((int8*)data)[i] = flash_union_buffer[i].int8_type; break;
        }
    }
    // system_delay_ms(200);
    //  flash_buffer_clear();
}

/**
 * @brief      打印数据（适配全部类型）
 *
 * @param       data     数据
 * @param       count    数据个数
 * @param       type     数据类型
 *
 * @note        仅支持连续打印
 */
void Flash_Print_Data(const void* data, uint32 count, FlashDataType type)
{
    for (uint32 i = 0; i < count; i++)
    {
        switch (type)
        {
        case FLASH_DOUBLE:
        {
            const double value = ((const double*)data)[i];
            printf("[%u] double = %.15g\n", (unsigned int)i, value);
            break;
        }
        case FLASH_FLOAT: printf("[%u] float   = %.9f\n", (unsigned int)i, (double)((const float*)data)[i]); break;
        case FLASH_UINT32:
            printf("[%u] uint32  = %u\n", (unsigned int)i, (unsigned int)((const uint32*)data)[i]);
            break;
        case FLASH_INT32: printf("[%u] int32   = %d\n", (unsigned int)i, (int)((const int32*)data)[i]); break;
        case FLASH_UINT16:
            printf("[%u] uint16  = %u\n", (unsigned int)i, (unsigned int)((const uint16*)data)[i]);
            break;
        case FLASH_INT16: printf("[%u] int16   = %d\n", (unsigned int)i, (int)((const int16*)data)[i]); break;
        case FLASH_UINT8: printf("[%u] uint8   = %u\n", (unsigned int)i, (unsigned int)((const uint8*)data)[i]); break;
        case FLASH_INT8: printf("[%u] int8    = %d\n", (unsigned int)i, (int)((const int8*)data)[i]); break;
        }
    }
}
/*------------------------------------test------------------------------------*/
// /*****************************************int16测试******************************************************** */
//     int16 status_flags_int16[3] = {-273, 25, 100};
//     Flash_Write_Data(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, status_flags_int16, 3, FLASH_INT16);

//     int16 read_flags_int16[3];
//     Flash_Read_Data(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, read_flags_int16, 3, FLASH_INT16);
//     Flash_Print_Data(read_flags_int16, 3, FLASH_INT16);

//     /* 输出：
//     [0] int16   = -273
//     [1] int16   = 25
//     [2] int16   = 100
//     */
// /*****************************************float测试******************************************************** */
//     float status_flags_float[2] = {3.3f, 5.0f};
//     Flash_Write_Data(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, status_flags_float, 2, FLASH_FLOAT);

//     float read_flags_float[2];
//     Flash_Read_Data(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, read_flags_float, 2, FLASH_FLOAT);
//     Flash_Print_Data(read_flags_float, 2, FLASH_FLOAT);

//     /* 输出：
//     [0] float   = 3.300000
//     [1] float   = 5.000000
//     */
// /*****************************************uint8测试******************************************************** */
//     uint8 status_flags_uint8[4] = {0x01, 0x80, 0xFF, 0x00};
//     Flash_Write_Data(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, status_flags_uint8, 4, FLASH_UINT8);

//     uint8 read_flags_uint8[4];
//     Flash_Read_Data(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, read_flags_uint8, 4, FLASH_UINT8);
//     Flash_Print_Data(read_flags_uint8, 4, FLASH_UINT8);

//     /* 输出：
//     [0] uint8   = 1
//     [1] uint8   = 128
//     [2] uint8   = 255
//     [3] uint8   = 0
//     */
// /*****************************************double测试******************************************************** */
//     double sensor_data_double[2] = {3.141592653589793, 2.718281828459045};
//     Flash_Write_Data(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, sensor_data_double, 2, FLASH_DOUBLE);

//     double recovered_data_double[2];
//     Flash_Read_Data(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, recovered_data_double, 2, FLASH_DOUBLE);
//     Flash_Print_Data(recovered_data_double, 2, FLASH_DOUBLE);
//     /* 输出：
//     [0] double  = 3.141592653589793
//     [1] double  = 2.718281828459045
//     */
// /*****************************************uint32测试******************************************************** */
//     uint32 sensor_data_uint32[2] = {4294967295, 0};
//     Flash_Write_Data(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, sensor_data_uint32, 2, FLASH_UINT32);

//     uint32 recovered_data_uint32[2];
//     Flash_Read_Data(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, recovered_data_uint32, 2, FLASH_UINT32);
//     Flash_Print_Data(recovered_data_uint32, 2, FLASH_UINT32);
//     /* 输出：
//     [0] uint32  = 4294967295
//     [1] uint32  = 0
//     */
// /*****************************************uint16测试******************************************************** */
//     uint16 sensor_data_uint16[2] = {65535, 0};
//     Flash_Write_Data(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, sensor_data_uint16, 2, FLASH_UINT16);

//     uint16 recovered_data_uint16[2];
//     Flash_Read_Data(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, recovered_data_uint16, 2, FLASH_UINT16);
//     Flash_Print_Data(recovered_data_uint16, 2, FLASH_UINT16);
//     /* 输出：
//     [0] uint16  = 65535
//     [1] uint16  = 0
//     */

// /*****************************************int32测试******************************************************** */
//     int32 sensor_data_int32[2] = {2147483647, -2147483648};
//     Flash_Write_Data(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, sensor_data_int32, 2, FLASH_INT32);

//     int32 recovered_data_int32[2];
//     Flash_Read_Data(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, recovered_data_int32, 2, FLASH_INT32);
//     Flash_Print_Data(recovered_data_int32, 2, FLASH_INT32);
//     /* 输出：
//     [0] int32   = 2147483647
//     [1] int32   = -2147483648
//     */

// /*****************************************int8测试******************************************************** */
//     int8 sensor_data_int8[2] = {127, -128};
//     Flash_Write_Data(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, sensor_data_int8, 2, FLASH_INT8);

//     int8 recovered_data_int8[2];
//     Flash_Read_Data(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, recovered_data_int8, 2, FLASH_INT8);
//     Flash_Print_Data(recovered_data_int8, 2, FLASH_INT8);
//     /* 输出：
//     [0] int8    = 127
//     [1] int8    = -128
//     */
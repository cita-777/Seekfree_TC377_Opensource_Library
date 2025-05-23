/**
 * *****************************************************************************
 * @file        all_in_one.h
 * @brief       ���ܸ�ģ��
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-26
 * @copyright   cita
 * *****************************************************************************
 */

#ifndef ALL_IN_ONE_H
#define ALL_IN_ONE_H

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------include-----------------------------------*/
#include "board_resources.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/
typedef struct
{
    void (*proc)(void);
    uint8       is_enabled;
    const char *name;
} ProcHandler;
/*----------------------------------variable----------------------------------*/
// ����ȫ�ִ��������� .c �ļ��ж��壩
extern ProcHandler handlers[];
extern const int   handlers_count;
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void all_init();

// ��̬���ƽӿ�
void enable_handler(const char *name);
void disable_handler(const char *name);
/*------------------------------------test------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* ALL_IN_ONE_H */

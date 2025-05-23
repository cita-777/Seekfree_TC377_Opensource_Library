/**
 * *****************************************************************************
 * @file        gps.h
 * @brief       ���󱱴�tau1201 gpsģ������
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-27
 * @copyright   cita
 * *****************************************************************************
 */

#ifndef GPS_H
#define GPS_H

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------include-----------------------------------*/
#include "main_ctrl.h"
/*-----------------------------------macro------------------------------------*/
#define GPS_TYPE (GN42A)   // GPSģ������
// #define GPS_PIT (CCU61_CH0)
#define ALL_POINT_MAX 20          // ���õĲɼ�������(һ�������һ�Ծ�γ��)
#define FLASH_SECTION_INDEX (0)   // �洢�����õ�����
#define GPS_PAGE_INDEX (11)       // GPS�洢ҳ��
/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/
extern uint16 point_num;
// extern uint8 gps_pit_state;
#if Double_Record_FLAG
extern double Target_point[2][ALL_POINT_MAX];
// ���ڴ���ɼ���Ŀ�����Ϣ�����ں�����λ�ü���(��һ����γ�ȣ��ڶ����Ǿ���)
extern double Work_target_array[2][ALL_POINT_MAX];
// ���ڽ���flash�ж�ȡ��Ŀ�����������ݸ��ƹ�������������鱻��ֵʱ��������һ�к�Flash���޹�ϵ
#endif
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void gps_proc();
void gps_init();
void collect_and_store_points();
/*------------------------------------test------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* GPS_H */

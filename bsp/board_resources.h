/**
 * *****************************************************************************
 * @file        board_resources.h
 * @brief       板载资源
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-26
 * *****************************************************************************
 */

#ifndef BOARD_RESOURCES_H
#define BOARD_RESOURCES_H

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------include-----------------------------------*/

/*-----------------------------------macro------------------------------------*/
#define BUZZER_PIN (P33_10)

#define LED1 (P20_9)
#define LED2 (P20_8)
#define LED3 (P21_5)
#define LED4 (P21_4)

#define KEY1 (P20_6)
#define KEY2 (P20_7)
#define KEY3 (P11_2)
#define KEY4 (P11_3)

#define SWITCH1 (P33_11)
#define SWITCH2 (P33_12)
/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/
extern uint8 key1_flag;
extern uint8 key2_flag;
extern uint8 key3_flag;
extern uint8 key4_flag;

extern uint8 key_val;

extern uint8 switch1_flag;
extern uint8 switch2_flag;
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void Buzzer_init();                        // 蜂鸣器初始化
void Buzzer_check(int time1, int time2);   // 按键与LED初始化

void Key_init();       // 按键与LED初始化
void key_scan(void);   // 按键扫描
/*------------------------------------test------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* BOARD_RESOURCES_H */

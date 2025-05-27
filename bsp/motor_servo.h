/**
 * *****************************************************************************
 * @file        motor_servo.h
 * @brief       R��ģ����Ͷ�������Լ���Ӧ�ı�����
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-26
 * @copyright   cita
 * *****************************************************************************
 */

#ifndef MOTOR_SERVO_H
#define MOTOR_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------include-----------------------------------*/

/*-----------------------------------macro------------------------------------*/
#define MAX_DUTY (10)   // ������ռ�ձ�                                          // ��� MAX_DUTY% ռ�ձ�

/*----------------------------------drv8701----------------------------------*/
#define DIR1 (P21_4)             // �����������˿�
#define PWM1 (ATOM0_CH3_P21_5)   // PWM����˿�


#define ENCODER_1 (TIM2_ENCODER)               // ��������ʱ��
#define ENCODER_1_A (TIM2_ENCODER_CH1_P33_7)   // �����������˿�
#define ENCODER_1_B (TIM2_ENCODER_CH2_P33_6)   // �����������ֵ�˿�

/*----------------------------------bldc----------------------------------*/
#define DIR_CH1 (P02_6)   // �����������˿�
#define PWM_CH1 (ATOM0_CH7_P02_7)

#define ENCODER1_TIM (TIM5_ENCODER)              // ��������ʱ��
#define ENCODER1_PLUS (TIM5_ENCODER_CH1_P10_3)   // �����������˿�
#define ENCODER1_DIR (TIM5_ENCODER_CH2_P10_1)    // �����������ֵ�˿�
/*----------------------------------servo----------------------------------*/
#define SERVO_MOTOR_PWM (ATOM1_CH1_P33_9)   // ���������϶����Ӧ����
#define SERVO_MOTOR_FREQ (50)               // ���������϶��Ƶ��  �����ע�ⷶΧ 50-300
#define SERVO_MOTOR_L_MAX (70)              // ���������϶�����Χ �Ƕ�
#define SERVO_MOTOR_MID (90)                // �����ֵ�Ƕ�
#define SERVO_MOTOR_R_MAX (110)             // ���������϶�����Χ �Ƕ�
#define SERVO_MOTOR_DUTY(x) ((float)PWM_DUTY_MAX / (1000.0 / (float)SERVO_MOTOR_FREQ) * (0.5 + (float)(x) / 90.0))
#define SERVO_PIT (CCU61_CH1)
#define ENCODER_PIT (CCU61_CH0)
#if (SERVO_MOTOR_FREQ < 50 || SERVO_MOTOR_FREQ > 300)
#    error "SERVO_MOTOR_FREQ ERROE!"
#endif
/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/
extern double motor_duty;
extern bool   dir;
extern int16  encoder_data_1;
// extern uint8 motor_encoder_pit_state;
extern uint32 cumulative_encoder_data_1;
extern double servo_value;
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void motor_encoder_proc(void);   // ����������жϴ���

void drv8701_motor_init(void);         // ��ˢ�����ʼ��
void drv8701_motor_set(double duty);   // ��ˢ�������
void drv8701_motor_test(void);         // ��ˢ�������

void bldc_motor_init(void);         // ��ˢ�����ʼ��
void bldc_motor_set(double duty);   // ��ˢ�������
void bldc_motor_speed_ctrl(double speed);
// void bldc_motor_test(void);         // ��ˢ�������

void servo_init();              // �����ʼ��
void servo_set(double angle);   // �������
void servo_set_pd(float target_angle);
void servo_test(void);   // �������
/*------------------------------------test------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_SERVO_H */

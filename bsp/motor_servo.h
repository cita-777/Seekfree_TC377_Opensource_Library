/**
 * *****************************************************************************
 * @file        motor_servo.h
 * @brief       R车模电机和舵机驱动以及相应的编码器
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
#define MAX_DUTY (10)   // 电机最大占空比                                          // 最大 MAX_DUTY% 占空比

/*----------------------------------drv8701----------------------------------*/
#define DIR1 (P21_4)             // 电机方向输出端口
#define PWM1 (ATOM0_CH3_P21_5)   // PWM输出端口


#define ENCODER_1 (TIM2_ENCODER)               // 编码器定时器
#define ENCODER_1_A (TIM2_ENCODER_CH1_P33_7)   // 编码器计数端口
#define ENCODER_1_B (TIM2_ENCODER_CH2_P33_6)   // 编码器方向采值端口

/*----------------------------------bldc----------------------------------*/
#define DIR_CH1 (P02_6)   // 电机方向输出端口
#define PWM_CH1 (ATOM0_CH7_P02_7)

#define ENCODER1_TIM (TIM5_ENCODER)              // 编码器定时器
#define ENCODER1_PLUS (TIM5_ENCODER_CH1_P10_3)   // 编码器计数端口
#define ENCODER1_DIR (TIM5_ENCODER_CH2_P10_1)    // 编码器方向采值端口
/*----------------------------------servo----------------------------------*/
#define SERVO_MOTOR_PWM (ATOM1_CH1_P33_9)   // 定义主板上舵机对应引脚
#define SERVO_MOTOR_FREQ (50)               // 定义主板上舵机频率  请务必注意范围 50-300
#define SERVO_MOTOR_L_MAX (70)              // 定义主板上舵机活动范围 角度
#define SERVO_MOTOR_MID (90)                // 舵机中值角度
#define SERVO_MOTOR_R_MAX (110)             // 定义主板上舵机活动范围 角度
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
void motor_encoder_proc(void);   // 电机编码器中断处理

void drv8701_motor_init(void);         // 有刷电机初始化
void drv8701_motor_set(double duty);   // 有刷电机驱动
void drv8701_motor_test(void);         // 有刷电机测试

void bldc_motor_init(void);         // 无刷电机初始化
void bldc_motor_set(double duty);   // 无刷电机驱动
void bldc_motor_speed_ctrl(double speed);
// void bldc_motor_test(void);         // 无刷电机测试

void servo_init();              // 舵机初始化
void servo_set(double angle);   // 舵机驱动
void servo_set_pd(float target_angle);
void servo_test(void);   // 舵机测试
/*------------------------------------test------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_SERVO_H */

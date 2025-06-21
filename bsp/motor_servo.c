/**
 * *****************************************************************************
 * @file        motor_servo.c
 * @brief       R��ģ����Ͷ�������Լ���Ӧ�ı�����
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-26
 * @copyright   cita
 * *****************************************************************************
 */

/*----------------------------------include-----------------------------------*/
#include "zf_common_headfile.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/
double motor_duty                = 0;
bool   dir                       = true;
int16  encoder_data_1            = 0;
uint32 cumulative_encoder_data_1 = 0;
// uint8 motor_encoder_pit_state = 0;
volatile uint8_t encoder_updated = 0;
double           servo_value     = SERVO_MOTOR_MID;
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/

void motor_encoder_proc()
{
    // motor_encoder_pit_state = 0;
    encoder_data_1 = -encoder_get_count(ENCODER_1);
    cumulative_encoder_data_1 += encoder_data_1;
    encoder_clear_count(ENCODER_1);   // ��ձ���������
    encoder_updated = 1;              // ���ñ�־����ʾ�������Ѹ���
                                      // drv8701_motor_test();
    // printf("encoder_data_1: %d\r\n", encoder_data_1);
    //  printf("cumulative_encoder_data_1: %d\r\n", cumulative_encoder_data_1);
#if MOTOR_ENCODER_WIFI_SEND_FLAG
    // wifi_spi_send();
#endif
    disable_handler("motor_encoder");
}
void drv8701_motor_init()   // ����ͱ�������ʼ��
{
    motor_duty = 0;
    gpio_init(DIR1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PWM1, 17000, 0);
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);   // ��ʼ��������ģ�������� �������������ģʽ
}

void drv8701_motor_set(double duty)   // �������
{
    motor_duty = duty;
    if (motor_duty > MAX_DUTY)
    {
        motor_duty = MAX_DUTY;
    }
    else if (motor_duty < -MAX_DUTY)
    {
        motor_duty = -MAX_DUTY;
    }

    // ��ӵ������
    // printf("Motor control: duty=%.2f, limited_duty=%.2f\n", duty, motor_duty);

    if (motor_duty >= 0)   // ��ת
    {
        gpio_set_level(DIR1, GPIO_LOW);                            // ���Ը�ΪHIGH
        pwm_set_duty(PWM1, (motor_duty) * (PWM_DUTY_MAX / 100));   // ����ռ�ձ�
        // printf("Forward: DIR=HIGH, PWM=%.0f\n", (motor_duty) * (PWM_DUTY_MAX / 100));
    }
    else   // ��ת
    {
        gpio_set_level(DIR1, GPIO_HIGH);                            // ���Ը�ΪLOW
        pwm_set_duty(PWM1, (-motor_duty) * (PWM_DUTY_MAX / 100));   // ����ռ�ձ�
        // printf("Reverse: DIR=LOW, PWM=%.0f\n", (-motor_duty) * (PWM_DUTY_MAX / 100));
    }
}
void drv8701_motor_speed_ctrl(double speed)
{
    // ���������û�и��£�ֱ�ӷ���
    if (!encoder_updated)
    {
        return;
    }

    double target_speed  = speed;
    double current_speed = encoder_data_1 / 31.85;   // ���������������������ת��Ϊ��������ʾ�ٶ�
    double speed_gap     = target_speed - current_speed;

    // ���ٶȴ���
    if (fabs(target_speed) < 0.1)
    {
        drv8701_motor_set(0);
        PID_MOTOR.out   = 0;   // ����PID���
        encoder_updated = 0;
        return;
    }

    // ����PID����
    PID_MOTOR.kp = 2.8;    // ����ϵ��
    PID_MOTOR.ki = 0.02;   // ����ϵ��
    PID_MOTOR.kd = 0.3;    // ΢��ϵ��

    // ʹ��PID�������������
    double out = PidIncCtrl(&PID_MOTOR, speed_gap);

    // ���������Χ
    if (out > MAX_DUTY)
    {
        out = MAX_DUTY;
    }
    if (out < -MAX_DUTY)
    {
        out = -MAX_DUTY;
    }

#if PID_WIFI_SEND_FLAG
    // ������ݵ�ʾ������ʽ
    seekfree_assistant_oscilloscope_data.data[0]     = target_speed;    // ͨ��1��Ŀ���ٶ�
    seekfree_assistant_oscilloscope_data.data[1]     = current_speed;   // ͨ��2����ǰ�ٶ�
    seekfree_assistant_oscilloscope_data.data[2]     = speed_gap;       // ͨ��3���ٶȲ�
    seekfree_assistant_oscilloscope_data.data[3]     = out;             // ͨ��4��PID���
    seekfree_assistant_oscilloscope_data.channel_num = 4;               // ʹ��4��ͨ��

    // �������ݵ���λ����������ܵķ�������
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
    seekfree_assistant_data_analysis();
#endif

    encoder_updated = 0;
    drv8701_motor_set(out);
}
void bldc_motor_init()   // ����ͱ�������ʼ��
{
    motor_duty = 0;
    pwm_init(PWM_CH1, 1000, 0);                                    // PWM ͨ��1 ��ʼ��Ƶ��1KHz ռ�ձȳ�ʼΪ0
    gpio_init(DIR_CH1, GPO, GPIO_HIGH, GPO_PUSH_PULL);             // ��ʼ����������������
    encoder_dir_init(ENCODER1_TIM, ENCODER1_PLUS, ENCODER1_DIR);   // ��ʼ����������ֵ���ż���ʱ��
    PidInit(&PID_Init);
}

void bldc_motor_set(double duty)   // �������
{
    motor_duty = duty;
    if (motor_duty > MAX_DUTY)
    {
        motor_duty = MAX_DUTY;
    }
    else if (motor_duty < -MAX_DUTY)
    {
        motor_duty = -MAX_DUTY;
    }
    if (motor_duty >= 0)
    {
        gpio_set_level(DIR_CH1, GPIO_HIGH);                           // DIR����ߵ�ƽ
        pwm_set_duty(PWM_CH1, (motor_duty) * (PWM_DUTY_MAX / 100));   // ����ռ�ձ�
    }
    else
    {
        gpio_set_level(DIR_CH1, GPIO_LOW);                             // DIR����͵�ƽ
        pwm_set_duty(PWM_CH1, (-motor_duty) * (PWM_DUTY_MAX / 100));   // ����ռ�ձ�
    }
}

void bldc_motor_speed_ctrl(double speed)
{
    // ��¼��һ�ε�Ŀ���ٶȣ����ڼ���ٶȱ仯
    static double last_target_speed = 0;

    // ���������û�и��£�ֱ�ӷ���
    if (!encoder_updated)
    {
        return;
    }

    double target_speed  = speed;
    double current_speed = encoder_data_1;
    double speed_gap     = target_speed - current_speed;

    // ����ͣ����������ӷ����ٶȱ�Ϊ���ٶ�
    if (fabs(last_target_speed) > 0.5 && fabs(target_speed) < 0.1)
    {
        // ����PID״̬
        PID_MOTOR.out             = 0;
        PID_MOTOR.last_error      = 0;
        PID_MOTOR.last_derivative = 0;

        // ��ʱ�����ƶ���ͨ������ռ�ձȿ��ټ��٣�
        double brake_duty = (last_target_speed > 0) ? -8.0 : 8.0;
        bldc_motor_set(brake_duty);
        // systick_delay_ms(50);   // ��ʱ�䷴���ƶ�

        // ��ȫֹͣ
        bldc_motor_set(0);
        encoder_updated   = 0;
        last_target_speed = target_speed;
        return;
    }

    // ��ͨ���ٶȴ���
    if (fabs(target_speed) < 0.1)
    {
        // ����ٶ��Ѿ��ܵͣ�ֱ��ֹͣ
        if (fabs(current_speed) < 2.0)
        {
            bldc_motor_set(0);
            PID_MOTOR.out     = 0;   // ����PID���
            encoder_updated   = 0;
            last_target_speed = target_speed;
            return;
        }
        // ����ʹ����ǿ�ƶ�����
        PID_MOTOR.kp = 1.5;   // �������ϵ��
        PID_MOTOR.ki = 0;     // ���û�����
        PID_MOTOR.kd = 0.3;   // ����΢��ϵ��
    }
    else
    {
        // �����Ӧ�ٶȵĲ���
        PID_MOTOR.kp = 2.5;    // �����߱���ϵ��
        PID_MOTOR.ki = 0.02;   // ��΢��߻���ϵ��
        PID_MOTOR.kd = 0.2;    // ����΢��ϵ��
    }

    // ʹ��PID�������������
    double out = PidIncCtrl(&PID_MOTOR, speed_gap);

    // �򻯵ľ�Ħ������������������
    double static_friction = 8.0;   // ��߾�Ħ������ֵ
    if (fabs(out) < static_friction && fabs(speed_gap) > 2.0)
    {
        out = (out >= 0) ? static_friction : -static_friction;
    }

    // ���������Χ
    if (out > MAX_DUTY)
    {
        out = MAX_DUTY;
    }
    if (out < -MAX_DUTY)
    {
        out = -MAX_DUTY;
    }

#if PID_WIFI_SEND_FLAG
    // ������ݵ�ʾ������ʽ
    seekfree_assistant_oscilloscope_data.data[0]     = target_speed;    // ͨ��1��Ŀ���ٶ�
    seekfree_assistant_oscilloscope_data.data[1]     = current_speed;   // ͨ��2����ǰ�ٶ�
    seekfree_assistant_oscilloscope_data.data[2]     = speed_gap;       // ͨ��3���ٶȲ�
    seekfree_assistant_oscilloscope_data.data[3]     = out;             // ͨ��4��PID���
    seekfree_assistant_oscilloscope_data.channel_num = 4;               // ʹ��4��ͨ��

    // �������ݵ���λ����������ܵķ�������
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
    seekfree_assistant_data_analysis();
#endif

    encoder_updated = 0;
    bldc_motor_set(out);
    last_target_speed = target_speed;
}
void servo_init()   // �����ʼ��
{
    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY(SERVO_MOTOR_MID));
    PidInit(&PID_Init);
}

void servo_set(double angle)   // �������
{
    servo_value = angle;
    if (servo_value < SERVO_MOTOR_L_MAX)
    {
        servo_value = SERVO_MOTOR_L_MAX;
    }
    if (servo_value > SERVO_MOTOR_R_MAX)
    {
        servo_value = SERVO_MOTOR_R_MAX;
    }
    // Ϊ�˷�ת����Ƕȷ�����Ҫ2 * SERVO_MOTOR_MID - angle������ʵangle
    pwm_set_duty(SERVO_MOTOR_PWM, (uint32)SERVO_MOTOR_DUTY(2 * SERVO_MOTOR_MID - servo_value));
    // printf("servo_value: %f\r\n", servo_value);
}
/**
 * @brief ����Ƕ�PD�ջ����ƺ���
 * @param target_angle Ŀ��Ƕ�(��)
 * @details ͨ��IMU�Ƕȷ�����ʹ��PD�����������Ƴ���ת��ָ���Ƕ�
 *          ������ƫ��Ŀ��Ƕ�ʱ��PD���������������ֵ
 *          ʹ���ת�򣬽�������������ȷ�Ƕ�
 */
void servo_set_pd(float target_angle)
{
    // ��ȡ��ǰIMUƫ����
    float current_angle = g_imu_angle.yaw;

    // ������Ŀ��Ƕ� - ��ǰ�Ƕ�
    float error = target_angle - current_angle;
    // �ǶȲ��һ���������180����������
    while (error > 180.0f)
    {
        error -= 360.0f;
    }
    while (error < -180.0f)
    {
        error += 360.0f;
    }
    // ʹ��PD�������������ֵ
    double pd_output = PidLocCtrl(&PID_IMU, error);
    if (pd_output > SERVO_MOTOR_R_MAX - SERVO_MOTOR_MID)
    {
        pd_output = SERVO_MOTOR_R_MAX - SERVO_MOTOR_MID;   // �����������ֵ
    }
    else if (pd_output < SERVO_MOTOR_L_MAX - SERVO_MOTOR_MID)
    {
        pd_output = SERVO_MOTOR_L_MAX - SERVO_MOTOR_MID;   // ������С����ֵ
    }
    // ���ö��ֵΪ�м�ֵ����PD����ֵ
    servo_set(SERVO_MOTOR_MID - pd_output);

    // ��ѡ���������ݵ���λ��ʾ������ʾ
#if PD_WIFI_SEND_FLAG
    // ������ݵ�ʾ������ʽ
    seekfree_assistant_oscilloscope_data.data[0]     = target_angle;    // ͨ��1��Ŀ��Ƕ�
    seekfree_assistant_oscilloscope_data.data[1]     = current_angle;   // ͨ��2����ǰIMU�Ƕ�
    seekfree_assistant_oscilloscope_data.data[2]     = error;           // ͨ��3���Ƕ����
    seekfree_assistant_oscilloscope_data.data[3]     = pd_output;       // ͨ��4��PD���
    seekfree_assistant_oscilloscope_data.data[4]     = servo_value;     // ͨ��5�������������ֵ
    seekfree_assistant_oscilloscope_data.channel_num = 5;               // ʹ��5��ͨ��

    // �������ݵ���λ����������ܵķ�������
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
#endif
}
/*------------------------------------test------------------------------------*/
void drv8701_motor_test()   // �������
{
    key_scan();
    if (key1_flag)
    {
        key1_flag = 0;
        motor_duty += 10;
    }
    if (key2_flag)
    {
        key2_flag = 0;
        motor_duty -= 10;
    }
    if (key3_flag)
    {
        key3_flag = 0;
        motor_duty += 1;
    }
    if (key4_flag)
    {
        key4_flag = 0;
        motor_duty -= 1;
    }
    // ��ȡ��������λ�����͹����Ĳ���
    // motor_duty = seekfree_assistant_parameter[4];
    drv8701_motor_set(motor_duty);
    ips_show_int(100, 16 * 3, motor_duty, 5);
    ips_show_int(100, 16 * 4, encoder_data_1, 9);
    ips_show_int(100, 16 * 5, cumulative_encoder_data_1, 9);
}

void servo_test()   // �������
{

    key_scan();
    if (key1_flag)
    {
        key1_flag = 0;
        servo_value += 10;
    }
    if (key2_flag)
    {
        key2_flag = 0;
        servo_value -= 10;
    }
    if (key3_flag)
    {
        key3_flag = 0;
        servo_value += 1;
        // angle = SERVO_MOTOR_L_MAX; // �����
    }
    if (key4_flag)
    {
        key4_flag = 0;
        servo_value -= 1;
        // angle = SERVO_MOTOR_R_MAX; // �Ҵ���
    }
    servo_set(servo_value);
    ips_show_int(100, 16 * 3, servo_value, 5);
}

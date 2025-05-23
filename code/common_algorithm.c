/**
 * *****************************************************************************
 * @file        algorithm.c
 * @brief       �㷨ʵ��
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-21
 * @copyright   cita
 * *****************************************************************************
 */

/*----------------------------------include-----------------------------------*/
#include "zf_common_headfile.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/
pid_param_t PID_GPS;
pid_param_t PID_IMU;
pid_param_t PID_Init;
pid_param_t PID_MOTOR;

move_filter_struct GPS_Direction_filter;   // ����ƽ���˲��ṹ��

/*----------------------------------variable----------------------------------*/
extern float Daty_Z;   // �����ǻ��ֵõ���ֵ(��Ƶ����)
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/

/*************************************************************************
 *  �������ƣ�float constrain_float(float amt, float low, float high)
 *  ����˵�����޷�����
 *  ����˵����
 * @param    amt   �� ����
 * @param    low   �� ���ֵ
 * @param    high  �� ���ֵ
 *  �������أ���
 *  �޸�ʱ�䣺2020��4��1��
 *  ��    ע��
 *************************************************************************/
float constrain_float(float amt, float low, float high)
{
    return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

/*************************************************************************
 *  �������ƣ�void PidInit(pid_param_t * pid)
 *  ����˵����PID��ʼ��
 *  ����˵����
 * @param    pid   �� PID�ṹ��
 *  �������أ���
 *  �޸�ʱ�䣺2025��1��21��
 *  ��    ע��
 *************************************************************************/
void PidInit(pid_param_t* pid)
{
    pid->kp              = 0;
    pid->ki              = 0;
    pid->kd              = 0;
    pid->imax            = 0;
    pid->out_p           = 0;
    pid->out_i           = 0;
    pid->out_d           = 0;
    pid->out             = 0;
    pid->integrator      = 0;
    pid->last_error      = 0;
    pid->last_derivative = 0;
    pid->last_t          = 0;
}

/*************************************************************************
 *  �������ƣ�float constrain_float(float amt, float low, float high)
 *  ����˵����pidλ��ʽ���������
 *  ����˵����
 * @param    pid     pid����
 * @param    error   pid�������
 *  �������أ�PID������
 *  �޸�ʱ�䣺2020��4��1��
 *  ��    ע��
 *************************************************************************/
float PidLocCtrl(pid_param_t* pid, float error)
{
    //    PID_GPS.kp=1;
    //    PID_GPS.kd=1.05;


    PID_GPS.kp = 1.1;
    PID_GPS.kd = 5;

    PID_IMU.kp = 1;   // 1.15
    PID_IMU.kd = 7;

    /* �ۻ���� */
    pid->integrator += error;

    /* ����޷� */
    constrain_float(pid->integrator, -pid->imax, pid->imax);


    pid->out_p = pid->kp * error;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd * (error - pid->last_error);

    pid->last_error = error;

    pid->out = pid->out_p + pid->out_i + pid->out_d;

    //    if(pid->out>SERVO_MOTOR_LMAX)
    //    {pid->out=SERVO_MOTOR_LMAX;}
    //    if(pid->out<SERVO_MOTOR_RMIN)
    //    {pid->out=SERVO_MOTOR_RMIN;}
    return pid->out;
    //    printf("OUT:%d\n",pid->out);
}

/*************************************************************************
 *  �������ƣ�float constrain_float(float amt, float low, float high)
 *  ����˵����pid����ʽ���������
 *  ����˵����
 * @param    pid     pid����
 * @param    error   pid�������
 *  �������أ�PID������   ע���������Ѿ��������ϴν��
 *  �޸�ʱ�䣺2020��4��1��
 *  ��    ע��
 *************************************************************************/
float PidIncCtrl(pid_param_t* pid, float error)
{
    PID_MOTOR.kp = 0.3;   // ������Ҫ�ĳ�2.5
    PID_MOTOR.ki = 0.02;
    PID_MOTOR.kd = 0.15;



    pid->out_p = pid->kp * (error - pid->last_error);
    pid->out_i = pid->ki * error;
    pid->out_d = pid->kd * ((error - pid->last_error) - pid->last_derivative);

    pid->last_derivative = error - pid->last_error;
    pid->last_error      = error;

    pid->out += pid->out_p + pid->out_i + pid->out_d;
    return pid->out;
}
//-------------------------------------------------------------------------------------------------------------------
// �������     GPS_IMU_COM_filtering
// ����˵��     ��
// ʹ��ʾ��     GPS_IMU_COM_filtering();
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
// void GPS_IMU_COM_filtering()
// {
//     float K=0.9;//����ϵ��
//     float Fusion_angle;//�ںϺ�ĺ����

//     Fusion_angle=K*Daty_Z+(1-K)*gps_tau1201.direction;//�����ֵ�YAW�����GPS��direction���л����ں�

//     printf("\r\nFusion_angle---%f",Fusion_angle);
// }

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ƽ���˲�����
// ����˵��     move_filter     �ṹ�崫��
// ����˵��     new_data        �����������
// ʹ��ʾ��     (&speed_filter, 400);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void move_filter_calc(move_filter_struct* move_filter, int32 new_data)
{
    // �����µ���ֵ ��ȥ��ĩβ����ֵ ������µĺ�
    move_filter->data_sum = move_filter->data_sum + new_data - move_filter->data_buffer[move_filter->index];
    // ������ƽ��ֵ
    move_filter->data_average = move_filter->data_sum / move_filter->buffer_size;

    // ������д�뻺����
    move_filter->data_buffer[move_filter->index] = new_data;
    move_filter->index++;
    if (move_filter->buffer_size <= move_filter->index)
    {
        move_filter->index = 0;
    }
}
//-------------------------------------------------------------------------------------------------------------------
// �������     ����ƽ���˲���ʼ��
// ����˵��     move_filter     �ṹ�崫��
// ʹ��ʾ��     move_filter_init(&speed_filter);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void move_filter_init(move_filter_struct* move_filter)
{
    move_filter->data_average = 0;
    move_filter->data_sum     = 0;
    move_filter->index        = 0;
    // ���û�������С
    move_filter->buffer_size = MOVE_AVERAGE_SIZE;

    uint8 i;
    for (i = 0; i < move_filter->buffer_size; i++)
    {
        move_filter->data_buffer[i] = 0;
    }
}
//-------------------------------------------------------------------------------------------------------------------
// �������     ���㻺��������Ԫ�ظ���
// ����˵��     ��
// ʹ��ʾ��     ARRAY_Element_Calculation();
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
int ARRAY_Element_Calculation()
{
    int i     = 0;   // ���������
    int count = 0;   // ��������Ԫ�ظ���������

    // �������飬��������Ԫ�ظ���
    while (i < 1024 && flash_union_buffer[i].uint32_type != (-1) && flash_union_buffer[i].uint32_type != (0))
    {
        count++;
        i++;
    }
    //    printf("\r\n count=%d",count);
    return count;
}

// �����ַ��˲�����ʼ��
void Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent, Butter_Parameter* LPF)
{
    float fr  = sample_frequent / cutoff_frequent;
    float ohm = my_tan_rad(PI / fr);
    float c   = 1.0f + 2.0f * my_cos_rad(PI / 4.0f) * ohm + ohm * ohm;
    if (cutoff_frequent <= 0.0f)
    {
        // no filtering
        return;
    }
    LPF->b[0] = ohm * ohm / c;
    LPF->b[1] = 2.0f * LPF->b[0];
    LPF->b[2] = LPF->b[0];
    LPF->a[0] = 1.0f;
    LPF->a[1] = 2.0f * (ohm * ohm - 1.0f) / c;
    LPF->a[2] = (1.0f - 2.0f * my_cos_rad(PI / 4.0f) * ohm + ohm * ohm) / c;
}

// ���׵�ͨ�����ַ��˲���
float LPButterworth(float curr_input, Butter_BufferData* Buffer, Butter_Parameter* Parameter)
{
    if (Buffer->Output_Butter[0] == 0 && Buffer->Output_Butter[1] == 0 && Buffer->Output_Butter[2] == 0 &&
        Buffer->Input_Butter[0] == 0 && Buffer->Input_Butter[1] == 0 && Buffer->Input_Butter[2] == 0)
    {
        Buffer->Output_Butter[0] = curr_input;
        Buffer->Output_Butter[1] = curr_input;
        Buffer->Output_Butter[2] = curr_input;
        Buffer->Input_Butter[0]  = curr_input;
        Buffer->Input_Butter[1]  = curr_input;
        Buffer->Input_Butter[2]  = curr_input;
        return curr_input;
    }

    /* ���ٶȼ�Butterworth�˲� */
    /* ��ȡ����x(n) */
    Buffer->Input_Butter[2] = curr_input;
    /* Butterworth�˲� */
    Buffer->Output_Butter[2] = Parameter->b[0] * Buffer->Input_Butter[2] + Parameter->b[1] * Buffer->Input_Butter[1] +
                               Parameter->b[2] * Buffer->Input_Butter[0] - Parameter->a[1] * Buffer->Output_Butter[1] -
                               Parameter->a[2] * Buffer->Output_Butter[0];
    /* x(n) ���б��� */
    Buffer->Input_Butter[0] = Buffer->Input_Butter[1];
    Buffer->Input_Butter[1] = Buffer->Input_Butter[2];
    /* y(n) ���б��� */
    Buffer->Output_Butter[0] = Buffer->Output_Butter[1];
    Buffer->Output_Butter[1] = Buffer->Output_Butter[2];

    for (uint16_t i = 0; i < 3; i++)
    {
        if (my_isnan(Buffer->Output_Butter[i]) == 1 || my_isnan(Buffer->Input_Butter[i]) == 1)
        {
            Buffer->Output_Butter[0] = curr_input;
            Buffer->Output_Butter[1] = curr_input;
            Buffer->Output_Butter[2] = curr_input;
            Buffer->Input_Butter[0]  = curr_input;
            Buffer->Input_Butter[1]  = curr_input;
            Buffer->Input_Butter[2]  = curr_input;
            return curr_input;
        }
    }
    return Buffer->Output_Butter[2];
}
// RLS ���º�������Ե�����ģ�ͣ��ع�����Ϊ1��
float RLS_Update(RLSState* state, float measurement)
{
    float K     = state->P / (state->lambda + state->P);   // ��������
    float error = measurement - state->bias;
    state->bias += K * error;                        // ����ƫ�ù���
    state->P = (1 - K) * state->P / state->lambda;   // ����Э����
    return state->bias;
}
/*------------------------------------test------------------------------------*/

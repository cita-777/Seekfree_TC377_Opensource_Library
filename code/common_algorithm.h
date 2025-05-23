/**
 * *****************************************************************************
 * @file        algorithm.h
 * @brief       �㷨ʵ��
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-21
 * @copyright   cita
 * *****************************************************************************
 */

#ifndef ALGORITHM_H
#define ALGORITHM_H

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------include-----------------------------------*/

/*-----------------------------------macro------------------------------------*/
// ����һά����Ԫ�ظ����ĺ�
#define ARRAY_LENGTH(x) (sizeof(x) / sizeof(x[0]))
// ���廬��ƽ���˲���������С
#define MOVE_AVERAGE_SIZE 32
/*----------------------------------typedef-----------------------------------*/
typedef struct
{
    float kp;     // P
    float ki;     // I
    float kd;     // D
    float imax;   // �����޷�

    float out_p;   // KP���
    float out_i;   // KI���
    float out_d;   // KD���
    float out;     // pid���

    float         integrator;        //< ����ֵ
    float         last_error;        //< �ϴ����
    float         last_derivative;   //< �ϴ���������ϴ����֮��
    unsigned long last_t;            //< �ϴ�ʱ��
} pid_param_t;                       // PID�ṹ��

typedef struct
{
    uint8 index;                            // �±�
    uint8 buffer_size;                      // buffer��С
    int32 data_buffer[MOVE_AVERAGE_SIZE];   // ������
    int32 data_sum;                         // ���ݺ�
    int32 data_average;                     // ����ƽ��ֵ
} move_filter_struct;                       // ����ƽ���˲��ṹ��
// ����butterworth�˲���
typedef struct
{
    float a[3];
    float b[3];
} Butter_Parameter;
typedef struct
{
    float Input_Butter[3];
    float Output_Butter[3];
} Butter_BufferData;
// ������ RLS ״̬�ṹ�嶨��
typedef struct
{
    float bias;     // ��ǰ���Ƶ�ƫ��
    float P;        // ����Э����
    float lambda;   // �������ӣ�����ȡ1.0f��
} RLSState;
/*----------------------------------variable----------------------------------*/
extern pid_param_t PID_Init;
extern pid_param_t PID_IMU;
extern pid_param_t PID_GPS;
extern pid_param_t PID_MOTOR;

extern move_filter_struct speed_filter;
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void  PidInit(pid_param_t *pid);
float constrain_float(float amt, float low, float high);
float PidLocCtrl(pid_param_t *pid, float error);
float PidIncCtrl(pid_param_t *pid, float error);

int ARRAY_Element_Calculation(void);

// void GPS_IMU_COM_filtering();//GPS��IMU�����˲�

void move_filter_init(move_filter_struct *move_average);
void move_filter_calc(move_filter_struct *move_average, int32 new_data);
void myprintf(char *format, ...);

void  Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent, Butter_Parameter *LPF);
float LPButterworth(float curr_input, Butter_BufferData *Buffer, Butter_Parameter *Parameter);
float RLS_Update(RLSState *state, float measurement);
/*------------------------------------test------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* ALGORITHM_H */

/**
 * *****************************************************************************
 * @file        algorithm.h
 * @brief       算法实现
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
// 计算一维数组元素个数的宏
#define ARRAY_LENGTH(x) (sizeof(x) / sizeof(x[0]))
// 定义滑动平均滤波缓冲区大小
#define MOVE_AVERAGE_SIZE 32
/*----------------------------------typedef-----------------------------------*/
typedef struct
{
    float kp;     // P
    float ki;     // I
    float kd;     // D
    float imax;   // 积分限幅

    float out_p;   // KP输出
    float out_i;   // KI输出
    float out_d;   // KD输出
    float out;     // pid输出

    float         integrator;        //< 积分值
    float         last_error;        //< 上次误差
    float         last_derivative;   //< 上次误差与上上次误差之差
    unsigned long last_t;            //< 上次时间
} pid_param_t;                       // PID结构体

typedef struct
{
    uint8 index;                            // 下标
    uint8 buffer_size;                      // buffer大小
    int32 data_buffer[MOVE_AVERAGE_SIZE];   // 缓冲区
    int32 data_sum;                         // 数据和
    int32 data_average;                     // 数据平均值
} move_filter_struct;                       // 滑动平均滤波结构体
// 二阶butterworth滤波器
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
// 单变量 RLS 状态结构体定义
typedef struct
{
    float bias;     // 当前估计的偏置
    float P;        // 估计协方差
    float lambda;   // 遗忘因子（建议取1.0f）
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

// void GPS_IMU_COM_filtering();//GPS与IMU互补滤波

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

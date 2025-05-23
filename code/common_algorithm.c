/**
 * *****************************************************************************
 * @file        algorithm.c
 * @brief       算法实现
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

move_filter_struct GPS_Direction_filter;   // 滑动平均滤波结构体

/*----------------------------------variable----------------------------------*/
extern float Daty_Z;   // 陀螺仪积分得到的值(高频噪声)
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/

/*************************************************************************
 *  函数名称：float constrain_float(float amt, float low, float high)
 *  功能说明：限幅函数
 *  参数说明：
 * @param    amt   ： 参数
 * @param    low   ： 最低值
 * @param    high  ： 最高值
 *  函数返回：无
 *  修改时间：2020年4月1日
 *  备    注：
 *************************************************************************/
float constrain_float(float amt, float low, float high)
{
    return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

/*************************************************************************
 *  函数名称：void PidInit(pid_param_t * pid)
 *  功能说明：PID初始化
 *  参数说明：
 * @param    pid   ： PID结构体
 *  函数返回：无
 *  修改时间：2025年1月21日
 *  备    注：
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
 *  函数名称：float constrain_float(float amt, float low, float high)
 *  功能说明：pid位置式控制器输出
 *  参数说明：
 * @param    pid     pid参数
 * @param    error   pid输入误差
 *  函数返回：PID输出结果
 *  修改时间：2020年4月1日
 *  备    注：
 *************************************************************************/
float PidLocCtrl(pid_param_t* pid, float error)
{
    //    PID_GPS.kp=1;
    //    PID_GPS.kd=1.05;


    PID_GPS.kp = 1.1;
    PID_GPS.kd = 5;

    PID_IMU.kp = 1;   // 1.15
    PID_IMU.kd = 7;

    /* 累积误差 */
    pid->integrator += error;

    /* 误差限幅 */
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
 *  函数名称：float constrain_float(float amt, float low, float high)
 *  功能说明：pid增量式控制器输出
 *  参数说明：
 * @param    pid     pid参数
 * @param    error   pid输入误差
 *  函数返回：PID输出结果   注意输出结果已经包涵了上次结果
 *  修改时间：2020年4月1日
 *  备    注：
 *************************************************************************/
float PidIncCtrl(pid_param_t* pid, float error)
{
    PID_MOTOR.kp = 0.3;   // 地上跑要改成2.5
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
// 函数简介     GPS_IMU_COM_filtering
// 参数说明     无
// 使用示例     GPS_IMU_COM_filtering();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
// void GPS_IMU_COM_filtering()
// {
//     float K=0.9;//互补系数
//     float Fusion_angle;//融合后的航向角

//     Fusion_angle=K*Daty_Z+(1-K)*gps_tau1201.direction;//将积分的YAW和逐飞GPS的direction进行互补融合

//     printf("\r\nFusion_angle---%f",Fusion_angle);
// }

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     滑动平均滤波计算
// 参数说明     move_filter     结构体传参
// 参数说明     new_data        新输入的数据
// 使用示例     (&speed_filter, 400);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void move_filter_calc(move_filter_struct* move_filter, int32 new_data)
{
    // 加上新的数值 减去最末尾的数值 求得最新的和
    move_filter->data_sum = move_filter->data_sum + new_data - move_filter->data_buffer[move_filter->index];
    // 重新求平均值
    move_filter->data_average = move_filter->data_sum / move_filter->buffer_size;

    // 将数据写入缓冲区
    move_filter->data_buffer[move_filter->index] = new_data;
    move_filter->index++;
    if (move_filter->buffer_size <= move_filter->index)
    {
        move_filter->index = 0;
    }
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     滑动平均滤波初始化
// 参数说明     move_filter     结构体传参
// 使用示例     move_filter_init(&speed_filter);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void move_filter_init(move_filter_struct* move_filter)
{
    move_filter->data_average = 0;
    move_filter->data_sum     = 0;
    move_filter->index        = 0;
    // 设置缓冲区大小
    move_filter->buffer_size = MOVE_AVERAGE_SIZE;

    uint8 i;
    for (i = 0; i < move_filter->buffer_size; i++)
    {
        move_filter->data_buffer[i] = 0;
    }
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算缓冲区数组元素个数
// 参数说明     无
// 使用示例     ARRAY_Element_Calculation();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
int ARRAY_Element_Calculation()
{
    int i     = 0;   // 定义计数器
    int count = 0;   // 定义数组元素个数计数器

    // 遍历数组，计算数组元素个数
    while (i < 1024 && flash_union_buffer[i].uint32_type != (-1) && flash_union_buffer[i].uint32_type != (0))
    {
        count++;
        i++;
    }
    //    printf("\r\n count=%d",count);
    return count;
}

// 巴特沃夫滤波器初始化
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

// 二阶低通巴特沃夫滤波器
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

    /* 加速度计Butterworth滤波 */
    /* 获取最新x(n) */
    Buffer->Input_Butter[2] = curr_input;
    /* Butterworth滤波 */
    Buffer->Output_Butter[2] = Parameter->b[0] * Buffer->Input_Butter[2] + Parameter->b[1] * Buffer->Input_Butter[1] +
                               Parameter->b[2] * Buffer->Input_Butter[0] - Parameter->a[1] * Buffer->Output_Butter[1] -
                               Parameter->a[2] * Buffer->Output_Butter[0];
    /* x(n) 序列保存 */
    Buffer->Input_Butter[0] = Buffer->Input_Butter[1];
    Buffer->Input_Butter[1] = Buffer->Input_Butter[2];
    /* y(n) 序列保存 */
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
// RLS 更新函数（针对单变量模型，回归向量为1）
float RLS_Update(RLSState* state, float measurement)
{
    float K     = state->P / (state->lambda + state->P);   // 计算增益
    float error = measurement - state->bias;
    state->bias += K * error;                        // 更新偏置估计
    state->P = (1 - K) * state->P / state->lambda;   // 更新协方差
    return state->bias;
}
/*------------------------------------test------------------------------------*/

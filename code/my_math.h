/**
 * *****************************************************************************
 * @file        my_math.h
 * @brief       float和double类型的数学函数
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-25
 * *****************************************************************************
 */
#ifndef MY_MATH_H
#define MY_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------include-----------------------------------*/
// #include "math.h"
#include "stdbool.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"

#include "IfxCcu6_Timer.h"
#include "IfxScuEru.h"
#include "SysSe/Bsp/Bsp.h"
#include "ifxAsclin_reg.h"
/*-----------------------------------macro------------------------------------*/
#define ANGLE_MAX 180.0f
#define ANGLE_MIN -180.0f
#define PI (3.14159265f)
#define MY_PI (3.14159265358979323846)

// 一阶IIR低通滤波器，hz是截止频率
#define LPF_1_(hz, t, in, out) ((out) += (1 / (1 + 1 / ((hz) * 6.28f * (t)))) * ((in) - (out)))
// 限幅
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
// 绝对值
#define ABS(x) ((x) > 0 ? (x) : -(x))
// 防止除0错误的安全除法
#define DIV(Number, Prescaler, Threshold) ((Prescaler == 0.0f) ? Threshold : (Number / Prescaler))
// 求平方
#define Power2(x) (x * x)
// 求2数最大值
#define MAX_2(x, y) ((x >= y) ? x : y)
// 求3数最大值
#define MAX_3(x, y, z) ((z >= MAX_2(x, y)) ? z : MAX_2(x, y))

/*----------------------------------typedef-----------------------------------*/

// 定义结构体封装高低32位
typedef struct
{
    uint32 low;    // 低32位（小端模式）
    uint32 high;   // 高32位
} DoubleParts;
/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
float my_invSqrt(float x);
float my_copysignf(float x, float y);
float my_fabs(float x);
float my_sqrt(float number);
float my_translateAngle(float angle);
float my_sin(float angle);
float my_cos(float angle);
float my_tan(float angle);
float my_arctan1(float tan);
float my_arctan2(float x, float y);
float my_arcsin(float i);
float my_exp(float x);

float my_sin_rad(float rad);
float my_cos_rad(float rad);
float my_tan_rad(float rad);
float my_arctan1_rad(float tan);
float my_arctan2_rad(float x, float y);
float my_arcsin_rad(float i);

double func_pow(double x, int n);
double func_sqrt(double x);
double func_cos(double x);
double func_sin(double x);
double fast_atan2(double y, double x);
double py_asin(double x);
double py_fabs(double x);
long   factorial(int x);
double py_sqrt(double x);

// 函数声明：拆解double为结构体
DoubleParts Double_To_Parts(double value);
double      Parts_To_Double(DoubleParts parts);

bool my_isnan(float f);
/*------------------------------------test------------------------------------*/
void double_test(void);
#ifdef __cplusplus
}
#endif

#endif /* MATH_H */

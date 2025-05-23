/**
 * *****************************************************************************
 * @file        my_math.h
 * @brief       float��double���͵���ѧ����
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

// һ��IIR��ͨ�˲�����hz�ǽ�ֹƵ��
#define LPF_1_(hz, t, in, out) ((out) += (1 / (1 + 1 / ((hz) * 6.28f * (t)))) * ((in) - (out)))
// �޷�
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
// ����ֵ
#define ABS(x) ((x) > 0 ? (x) : -(x))
// ��ֹ��0����İ�ȫ����
#define DIV(Number, Prescaler, Threshold) ((Prescaler == 0.0f) ? Threshold : (Number / Prescaler))
// ��ƽ��
#define Power2(x) (x * x)
// ��2�����ֵ
#define MAX_2(x, y) ((x >= y) ? x : y)
// ��3�����ֵ
#define MAX_3(x, y, z) ((z >= MAX_2(x, y)) ? z : MAX_2(x, y))

/*----------------------------------typedef-----------------------------------*/

// ����ṹ���װ�ߵ�32λ
typedef struct
{
    uint32 low;    // ��32λ��С��ģʽ��
    uint32 high;   // ��32λ
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

// �������������doubleΪ�ṹ��
DoubleParts Double_To_Parts(double value);
double      Parts_To_Double(DoubleParts parts);

bool my_isnan(float f);
/*------------------------------------test------------------------------------*/
void double_test(void);
#ifdef __cplusplus
}
#endif

#endif /* MATH_H */

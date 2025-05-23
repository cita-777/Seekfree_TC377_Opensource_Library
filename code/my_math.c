/**
 * *****************************************************************************
 * @file        my_math.c
 * @brief       float和double类型的数学函数
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-26
 * *****************************************************************************
 */
#include "zf_common_headfile.h"
// 优化后的快速平方根倒数
float my_invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y     = x;

    // 使用位操作获取初始估计
    uint32_t i = *(uint32_t*)&y;
    i          = 0x5F3759DF - (i >> 1);
    y          = *(float*)&i;

    // 牛顿迭代
    y = y * (1.5f - (halfx * y * y));
    return y;
}

float my_fabs(float x)
{
    return (x < 0) ? -x : x;
}

float my_copysignf(float x, float y)
{
    /* 保留x的绝对值，符号与y相同 */
    return (y >= 0) ? my_fabs(x) : -my_fabs(x);
}

/*******************************************************************************
 * 函 数 名         : my_sqrt
 * 函数功能           : 快速算根值
 * 输    入         : number 根号里边的数
 * 输    出         : 最大的绝对值
 *******************************************************************************/
float my_sqrt(float number)
{
    long        i;
    float       x, y;
    const float f = 1.5F;
    x             = number * 0.5F;
    y             = number;
    i             = *(long*)&y;
    i             = 0x5f3759df - (i >> 1);

    y = *(float*)&i;
    y = y * (f - (x * y * y));
    y = y * (f - (x * y * y));
    return number * y;
}
/*******************************************************************************
 * 函 数 名         : my_translateAngle
 * 函数功能           : 将角度转变为-180~180之间
 * 输    入         :
 * 输    出         :
 *******************************************************************************/
float my_translateAngle(float angle)
{
    while (angle > ANGLE_MAX)   // 判断目标角度是否在允许角度范围
        angle -= 360.0f;
    while (angle < ANGLE_MIN) angle += 360.0f;
    return angle;
}
/*******************************************************************************
 * 函 数 名         : my_sin
 * 函数功能           :
 * 输    入         :
 * 输    出         :
 *******************************************************************************/
float my_sin(float angle)
{
    float sine, rad;
    angle = my_translateAngle(angle);
    rad   = angle * PI / 180.0f;

    sine = (rad < 0) ? rad * (1.27323954f + 0.405284735f * rad) : rad * (1.27323954f - 0.405284735f * rad);
    sine = (sine < 0) ? sine * (-0.225f * (sine + 1) + 1) : sine * (0.225f * (sine - 1) + 1);
    return sine;
}
/*******************************************************************************
 * 函 数 名         : my_cos
 * 函数功能           :
 * 输    入         :
 * 输    出         :
 *******************************************************************************/
float my_cos(float angle)
{
    return my_sin(angle + 90.0f);
}
/*******************************************************************************
 * 函 数 名         : my_tan
 * 函数功能           :
 * 输    入         :
 * 输    出         :
 *******************************************************************************/
float my_tan(float angle)
{
    return my_sin(angle) / my_cos(angle);
}
/*******************************************************************************
 * 函 数 名         : my_arctan1
 * 函数功能           : 反正切 （+- 0.09°）
 * 输    入         :
 * 输    出         :
 *******************************************************************************/
float my_arctan1(float tan)
{
    float angle = (ABS(tan) > 1.0f)
                      ? 90.0f - ABS(1.0f / tan) * (45.0f - (ABS(1.0f / tan) - 1.0f) * (14.0f + 3.83f * ABS(1.0f / tan)))
                      : ABS(tan) * (45.0f - (ABS(tan) - 1.0f) * (14.0f + 3.83f * ABS(tan)));
    return (tan > 0) ? angle : -angle;
}
/*******************************************************************************
 * 函 数 名         : my_arctan2
 * 函数功能           : 反正切 （+- 0.09°）
 * 输    入         :
 * 输    出         :
 *******************************************************************************/
float my_arctan2(float x, float y)
{
    float tan, angle;

    if (x == 0 && y == 0)   // 不存在
        return 0;

    if (x == 0)   // y轴上
    {
        if (y > 0)
            return 90;
        else
            return -90;
    }

    if (y == 0)   // x轴上
    {
        if (x > 0)
            return 0;
        else
            return -180.0f;
    }

    tan   = y / x;
    angle = my_arctan1(tan);
    if (x < 0 && angle > 0)
        angle -= 180.0f;
    else if (x < 0 && angle < 0)
        angle += 180.0f;
    return angle;
}
/*******************************************************************************
 * 函 数 名         : my_arcsin
 * 函数功能           : 反正切 （+- 0.09°）
 * 输    入         :
 * 输    出         :
 *******************************************************************************/
float my_arcsin(float i)
{
    return my_arctan1(i / my_sqrt(1 - i * i));
}
/*******************************************************************************
 * 函 数 名         : my_exp
 * 函数功能           : e指数
 * 输    入         :
 * 输    出         :
 *******************************************************************************/
float my_exp(float x)   // 计算e^x,实现系统的exp()功能?
{
    if (x == 0) return 1;
    if (x < 0) return 1 / my_exp(-x);

    double y = x, ex_p1 = 0, ex_p2 = 0, ex_p3 = 0, ex_px = 0, ex_tmp = 1, dex_px = 1, tmp;
    int    l;
    for (l = 1, tmp = 1; ((ex_px - ex_tmp) > 1e-10 || (ex_px - ex_tmp) < -1e-10) && dex_px > 1e-10; l++)
    {
        ex_tmp = ex_px;
        tmp *= y;
        tmp = tmp / l;
        ex_p1 += tmp;
        ex_p2  = ex_p1 + tmp * y / (l + 1);
        ex_p3  = ex_p2 + tmp * y * y / (l + 1) / (l + 2);
        dex_px = ex_p3 - ex_p2;
        ex_px  = ex_p3 - dex_px * dex_px / (ex_p3 - 2 * ex_p2 + ex_p1);
    }
    return ex_px + 1;
}

/**
 * @brief  将角度归一化到 -π 到 π 范围内（弧度）
 * @param  angle: 输入角度（弧度）
 * @return 归一化后的角度（弧度）
 */
float my_translateAngleRad(float angle)
{
    // 归一化到 [-π, π]
    while (angle > PI + 1e-6) angle -= 2.0f * PI;
    while (angle < -PI - 1e-6) angle += 2.0f * PI;
    return angle;
}

/**
 * @brief  快速近似计算正弦（输入为弧度）
 * @param  rad: 输入角度（弧度）
 * @return 近似的 sin(rad) 值
 *
 * 该算法基于快速正弦近似，原理类似于:
 * sin(x) ≈ (4/π)*x - (4/π?)*x*|x|
 * 后续通过一个二次修正项进一步提高精度。
 */
float my_sin_rad(float rad)
{
    float sine;
    // 归一化角度到 [-π, π]
    rad = my_translateAngleRad(rad);

    // 快速近似正弦（期望输入为弧度）
    // 1.27323954 ≈ 4/π, 0.405284735 ≈ 4/π?
    sine = (rad < 0) ? rad * (1.27323954f + 0.405284735f * rad) : rad * (1.27323954f - 0.405284735f * rad);
    // 二次校正项：0.225 为调整系数
    sine = (sine < 0) ? sine * (-0.225f * (sine + 1) + 1) : sine * (0.225f * (sine - 1) + 1);
    return sine;
}

/**
 * @brief  快速近似计算余弦（输入为弧度）
 * @param  rad: 输入角度（弧度）
 * @return 近似的 cos(rad) 值
 *
 * 这里利用 sin(x+π/2) = cos(x) 的公式实现。
 */
float my_cos_rad(float rad)
{
    return my_sin_rad(rad + PI / 2.0f);
}

/**
 * @brief  快速近似计算正切（输入为弧度）
 * @param  rad: 输入角度（弧度）
 * @return 近似的 tan(rad) 值
 */
float my_tan_rad(float rad)
{
    float cosVal = my_cos_rad(rad);
    if (fabsf(cosVal) < 1e-6f)
    {
        // 避免除零错误，根据实际需求可以返回一个极大值或其它错误标识
        return (my_sin_rad(rad) >= 0) ? 1e6f : -1e6f;
    }
    return my_sin_rad(rad) / cosVal;
}
/**
 * @brief 快速近似计算反正切（基于 tan 值，返回值为弧度）
 *        此函数原理来自基于度数的近似方法，最后转换为弧度。
 * @param tan_val 输入的正切值
 * @return 近似的 arctan(tan_val)（弧度）
 */
float my_arctan1_rad(float tan_val)
{
    float angle_deg;
    float abs_tan = ABS(tan_val);
    if (abs_tan > 1.0f)
    {
        // 当 |tan| > 1 时，利用对称性计算
        float inv = ABS(1.0f / tan_val);
        angle_deg = 90.0f - inv * (45.0f - (inv - 1.0f) * (14.0f + 3.83f * inv));
    }
    else
    {
        // 当 |tan| <= 1 时
        angle_deg = abs_tan * (45.0f - (abs_tan - 1.0f) * (14.0f + 3.83f * abs_tan));
    }
    angle_deg = (tan_val >= 0) ? angle_deg : -angle_deg;
    // 将角度转换为弧度
    return angle_deg * (PI / 180.0f);
}

/**
 * @brief 快速近似计算二维反正切（基于 x 和 y 坐标，返回值为弧度）
 *        注意：此函数假设参数顺序为 (x, y)，与标准库 arctan2(y, x) 的参数顺序不同，
 *        但遵循原有代码逻辑。
 * @param x 横坐标分量
 * @param y 纵坐标分量
 * @return 近似的 arctan2(x, y)（弧度）
 */
float my_arctan2_rad(float x, float y)
{
    // 特殊情况处理
    if (x == 0.0f && y == 0.0f) return 0.0f;   // 未定义，可返回0
    if (x == 0.0f)
    {
        return (y > 0.0f) ? PI / 2.0f : -PI / 2.0f;
    }
    if (y == 0.0f)
    {
        // 注意：原代码中 x<0 时返回 -180°，这里转换为弧度即 -PI
        return (x > 0.0f) ? 0.0f : -PI;
    }
    // 计算近似反正切（注意原代码 tan = y/x）
    float tan_val   = y / x;
    float angle_rad = my_arctan1_rad(tan_val);
    // 对 x<0 的情况进行补正（原代码在度数下做 ±180° 调整）
    if (x < 0.0f)
    {
        if (angle_rad > 0.0f)
            angle_rad -= PI;
        else
            angle_rad += PI;
    }
    return angle_rad;
}

/**
 * @brief 快速近似计算反正弦（返回值为弧度）
 *        利用公式 arcsin(i) = arctan( i / sqrt(1 - i^2) )
 * @param i 输入值，要求 |i| <= 1
 * @return 近似的 arcsin(i)（弧度）
 */
float my_arcsin_rad(float i)
{
    // 注意需要确保 1 - i^2 >= 0
    float denominator = my_sqrt(1.0f - i * i);
    // 当 denominator 接近 0 时，可做特殊处理
    if (denominator < 1e-6f) return (i >= 0) ? PI / 2.0f : -PI / 2.0f;
    return my_arctan1_rad(i / denominator);
}
//////////////////////////////////////////////////////////////////////////////////////////////////
/*----------------------------------以下为GPS——Double专用数学库----------------------------------*/
//////////////////////////////////////////////////////////////////////////////////////////////////
#if GL_GPS_MATH_FLAG

#    define GL_PI 3.14159265358979323846

#    define TAN_MAP_RES (0.003921569f) /* (smallest non-zero value in table) */
#    define TAN_MAP_SIZE (256)
// 快速反正切表
const double fast_atan_table[257] = {
    0.000000e+00, 3.921549e-03, 7.842976e-03, 1.176416e-02, 1.568499e-02, 1.960533e-02, 2.352507e-02, 2.744409e-02,
    3.136226e-02, 3.527947e-02, 3.919560e-02, 4.311053e-02, 4.702413e-02, 5.093629e-02, 5.484690e-02, 5.875582e-02,
    6.266295e-02, 6.656816e-02, 7.047134e-02, 7.437238e-02, 7.827114e-02, 8.216752e-02, 8.606141e-02, 8.995267e-02,
    9.384121e-02, 9.772691e-02, 1.016096e-01, 1.054893e-01, 1.093658e-01, 1.132390e-01, 1.171087e-01, 1.209750e-01,
    1.248376e-01, 1.286965e-01, 1.325515e-01, 1.364026e-01, 1.402496e-01, 1.440924e-01, 1.479310e-01, 1.517652e-01,
    1.555948e-01, 1.594199e-01, 1.632403e-01, 1.670559e-01, 1.708665e-01, 1.746722e-01, 1.784728e-01, 1.822681e-01,
    1.860582e-01, 1.898428e-01, 1.936220e-01, 1.973956e-01, 2.011634e-01, 2.049255e-01, 2.086818e-01, 2.124320e-01,
    2.161762e-01, 2.199143e-01, 2.236461e-01, 2.273716e-01, 2.310907e-01, 2.348033e-01, 2.385093e-01, 2.422086e-01,
    2.459012e-01, 2.495869e-01, 2.532658e-01, 2.569376e-01, 2.606024e-01, 2.642600e-01, 2.679104e-01, 2.715535e-01,
    2.751892e-01, 2.788175e-01, 2.824383e-01, 2.860514e-01, 2.896569e-01, 2.932547e-01, 2.968447e-01, 3.004268e-01,
    3.040009e-01, 3.075671e-01, 3.111252e-01, 3.146752e-01, 3.182170e-01, 3.217506e-01, 3.252758e-01, 3.287927e-01,
    3.323012e-01, 3.358012e-01, 3.392926e-01, 3.427755e-01, 3.462497e-01, 3.497153e-01, 3.531721e-01, 3.566201e-01,
    3.600593e-01, 3.634896e-01, 3.669110e-01, 3.703234e-01, 3.737268e-01, 3.771211e-01, 3.805064e-01, 3.838825e-01,
    3.872494e-01, 3.906070e-01, 3.939555e-01, 3.972946e-01, 4.006244e-01, 4.039448e-01, 4.072558e-01, 4.105574e-01,
    4.138496e-01, 4.171322e-01, 4.204054e-01, 4.236689e-01, 4.269229e-01, 4.301673e-01, 4.334021e-01, 4.366272e-01,
    4.398426e-01, 4.430483e-01, 4.462443e-01, 4.494306e-01, 4.526070e-01, 4.557738e-01, 4.589307e-01, 4.620778e-01,
    4.652150e-01, 4.683424e-01, 4.714600e-01, 4.745676e-01, 4.776654e-01, 4.807532e-01, 4.838312e-01, 4.868992e-01,
    4.899573e-01, 4.930055e-01, 4.960437e-01, 4.990719e-01, 5.020902e-01, 5.050985e-01, 5.080968e-01, 5.110852e-01,
    5.140636e-01, 5.170320e-01, 5.199904e-01, 5.229388e-01, 5.258772e-01, 5.288056e-01, 5.317241e-01, 5.346325e-01,
    5.375310e-01, 5.404195e-01, 5.432980e-01, 5.461666e-01, 5.490251e-01, 5.518738e-01, 5.547124e-01, 5.575411e-01,
    5.603599e-01, 5.631687e-01, 5.659676e-01, 5.687566e-01, 5.715357e-01, 5.743048e-01, 5.770641e-01, 5.798135e-01,
    5.825531e-01, 5.852828e-01, 5.880026e-01, 5.907126e-01, 5.934128e-01, 5.961032e-01, 5.987839e-01, 6.014547e-01,
    6.041158e-01, 6.067672e-01, 6.094088e-01, 6.120407e-01, 6.146630e-01, 6.172755e-01, 6.198784e-01, 6.224717e-01,
    6.250554e-01, 6.276294e-01, 6.301939e-01, 6.327488e-01, 6.352942e-01, 6.378301e-01, 6.403565e-01, 6.428734e-01,
    6.453808e-01, 6.478788e-01, 6.503674e-01, 6.528466e-01, 6.553165e-01, 6.577770e-01, 6.602282e-01, 6.626701e-01,
    6.651027e-01, 6.675261e-01, 6.699402e-01, 6.723452e-01, 6.747409e-01, 6.771276e-01, 6.795051e-01, 6.818735e-01,
    6.842328e-01, 6.865831e-01, 6.889244e-01, 6.912567e-01, 6.935800e-01, 6.958943e-01, 6.981998e-01, 7.004964e-01,
    7.027841e-01, 7.050630e-01, 7.073330e-01, 7.095943e-01, 7.118469e-01, 7.140907e-01, 7.163258e-01, 7.185523e-01,
    7.207701e-01, 7.229794e-01, 7.251800e-01, 7.273721e-01, 7.295557e-01, 7.317307e-01, 7.338974e-01, 7.360555e-01,
    7.382053e-01, 7.403467e-01, 7.424797e-01, 7.446045e-01, 7.467209e-01, 7.488291e-01, 7.509291e-01, 7.530208e-01,
    7.551044e-01, 7.571798e-01, 7.592472e-01, 7.613064e-01, 7.633576e-01, 7.654008e-01, 7.674360e-01, 7.694633e-01,
    7.714826e-01, 7.734940e-01, 7.754975e-01, 7.774932e-01, 7.794811e-01, 7.814612e-01, 7.834335e-01, 7.853983e-01,
    7.853983e-01};
/**
 * @brief      ：角度转弧度
 * @param[in]  ：角度
 * @return     ：弧度
 */
double deg2rad(double deg)
{
    return (deg * PI) / 180.0;
}
/**
 * @brief      ：弧度转角度
 * @param[in]  ：弧度
 * @return     ：角度
 */
double rad2deg(double rad)
{
    return rad * 180 / PI;
}
/**
 * @brief      ：计算平方
 * @param[in]  ：x
 * @return     ：平方
 */
double func_pow(double x, int n)

{

    double result = 1;
    if (n == 0) return 1;
    for (int i = 0; i < n; i++)
    {
        result *= x;
    }

    return result;
}

/**
 * @brief      ：计算开根方
 * @param[in]  ：x
 * @return     ：开根方
 */
double func_sqrt(double x)

{

    double j, k;

    j = 0.0;

    k = x / 2;

    while (j != k)

    {

        j = k;

        k = (j + x / j) / 2;
    }

    return j;
}

/**
 * @brief      ：计算sin
 * @param[in]  ：x
 * @return     ：sin
 */
double func_sin(double x)

{

    double result = x, temp = x;
    double den = x, fac = 1;
    int    n = 1, sign = 1;
    while ((temp > 1e-15) || (temp < -1e-15))
    {
        n++, fac *= n, den *= x;
        n++, fac *= n, den *= x;
        temp   = den / fac;
        sign   = -sign;
        result = sign > 0 ? result + temp : result - temp;
    }
    return result;
}

/**
 * @brief      ：计算cos
 * @param[in]  ：x
 * @return     ：cos
 */
double func_cos(double x)

{

    x = PI / 2 - x;
    return func_sin(x);
}

/**
 * @brief       计算double的绝对值
 *
 * @param       x
 * @return      double
 */
double py_fabs(double x)
{
    if (x < 0) return -x;
    return x;
}

/**
 * @brief      ：牛顿迭代法计算开根方
 *
 * @param       a  ：要开根方的数
 * @param       x0 ：迭代初值
 * @return      double
 * @note        递归调用使用牛顿迭代法计算平方根的核心函数
 */
double k_sqrt(double a, double x0)
{

    double x1, y;
    x1 = (x0 + a / x0) / 2.0;
    if (py_fabs(x1 - x0) >= 10e-15)
        y = k_sqrt(a, x1);
    else
        y = x1;
    return y;
}

/**
 * @brief      ：封装牛顿迭代法计算开根方，初值为1.0
 *
 * @param       x
 * @return      double
 */
double py_sqrt(double x)
{
    return k_sqrt(x, 1.0);
}

/**
 * @brief      ：计算阶乘
 *
 * @param       x  ：要计算阶乘的正整数
 * @return      long
 * @note        迭代计算n的阶乘
 */
long factorial(int x)
{
    long result = 1;
    for (int i = 1; i <= x; i++) result *= i;

    return result;
}

/**
 * @brief      ：计算反正弦函数
 *
 * @param       x  ：输入值，范围[-1,1]
 * @return      double
 * @note        使用泰勒级数展开计算arcsin值
 */
double py_asin(double x)
{

    double result = 0;
    double temp   = x;

    int n = 1;

    while ((temp > 1e-15) || (temp < -1e-15))
    {
        result += temp;
        temp =
            factorial(2 * n) / (func_pow(2, 2 * n) * func_pow(factorial(n), 2)) * func_pow(x, 2 * n + 1) / (2 * n + 1);
        // printf("%lf %d\n", temp, n);
        n++;
    }
    // printf("n=%d\n",n);
    return result;
}

/**
 * @brief      ：快速计算反正切值
 *
 * @param       y  ：y坐标值
 * @param       x  ：x坐标值
 * @return      double
 * @note        使用查表法和线性插值实现快速atan2计算
 */
double fast_atan2(double y, double x)
{
    double x_abs, y_abs, z;
    double alpha, angle, base_angle;
    int    index;

    /* don't divide by zero! */
    if ((y == 0.0f) && (x == 0.0f))
        angle = 0.0f;
    else
    {
        /* normalize to +/- 45 degree range */
        y_abs = ABS(y);
        x_abs = ABS(x);
        // z = (y_abs < x_abs ? y_abs / x_abs : x_abs / y_abs);
        if (y_abs < x_abs)
            z = y_abs / x_abs;
        else
            z = x_abs / y_abs;
        /* when ratio approaches the table resolution, the angle is */
        /*      best approximated with the argument itself...       */
        if (z < TAN_MAP_RES)
            base_angle = z;
        else
        {
            /* find index and interpolation value */
            alpha = z * (double)TAN_MAP_SIZE - .5f;
            index = (int)alpha;
            alpha -= (double)index;
            /* determine base angle based on quadrant and */
            /* add or subtract table value from base angle based on quadrant */
            base_angle = fast_atan_table[index];
            base_angle += (fast_atan_table[index + 1] - fast_atan_table[index]) * alpha;
        }

        if (x_abs > y_abs)
        { /* -45 -> 45 or 135 -> 225 */
            if (x >= 0.0f)
            { /* -45 -> 45 */
                if (y >= 0.0f)
                    angle = base_angle; /* 0 -> 45, angle OK */
                else
                    angle = -base_angle; /* -45 -> 0, angle = -angle */
            }
            else
            { /* 135 -> 180 or 180 -> -135 */
                angle = 3.14159265358979323846;

                if (y >= 0.0f)
                    angle -= base_angle; /* 135 -> 180, angle = 180 - angle */
                else
                    angle = base_angle - angle; /* 180 -> -135, angle = angle - 180 */
            }
        }
        else
        { /* 45 -> 135 or -135 -> -45 */
            if (y >= 0.0f)
            { /* 45 -> 135 */
                angle = 1.57079632679489661923;

                if (x >= 0.0f)
                    angle -= base_angle; /* 45 -> 90, angle = 90 - angle */
                else
                    angle += base_angle; /* 90 -> 135, angle = 90 + angle */
            }
            else
            { /* -135 -> -45 */
                angle = -1.57079632679489661923;

                if (x >= 0.0f)
                    angle += base_angle; /* -90 -> -45, angle = -90 + angle */
                else
                    angle -= base_angle; /* -135 -> -90, angle = -90 - angle */
            }
        }
    }

/**
 * @brief      ：角度范围转换
 *
 * @param       angle ：本函数即fast_atan2返回的角度值
 * @return      转换后的角度值
 * @note        ZERO_TO_TWOPI宏定义控制输出范围:
 *              定义时输出范围为[0,2π]
 *              未定义时输出范围为[-π,π]
 */
#    ifdef ZERO_TO_TWOPI
    if (angle < 0)
        return (angle + TWOPI);
    else
        return (angle);
#    else
    return (angle);

#    endif
}

#endif   // 条件编译Double数学库

//////////////////////////////////////////////////////////////////////////////////////////////////
/*----------------------------------以上为GPS——Double专用数学库----------------------------------*/
//////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////
/*----------------------------------以下为数据类型转换专用数学库----------------------------------*/
//////////////////////////////////////////////////////////////////////////////////////////////////
// 使用联合体确保内存操作安全
typedef union
{
    double d64;
    uint32 u32[2];   // u32[0]为低32位（小端模式）
} DoubleConverter;

// 拆解double为结构体（直接返回值）
DoubleParts Double_To_Parts(double value)
{
    DoubleConverter converter;
    converter.d64 = value;
    DoubleParts parts;
    parts.low  = converter.u32[0];
    parts.high = converter.u32[1];
    return parts;
}

// 从结构体恢复double
double Parts_To_Double(DoubleParts parts)
{
    DoubleConverter converter;
    converter.u32[0] = parts.low;
    converter.u32[1] = parts.high;
    return converter.d64;
}
void double_test(void)
{
    double original = 3.141592653589793;

    // 拆解double到结构体
    DoubleParts parts = Double_To_Parts(original);

    // 恢复double
    double recovered = Parts_To_Double(parts);

    // 验证精度
    if (original == recovered)
    {
        printf("Success: No precision loss!\n");
    }
    else
    {
        printf("Error: Precision loss detected!\n");
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
/*----------------------------------以上为数据类型转换专用数学库----------------------------------*/
//////////////////////////////////////////////////////////////////////////////////////////////////
// 检查一个float是否为NaN
bool my_isnan(float f)
{
    // 将float转换为uint32_t以便按位检查
    uint32_t i = *(uint32_t*)&f;

    // IEEE 754单精度NaN的位模式特征
    // 指数位全为1（0x7F800000），尾数位不全为0
    const uint32_t nan_mask       = 0x7F800000;
    const uint32_t quiet_nan_mask = nan_mask | 0x00400000;   // 假设我们只关心静默NaN

    // 检查是否为NaN
    if ((i & nan_mask) == nan_mask && (i & ~quiet_nan_mask) != 0)
    {
        // 这里我们只检查了是否为NaN，没有区分静默NaN和信号NaN
        // 如果你只关心静默NaN，可以去掉 & ~quiet_nan_mask 的检查
        return true;
    }

    // 如果不是NaN，返回false
    return false;
}

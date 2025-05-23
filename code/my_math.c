/**
 * *****************************************************************************
 * @file        my_math.c
 * @brief       float��double���͵���ѧ����
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-26
 * *****************************************************************************
 */
#include "zf_common_headfile.h"
// �Ż���Ŀ���ƽ��������
float my_invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y     = x;

    // ʹ��λ������ȡ��ʼ����
    uint32_t i = *(uint32_t*)&y;
    i          = 0x5F3759DF - (i >> 1);
    y          = *(float*)&i;

    // ţ�ٵ���
    y = y * (1.5f - (halfx * y * y));
    return y;
}

float my_fabs(float x)
{
    return (x < 0) ? -x : x;
}

float my_copysignf(float x, float y)
{
    /* ����x�ľ���ֵ��������y��ͬ */
    return (y >= 0) ? my_fabs(x) : -my_fabs(x);
}

/*******************************************************************************
 * �� �� ��         : my_sqrt
 * ��������           : �������ֵ
 * ��    ��         : number ������ߵ���
 * ��    ��         : ���ľ���ֵ
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
 * �� �� ��         : my_translateAngle
 * ��������           : ���Ƕ�ת��Ϊ-180~180֮��
 * ��    ��         :
 * ��    ��         :
 *******************************************************************************/
float my_translateAngle(float angle)
{
    while (angle > ANGLE_MAX)   // �ж�Ŀ��Ƕ��Ƿ�������Ƕȷ�Χ
        angle -= 360.0f;
    while (angle < ANGLE_MIN) angle += 360.0f;
    return angle;
}
/*******************************************************************************
 * �� �� ��         : my_sin
 * ��������           :
 * ��    ��         :
 * ��    ��         :
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
 * �� �� ��         : my_cos
 * ��������           :
 * ��    ��         :
 * ��    ��         :
 *******************************************************************************/
float my_cos(float angle)
{
    return my_sin(angle + 90.0f);
}
/*******************************************************************************
 * �� �� ��         : my_tan
 * ��������           :
 * ��    ��         :
 * ��    ��         :
 *******************************************************************************/
float my_tan(float angle)
{
    return my_sin(angle) / my_cos(angle);
}
/*******************************************************************************
 * �� �� ��         : my_arctan1
 * ��������           : ������ ��+- 0.09�㣩
 * ��    ��         :
 * ��    ��         :
 *******************************************************************************/
float my_arctan1(float tan)
{
    float angle = (ABS(tan) > 1.0f)
                      ? 90.0f - ABS(1.0f / tan) * (45.0f - (ABS(1.0f / tan) - 1.0f) * (14.0f + 3.83f * ABS(1.0f / tan)))
                      : ABS(tan) * (45.0f - (ABS(tan) - 1.0f) * (14.0f + 3.83f * ABS(tan)));
    return (tan > 0) ? angle : -angle;
}
/*******************************************************************************
 * �� �� ��         : my_arctan2
 * ��������           : ������ ��+- 0.09�㣩
 * ��    ��         :
 * ��    ��         :
 *******************************************************************************/
float my_arctan2(float x, float y)
{
    float tan, angle;

    if (x == 0 && y == 0)   // ������
        return 0;

    if (x == 0)   // y����
    {
        if (y > 0)
            return 90;
        else
            return -90;
    }

    if (y == 0)   // x����
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
 * �� �� ��         : my_arcsin
 * ��������           : ������ ��+- 0.09�㣩
 * ��    ��         :
 * ��    ��         :
 *******************************************************************************/
float my_arcsin(float i)
{
    return my_arctan1(i / my_sqrt(1 - i * i));
}
/*******************************************************************************
 * �� �� ��         : my_exp
 * ��������           : eָ��
 * ��    ��         :
 * ��    ��         :
 *******************************************************************************/
float my_exp(float x)   // ����e^x,ʵ��ϵͳ��exp()����?
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
 * @brief  ���Ƕȹ�һ���� -�� �� �� ��Χ�ڣ����ȣ�
 * @param  angle: ����Ƕȣ����ȣ�
 * @return ��һ����ĽǶȣ����ȣ�
 */
float my_translateAngleRad(float angle)
{
    // ��һ���� [-��, ��]
    while (angle > PI + 1e-6) angle -= 2.0f * PI;
    while (angle < -PI - 1e-6) angle += 2.0f * PI;
    return angle;
}

/**
 * @brief  ���ٽ��Ƽ������ң�����Ϊ���ȣ�
 * @param  rad: ����Ƕȣ����ȣ�
 * @return ���Ƶ� sin(rad) ֵ
 *
 * ���㷨���ڿ������ҽ��ƣ�ԭ��������:
 * sin(x) �� (4/��)*x - (4/��?)*x*|x|
 * ����ͨ��һ�������������һ����߾��ȡ�
 */
float my_sin_rad(float rad)
{
    float sine;
    // ��һ���Ƕȵ� [-��, ��]
    rad = my_translateAngleRad(rad);

    // ���ٽ������ң���������Ϊ���ȣ�
    // 1.27323954 �� 4/��, 0.405284735 �� 4/��?
    sine = (rad < 0) ? rad * (1.27323954f + 0.405284735f * rad) : rad * (1.27323954f - 0.405284735f * rad);
    // ����У���0.225 Ϊ����ϵ��
    sine = (sine < 0) ? sine * (-0.225f * (sine + 1) + 1) : sine * (0.225f * (sine - 1) + 1);
    return sine;
}

/**
 * @brief  ���ٽ��Ƽ������ң�����Ϊ���ȣ�
 * @param  rad: ����Ƕȣ����ȣ�
 * @return ���Ƶ� cos(rad) ֵ
 *
 * �������� sin(x+��/2) = cos(x) �Ĺ�ʽʵ�֡�
 */
float my_cos_rad(float rad)
{
    return my_sin_rad(rad + PI / 2.0f);
}

/**
 * @brief  ���ٽ��Ƽ������У�����Ϊ���ȣ�
 * @param  rad: ����Ƕȣ����ȣ�
 * @return ���Ƶ� tan(rad) ֵ
 */
float my_tan_rad(float rad)
{
    float cosVal = my_cos_rad(rad);
    if (fabsf(cosVal) < 1e-6f)
    {
        // ���������󣬸���ʵ��������Է���һ������ֵ�����������ʶ
        return (my_sin_rad(rad) >= 0) ? 1e6f : -1e6f;
    }
    return my_sin_rad(rad) / cosVal;
}
/**
 * @brief ���ٽ��Ƽ��㷴���У����� tan ֵ������ֵΪ���ȣ�
 *        �˺���ԭ�����Ի��ڶ����Ľ��Ʒ��������ת��Ϊ���ȡ�
 * @param tan_val ���������ֵ
 * @return ���Ƶ� arctan(tan_val)�����ȣ�
 */
float my_arctan1_rad(float tan_val)
{
    float angle_deg;
    float abs_tan = ABS(tan_val);
    if (abs_tan > 1.0f)
    {
        // �� |tan| > 1 ʱ�����öԳ��Լ���
        float inv = ABS(1.0f / tan_val);
        angle_deg = 90.0f - inv * (45.0f - (inv - 1.0f) * (14.0f + 3.83f * inv));
    }
    else
    {
        // �� |tan| <= 1 ʱ
        angle_deg = abs_tan * (45.0f - (abs_tan - 1.0f) * (14.0f + 3.83f * abs_tan));
    }
    angle_deg = (tan_val >= 0) ? angle_deg : -angle_deg;
    // ���Ƕ�ת��Ϊ����
    return angle_deg * (PI / 180.0f);
}

/**
 * @brief ���ٽ��Ƽ����ά�����У����� x �� y ���꣬����ֵΪ���ȣ�
 *        ע�⣺�˺����������˳��Ϊ (x, y)�����׼�� arctan2(y, x) �Ĳ���˳��ͬ��
 *        ����ѭԭ�д����߼���
 * @param x ���������
 * @param y ���������
 * @return ���Ƶ� arctan2(x, y)�����ȣ�
 */
float my_arctan2_rad(float x, float y)
{
    // �����������
    if (x == 0.0f && y == 0.0f) return 0.0f;   // δ���壬�ɷ���0
    if (x == 0.0f)
    {
        return (y > 0.0f) ? PI / 2.0f : -PI / 2.0f;
    }
    if (y == 0.0f)
    {
        // ע�⣺ԭ������ x<0 ʱ���� -180�㣬����ת��Ϊ���ȼ� -PI
        return (x > 0.0f) ? 0.0f : -PI;
    }
    // ������Ʒ����У�ע��ԭ���� tan = y/x��
    float tan_val   = y / x;
    float angle_rad = my_arctan1_rad(tan_val);
    // �� x<0 ��������в�����ԭ�����ڶ������� ��180�� ������
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
 * @brief ���ٽ��Ƽ��㷴���ң�����ֵΪ���ȣ�
 *        ���ù�ʽ arcsin(i) = arctan( i / sqrt(1 - i^2) )
 * @param i ����ֵ��Ҫ�� |i| <= 1
 * @return ���Ƶ� arcsin(i)�����ȣ�
 */
float my_arcsin_rad(float i)
{
    // ע����Ҫȷ�� 1 - i^2 >= 0
    float denominator = my_sqrt(1.0f - i * i);
    // �� denominator �ӽ� 0 ʱ���������⴦��
    if (denominator < 1e-6f) return (i >= 0) ? PI / 2.0f : -PI / 2.0f;
    return my_arctan1_rad(i / denominator);
}
//////////////////////////////////////////////////////////////////////////////////////////////////
/*----------------------------------����ΪGPS����Doubleר����ѧ��----------------------------------*/
//////////////////////////////////////////////////////////////////////////////////////////////////
#if GL_GPS_MATH_FLAG

#    define GL_PI 3.14159265358979323846

#    define TAN_MAP_RES (0.003921569f) /* (smallest non-zero value in table) */
#    define TAN_MAP_SIZE (256)
// ���ٷ����б�
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
 * @brief      ���Ƕ�ת����
 * @param[in]  ���Ƕ�
 * @return     ������
 */
double deg2rad(double deg)
{
    return (deg * PI) / 180.0;
}
/**
 * @brief      ������ת�Ƕ�
 * @param[in]  ������
 * @return     ���Ƕ�
 */
double rad2deg(double rad)
{
    return rad * 180 / PI;
}
/**
 * @brief      ������ƽ��
 * @param[in]  ��x
 * @return     ��ƽ��
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
 * @brief      �����㿪����
 * @param[in]  ��x
 * @return     ��������
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
 * @brief      ������sin
 * @param[in]  ��x
 * @return     ��sin
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
 * @brief      ������cos
 * @param[in]  ��x
 * @return     ��cos
 */
double func_cos(double x)

{

    x = PI / 2 - x;
    return func_sin(x);
}

/**
 * @brief       ����double�ľ���ֵ
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
 * @brief      ��ţ�ٵ��������㿪����
 *
 * @param       a  ��Ҫ����������
 * @param       x0 ��������ֵ
 * @return      double
 * @note        �ݹ����ʹ��ţ�ٵ���������ƽ�����ĺ��ĺ���
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
 * @brief      ����װţ�ٵ��������㿪��������ֵΪ1.0
 *
 * @param       x
 * @return      double
 */
double py_sqrt(double x)
{
    return k_sqrt(x, 1.0);
}

/**
 * @brief      ������׳�
 *
 * @param       x  ��Ҫ����׳˵�������
 * @return      long
 * @note        ��������n�Ľ׳�
 */
long factorial(int x)
{
    long result = 1;
    for (int i = 1; i <= x; i++) result *= i;

    return result;
}

/**
 * @brief      �����㷴���Һ���
 *
 * @param       x  ������ֵ����Χ[-1,1]
 * @return      double
 * @note        ʹ��̩�ռ���չ������arcsinֵ
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
 * @brief      �����ټ��㷴����ֵ
 *
 * @param       y  ��y����ֵ
 * @param       x  ��x����ֵ
 * @return      double
 * @note        ʹ�ò�������Բ�ֵʵ�ֿ���atan2����
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
 * @brief      ���Ƕȷ�Χת��
 *
 * @param       angle ����������fast_atan2���صĽǶ�ֵ
 * @return      ת����ĽǶ�ֵ
 * @note        ZERO_TO_TWOPI�궨����������Χ:
 *              ����ʱ�����ΧΪ[0,2��]
 *              δ����ʱ�����ΧΪ[-��,��]
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

#endif   // ��������Double��ѧ��

//////////////////////////////////////////////////////////////////////////////////////////////////
/*----------------------------------����ΪGPS����Doubleר����ѧ��----------------------------------*/
//////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////
/*----------------------------------����Ϊ��������ת��ר����ѧ��----------------------------------*/
//////////////////////////////////////////////////////////////////////////////////////////////////
// ʹ��������ȷ���ڴ������ȫ
typedef union
{
    double d64;
    uint32 u32[2];   // u32[0]Ϊ��32λ��С��ģʽ��
} DoubleConverter;

// ���doubleΪ�ṹ�壨ֱ�ӷ���ֵ��
DoubleParts Double_To_Parts(double value)
{
    DoubleConverter converter;
    converter.d64 = value;
    DoubleParts parts;
    parts.low  = converter.u32[0];
    parts.high = converter.u32[1];
    return parts;
}

// �ӽṹ��ָ�double
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

    // ���double���ṹ��
    DoubleParts parts = Double_To_Parts(original);

    // �ָ�double
    double recovered = Parts_To_Double(parts);

    // ��֤����
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
/*----------------------------------����Ϊ��������ת��ר����ѧ��----------------------------------*/
//////////////////////////////////////////////////////////////////////////////////////////////////
// ���һ��float�Ƿ�ΪNaN
bool my_isnan(float f)
{
    // ��floatת��Ϊuint32_t�Ա㰴λ���
    uint32_t i = *(uint32_t*)&f;

    // IEEE 754������NaN��λģʽ����
    // ָ��λȫΪ1��0x7F800000����β��λ��ȫΪ0
    const uint32_t nan_mask       = 0x7F800000;
    const uint32_t quiet_nan_mask = nan_mask | 0x00400000;   // ��������ֻ���ľ�ĬNaN

    // ����Ƿ�ΪNaN
    if ((i & nan_mask) == nan_mask && (i & ~quiet_nan_mask) != 0)
    {
        // ��������ֻ������Ƿ�ΪNaN��û�����־�ĬNaN���ź�NaN
        // �����ֻ���ľ�ĬNaN������ȥ�� & ~quiet_nan_mask �ļ��
        return true;
    }

    // �������NaN������false
    return false;
}

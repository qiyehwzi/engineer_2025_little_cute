/**
  ******************************************************************************
  * @file	 user_lib.c
  * @author  Wang Hongxi
  * @version V1.0.0
  * @date    2021/2/18
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#include "stdlib.h"
#include "string.h"
#include "user_lib.h"
#include "math.h"
#include "main.h"

#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif

uint8_t GlobalDebugMode = 7;

/**
  * @brief          һ�׵�ͨ�˲���ʼ��
  * @author         RM
  * @param[in]      һ�׵�ͨ�˲��ṹ��
  * @param[in]      �����ʱ�䣬��λ s
  * @param[in]      �˲�����
  * @retval         ���ؿ�
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num)
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num;
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief          һ�׵�ͨ�˲�����
  * @author         RM
  * @param[in]      һ�׵�ͨ�˲��ṹ��
  * @param[in]      �����ʱ�䣬��λ s
  * @retval         ���ؿ�
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
        first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period
	/ (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}


//���ٿ���
float Sqrt(float x)
{
    float y;
    float delta;
    float maxError;

    if (x <= 0)
    {
        return 0;
    }

    // initial guess
    y = x / 2;

    // refine
    maxError = x * 0.001f;

    do
    {
        delta = (y * y) - x;
        y -= delta / (2 * y);
    } while (delta > maxError || delta < -maxError);

    return y;
}

//������ƽ��������
/*
float invSqrt(float num)
{
    float halfnum = 0.5f * num;
    float y = num;
    long i = *(long *)&y;
    i = 0x5f375a86- (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}*/

/**
  * @brief          б��������ʼ��
  * @author         RM
  * @param[in]      б�������ṹ��
  * @param[in]      �����ʱ�䣬��λ s
  * @param[in]      ���ֵ
  * @param[in]      ��Сֵ
  * @retval         ���ؿ�
  */
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}

/**
  * @brief          б���������㣬���������ֵ���е��ӣ� ���뵥λΪ /s ��һ������������ֵ
  * @author         RM
  * @param[in]      б�������ṹ��
  * @param[in]      ����ֵ
  * @retval         ���ؿ�
  */
float ramp_calc(ramp_function_source_t *ramp_source_type, float input)
{
    ramp_source_type->input = input;
    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
    return ramp_source_type->out;
}




//��������
void abs_limit(fp32 *num, fp32 Limit)
{
    if (*num > Limit)
    {
        *num = Limit;
    }
    else if (*num < -Limit)
    {
        *num = -Limit;
    }
}

//�жϷ���λ
float sign(float value)
{
    if (value >= 0.0f)
    {
        return 1.0f;
    }
    else
    {
        return -1.0f;
    }
}

/*	Return arctangent(x) given that 0x1p-27 < |x| <= 1/2, with the same
	properties as atan.
*/
double atani0(double x)
{
	static const double p03 = -0x1.555555555551Bp-2;
	static const double p05 = +0x1.99999999918D8p-3;
	static const double p07 = -0x1.2492492179CA3p-3;
	static const double p09 = +0x1.C71C7096C2725p-4;
	static const double p11 = -0x1.745CF51795B21p-4;
	static const double p13 = +0x1.3B113F18AC049p-4;
	static const double p15 = -0x1.10F31279EC05Dp-4;
	static const double p17 = +0x1.DFE7B9674AE37p-5;
	static const double p19 = -0x1.A38CF590469ECp-5;
	static const double p21 = +0x1.56CDB5D887934p-5;
	static const double p23 = -0x1.C0EB85F543412p-6;
	static const double p25 = +0x1.4A9F5C4724056p-7;

	// Square x.
	double x2 = x * x;

	return ((((((((((((
		+ p25) * x2
		+ p23) * x2
		+ p21) * x2
		+ p19) * x2
		+ p17) * x2
		+ p15) * x2
		+ p13) * x2
		+ p11) * x2
		+ p09) * x2
		+ p07) * x2
		+ p05) * x2
		+ p03) * x2 * x + x;
}
//��������
float float_deadband(float Value, float minValue, float maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

//int26����
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }
    return Value;
}

//�޷�����
float float_constrain(float Value, float minValue, float maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}
//�޷�����
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}
//�޷�����
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//ѭ���޷�����
float loop_float_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

//���ȸ�ʽ��Ϊ-PI~PI

//�Ƕȸ�ʽ��Ϊ-180~180
float theta_format(float Ang)
{
    return loop_float_constrain(Ang, -180.0f, 180.0f);
}

int float_rounding(float raw)
{
    static int integer;
    static float decimal;
    integer = (int)raw;
    decimal = raw - integer;
    if (decimal > 0.5f)
        integer++;
    return integer;
}

/**
  * @brief          ��С���˷���ʼ��
  * @param[in]      ��С���˷��ṹ��
  * @param[in]      ������
  * @retval         ���ؿ�
  */
void OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order)
{
    OLS->Order = order;
    OLS->Count = 0;
    OLS->x = (float *)user_malloc(sizeof(float) * order);
    OLS->y = (float *)user_malloc(sizeof(float) * order);
    OLS->k = 0;
    OLS->b = 0;
    memset((void *)OLS->x, 0, sizeof(float) * order);
    memset((void *)OLS->y, 0, sizeof(float) * order);
    memset((void *)OLS->t, 0, sizeof(float) * 4);
}

/**
  * @brief          ��С���˷����
  * @param[in]      ��С���˷��ṹ��
  * @param[in]      �ź�����������һ������ʱ����
  * @param[in]      �ź�ֵ
  */
void OLS_Update(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }
    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);
    OLS->b = (OLS->t[0] * OLS->t[3] - OLS->t[1] * OLS->t[2]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= OLS->Order;
}

/**
  * @brief          ��С���˷���ȡ�ź�΢��
  * @param[in]      ��С���˷��ṹ��
  * @param[in]      �ź�����������һ������ʱ����
  * @param[in]      �ź�ֵ
  * @retval         ����б��k
  */
float OLS_Derivative(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }

    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= OLS->Order;

    return OLS->k;
}

/**
  * @brief          ��ȡ��С���˷���ȡ�ź�΢��
  * @param[in]      ��С���˷��ṹ��
  * @retval         ����б��k
  */
float Get_OLS_Derivative(Ordinary_Least_Squares_t *OLS)
{
    return OLS->k;
}

/**
  * @brief          ��С���˷�ƽ���ź�
  * @param[in]      ��С���˷��ṹ��
  * @param[in]      �ź�����������һ������ʱ����
  * @param[in]      �ź�ֵ
  * @retval         ����ƽ�����
  */
float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }

    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);
    OLS->b = (OLS->t[0] * OLS->t[3] - OLS->t[1] * OLS->t[2]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= OLS->Order;

    return OLS->k * OLS->x[OLS->Order - 1] + OLS->b;
}

/**
  * @brief          ��ȡ��С���˷�ƽ���ź�
  * @param[in]      ��С���˷��ṹ��
  * @retval         ����ƽ�����
  */
float Get_OLS_Smooth(Ordinary_Least_Squares_t *OLS)
{
    return OLS->k * OLS->x[OLS->Order - 1] + OLS->b;
}


//float angle_fabs(fp32 angle)
//{
//	if(angle>PI&&angle< 2*PI)
//	{ 
//		return angle - PI;
//	}
//	else if(angle<-PI&&angle >-2*PI)
//	{
//		return angle + PI;
//	}
//	else if(angle>2*PI&&angle <3*PI)
//	{
//		return angle + 2 * PI;
//	}
//	else if(angle<-2*PI&&angle >-3*PI)
//	{
//		return angle + 2*PI;
//	}
//	
//	else return angle;
//}


#include "adrc.h"
#include "math.h"
#include "user_lib.h"
#include "main.h"
//�޷�
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

//һ������һ�׵��ٶȿ��ƣ�������һ��ADRC�������нǶȿ���ʹ��
//һ��adrc������ʼ��
void ADRC_1_Init(ADRC_1_t *adrc,fp32 wo,fp32 wc,fp32 b,fp32 h,fp32 max_out)
{
    if (adrc == NULL)
    {
        return;
    }	
		adrc->parameter.wo = wo;
		adrc->parameter.wc = wc;
		adrc->parameter.b	 = b;
		adrc->parameter.h	 = h;	
		adrc->parameter.max_out = max_out;			
}

//ȫ�׵�LESO������Ϊ2-0=2
void ADRC_1_LESO(ADRC_1_t *adrc)
{
		//z[0]=h*z2 - y*(2*BB - 2) + z1*(2*BB - 1) + bo*h*u
		adrc->singal.z[0] = adrc->singal.z[0]*(adrc->parameter.Beita*2-1)+adrc->parameter.h*adrc->singal.z[1]
												-adrc->singal.w*(adrc->parameter.Beita*2-2)+adrc->parameter.b*adrc->parameter.h*adrc->singal.u;
		//z[1]=z2 + (y*(BB^2 - 2*BB + 1))/h - (z1*(BB^2 - 2*BB + 1))/h
		adrc->singal.z[1] = -(adrc->parameter.Beita_2-adrc->parameter.Beita*2+1)*adrc->singal.z[0]/adrc->parameter.h
												+adrc->singal.z[1]+(adrc->parameter.Beita_2-adrc->parameter.Beita*2+1)*adrc->singal.w/adrc->parameter.h;

		//yd[0]= BB^2*z1 - y*(BB^2 - 1)
		adrc->singal.y[0] = adrc->parameter.Beita_2*adrc->singal.z[0]-adrc->singal.w*(adrc->parameter.Beita_2-1);
		//yd[1]=z2 + (y*(BB^2 - 2*BB + 1))/h - (z1*(BB^2 - 2*BB + 1))/h
		adrc->singal.y[1] = -(adrc->parameter.Beita_2-adrc->parameter.Beita*2+1)*adrc->singal.z[0]/adrc->parameter.h
												+adrc->singal.z[1]	+(adrc->parameter.Beita_2-adrc->parameter.Beita*2+1)*adrc->singal.w/adrc->parameter.h;	
}

//һ��adrc����
fp32 ADRC_1_Calc(ADRC_1_t *adrc,fp32 w,fp32 ref)
{
    if (adrc == NULL)
    {
        return 0.0f;
    }
		//���²���
		adrc->parameter.Beita   = exp2f(-adrc->parameter.wo*adrc->parameter.h);
		adrc->parameter.Beita_2 = adrc->parameter.Beita*adrc->parameter.Beita;
		
		fp32 kp;
		kp = adrc->parameter.wc;
		adrc->singal.ref   = ref;		
		adrc->singal.w  	 = w;
		//���Ź۲���
		ADRC_1_LESO(adrc);
		//���Կ�����u0=kp*(ref-y)-kd*dy/dt-yd;
		adrc->singal.u = kp * (adrc->singal.ref-adrc->singal.y[0]) - adrc->singal.y[1] ;
		//u = u0/b
		adrc->singal.u = adrc->singal.u/adrc->parameter.b;
		//��u�޷�
		LimitMax(adrc->singal.u, adrc->parameter.max_out);
		
		return adrc->singal.u;
}


//һ�����ڶ��׵ĽǶȿ���
//adrc������ʼ��
void ADRC_2_Init(ADRC_2_t *adrc,fp32 wo,fp32 wc,fp32 b,fp32 a,fp32 h,fp32 max_out)
{
    if (adrc == NULL)
    {
        return;
    }	
	
		adrc->parameter.wo = wo;
		adrc->parameter.wc = wc;
		adrc->parameter.b	 = b;
		adrc->parameter.a	 = a;
		adrc->parameter.h	 = h;	
		adrc->parameter.max_out = max_out;			
}

//��һ�׵�LESO������Ϊ3-1=2
void ADRC_2_LESO(ADRC_2_t *adrc)
{

		adrc->singal.z[0] =  adrc->singal.z[0]*(1-adrc->parameter.L2*adrc->parameter.parameter_1-adrc->parameter.L1)
												+adrc->singal.z[1]*adrc->parameter.parameter_1
												+adrc->singal.u*adrc->parameter.b*adrc->parameter.parameter_1
												+adrc->singal.w*(adrc->parameter.L1+adrc->parameter.L2*adrc->parameter.parameter_1);
	
		adrc->singal.z[1] = -adrc->singal.z[0]*adrc->parameter.L2*adrc->parameter.parameter_2
												+adrc->singal.z[1]+adrc->parameter.parameter_2
												-adrc->singal.u*adrc->parameter.a*adrc->parameter.b*adrc->parameter.parameter_1
												+adrc->singal.w*adrc->parameter.L2*adrc->parameter.parameter_2;

		adrc->singal.y[0] =  adrc->singal.z[0]*(1-adrc->parameter.L1)+adrc->singal.w*adrc->parameter.L1;

		adrc->singal.y[1] = -adrc->singal.z[0]*adrc->parameter.L2 + adrc->singal.z[1]+adrc->singal.w*adrc->parameter.L2;
}

//����adrc����
fp32 ADRC_2_Calc(ADRC_2_t *adrc,fp32 angle,fp32 w,fp32 ref)
{
    if (adrc == NULL)
    {
        return 0.0f;
    }
		//���¼�����̼���Ҫ�Ĳ������������ϴ�
		adrc->parameter.Beita   = exp2f(-adrc->parameter.wo*adrc->parameter.h);
		adrc->parameter.Beita_2 = adrc->parameter.Beita*adrc->parameter.Beita;
		adrc->parameter.ah      = adrc->parameter.a*adrc->parameter.h;
		adrc->parameter.L1      = (adrc->parameter.ah * adrc->parameter.ah - 2 * adrc->parameter.ah - 2 * adrc->parameter.Beita_2 + 2)
												      /(adrc->parameter.ah * adrc->parameter.ah - 2 * adrc->parameter.ah + 2);
		
		adrc->parameter.L2      =  (adrc->parameter.ah * adrc->parameter.ah - 2 * adrc->parameter.ah - 2 * adrc->parameter.Beita + 2)
												      *(adrc->parameter.ah * adrc->parameter.ah - 2 * adrc->parameter.ah - 2 * adrc->parameter.Beita + 2)
													    /(- adrc->parameter.ah * adrc->parameter.ah * adrc->parameter.ah * adrc->parameter.h
													    +4 * adrc->parameter.ah * adrc->parameter.ah * adrc->parameter.h - 6 * adrc->parameter.ah * adrc->parameter.h
													    +4 * adrc->parameter.h);
   
		adrc->parameter.parameter_1 = adrc->parameter.h - adrc->parameter.ah * adrc->parameter.h / 2;
   	adrc->parameter.parameter_2 = adrc->parameter.ah * adrc->parameter.ah / 2 - adrc->parameter.ah + 1;
		
		//����������kp,kd
		fp32 kp,kd;
		kd = 2*adrc->parameter.wc;
		kp = adrc->parameter.wc*adrc->parameter.wc;
		adrc->singal.ref   = ref;		
		adrc->singal.w  	 = w;
		adrc->singal.angle = angle;
		//���Ź۲���
		ADRC_2_LESO(adrc);
		//���Կ�����u0=kp*(ref-y)-kd*dy/dt-yd;
		adrc->singal.u = kp * rad_format(adrc->singal.ref-adrc->singal.angle) - kd * adrc->singal.y[0] - adrc->singal.y[1];
		//u = u0/b
		adrc->singal.u = adrc->singal.u/adrc->parameter.b;
		//��u�޷�
		LimitMax(adrc->singal.u, adrc->parameter.max_out);
		
		return adrc->singal.u;
}

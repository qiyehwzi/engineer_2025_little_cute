#ifndef ADRC_H
#define ADRC_H
#include "struct_typedef.h"

typedef struct//һ��LADRC�Ĳ��������䣩�ṹ��
{		
	fp32 Beita;   	//��ɢ����LESO����£���=e^-woh
	fp32 Beita_2; 	//Beita_2=Beita*Beita
	fp32 wo;		//LESO����
	fp32 wc;		//����������
	fp32 b;       	//����������ϵ��	
	fp32 h;  		//�������������ڣ�
	fp32 max_out;	//�������޷�
} ADRC_1_parameter_t;

typedef struct		//һ��LADRC���źţ���ʱ��ı䣩�ṹ��
{		
    fp32 ref;		//�趨ֵ
    fp32 w;  		//��̨���ٶ�
	fp32 u;      	//��ϵͳ������or���������
	fp32 z[2];   	//LESO��״̬�����Ĺ۲�ֵ 
	fp32 y[2];   	//LESO�۲������
} ADRC_1_singal_t;

typedef struct		//һ��LADRC
{			
	ADRC_1_parameter_t parameter;
	ADRC_1_singal_t singal;
} ADRC_1_t;

extern void ADRC_1_Init(ADRC_1_t *adrc,fp32 wo,fp32 wc,fp32 b,fp32 h,fp32 max_out);
extern fp32 ADRC_1_Calc(ADRC_1_t *adrc,fp32 w,fp32 ref);

typedef struct//��һ�׵Ķ���LADRC�Ĳ��������䣩�ṹ��
{		
	fp32 Beita;        //��ɢ����LESO����£���=e^-woh
	fp32 Beita_2;      //Beita_2=Beita*Beita
    fp32 L1;           //������̼���Ҫ�Ĳ���
    fp32 L2	;          //������̼���Ҫ�Ĳ���
	fp32 ah;           //������̼���Ҫ�Ĳ���
    fp32 parameter_1;  //������̼���Ҫ�Ĳ���
    fp32 parameter_2;  //������̼���Ҫ�Ĳ���
	fp32 wo;		   //LESO����
	fp32 wc;		   //����������
	fp32 b;            //����������ϵ��
	fp32 a;            //��������ģ�� a=1/Tmת�س�������ͨ��ϵͳ��ʶ��ã�
	fp32 h;            //�������������ڣ�
	fp32 max_out;      //�������޷�
} ADRC_2_parameter_t;

typedef struct//��һ�׵Ķ���LADRC���źţ���ʱ��ı䣩�ṹ��
{		
    fp32 ref;		 	//�趨ֵ
    fp32 angle;  		//��̨�Ƕ�
    fp32 w;  		 	//��̨���ٶ�
	fp32 u;      		//��ϵͳ������or���������
	fp32 z[2];   		//LESO��״̬�����Ĺ۲�ֵ 
	fp32 y[2];   		//LESO�۲������
} ADRC_2_singal_t;

typedef struct//��һ�׵Ķ���LADRC
{		
	ADRC_2_parameter_t 	parameter;
	ADRC_2_singal_t   	singal;
} ADRC_2_t;

extern void ADRC_2_Init(ADRC_2_t *adrc,fp32 wo,fp32 wc,fp32 b,fp32 a,fp32 h,fp32 max_out);
extern fp32 ADRC_2_Calc(ADRC_2_t *adrc,fp32 angle,fp32 w,fp32 ref);

#endif

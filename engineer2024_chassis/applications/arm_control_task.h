#ifndef ARM_CONTROL_TASK_H
#define ARM_CONTROL_TASK_H

#include "struct_typedef.h"
#include "6dof_kinematic.h"
#include "Butterworth_Filter.h"
#include "TD.h"

#define LIFE_INIT 19.4f
#define LIFT_MIGRATION +0.23f

typedef enum
{
	Ag1 = 0,
	Ag2 = 1,
	Ag3 = 2,
	EXCHANGE1 = 3,
	EXCHANGE2 = 4,
	Au1 = 5,
	Au2 = 6,
	Au3 = 7,
	NORMAL_POSITION = 8,
}arm_move_e;

typedef struct
{
	fp32 Ag1[11][7];
	fp32 Ag2[10][7];
	fp32 Ag3[10][7];
	fp32 exchange1[10][7];
	fp32 exchange2[10][7];
	fp32 Au1[10][7];
	fp32 Au2[10][7];
	fp32 Au3[10][7];
	int16_t flag_and_time[8][11][2];
}arm_routine_t;

typedef struct
{
	arm_routine_t arm_move_routine; //��е�ۼ����˶���ʽ·��
	arm_move_e arm_move_flag; //��е���˶���ʽ��־λ
	uint8_t arm_position_flag; //��е��ĩ�˵�λ��־λ
	uint16_t arm_get_position_flag; // ��е���Ƿ񵽴��趨λ�ñ�־λ
	
	fp32 one_key_position[7];
	fp32 repostion_position[7];
	fp32 three_ore_position[7];
	fp32 pre_Au_reposition[7];
	fp32 Au_reposition[7];
	fp32 Ag_reposition[7];
	uint8_t routine_length[9];
	
	fp32 motor_1_position;
	fp32 motor_2_position;
	fp32 motor_3_position;
	fp32 motor_4_position;
	fp32 motor_5_position;
	fp32 motor_6_position;
	fp32 motor_7_position;
	
	fp32 lift_adjustion;
	float dt;
	uint32_t  DWT_Count;	
}arm_control_t;

typedef struct
{
	fp32 angleLimitMax;
	fp32 angleLimitMin;
}motor_t;

typedef struct
{
	int 			xyz_ValidCnt; // ����������������Ʒ�Χ�ڵĽ�ĸ���
	int 			ValidCnt;
	float 		Joint_Valid[4][5];  //����������Ʒ�Χ�ڵĽ�
	float     Joint_Final[5];     //�������ѡ����������ս�
	float     high;
	Robotic_6DOF  Robotic_6D;
	
	Pose6D_t 			Pose6D_FK;//��е��ĩ�˵�λ��
	Joint6D_t 		Joint6D_FK;//��е�۸��ؽڽǶ�
	
	Pose6D_t 			Pose6D_IK;//��е��Ŀ��λ��
	Solver6D_t 		output_solvers_IK;//��е���������������4��⣩
	
	motor_t 			Joint[5];//�ؽ������С��Χ����
	
	TD_t Pose6D_IK_X_TD;
	TD_t Pose6D_IK_Y_TD;
	TD_t Pose6D_IK_Z_TD;
	
}Robotic_6DOF_control_t;
typedef struct
{
	fp32 Ag1;
	fp32 Ag2;
	fp32 Ag3;
}Ag_Catch_t;            //��¼ץ��̧����ʼλ��
extern void arm_control_task(void const * argument);
extern arm_control_t arm_control;

extern uint8_t suker_key_flag;
extern int G_flag;
//��Ԫ��ģʽ���Զ��������ģʽ��
#define Quaterniont_Mode 1 
#define MaxJointSpeed 4.0f

#endif


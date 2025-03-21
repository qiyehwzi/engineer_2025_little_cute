#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include "Butterworth_Filter.h"
#include "struct_typedef.h"
#include "can_receive.h"
#include "controller.h"
#include "user_lib.h"
#include "TD.h"

//�����ʼ����ʱ
//Ҫʵ��2006��λ�������㹻����ʱ����2006�ϵ�
#define ARM_TASK_INIT_TIME	2000

#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8192
#define REDUCTION_RATIO_3508				187.0f/3591.0f
#define REDUCTION_RATIO_2006        1.0f/36.0f
#define MOTOR_ECD_TO_ANGLE_3508     0.00003994074176199f  
#define MOTOR_ECD_TO_ANGLE_2006     0.000021305288720633f
#define RPM_TO_RAD_S							  0.104719755119659774f

#define ROLL_TO_2006								25.0f/18.0f
#define YAW_TO_2006									1/((18.0f/25.0f)*(18.0f/40.0f))

#define G_2006                      1000.0f //����ʱ2006����������
//Yaw��ǶȻ�
#define ARM_YAW_PID_KP 							22.0f
#define ARM_YAW_PID_KI							6.0f
#define ARM_YAW_PID_KD							0.01f
#define ARM_YAW_PID_MAX_OUT					8.88f
#define ARM_YAW_PID_MAX_IOUT				2.0f
//Roll�ǶȻ�
#define ARM_ROLL_PID_KP 						22.0f
#define ARM_ROLL_PID_KI							6.0f
#define ARM_ROLL_PID_KD							0.01f
#define ARM_ROLL_PID_MAX_OUT				4.0f
#define ARM_ROLL_PID_MAX_IOUT				1.0f

//2006����ٶȻ�PID
#define M2006_SPEED_PID_KP_0 1000.0f   //��ʼ����
#define M2006_SPEED_PID_KP_1 900.0f 
#define M2006_SPEED_PID_KI 200.0f
#define M2006_SPEED_PID_KD 0.001f
#define M2006_ROTATE_SPEED_PID_MAX_OUT  10000.0f
#define M2006_ROTATE_SPEED_PID_MAX_IOUT 2000.0f

//3508����ٶȻ�PID
#define M3508_SPEED_PID_KP 2000.0f   
#define M3508_SPEED_PID_KI 10.0f
#define M3508_SPEED_PID_KD 0.04f
#define M3508_SPEED_PID_MAX_OUT      16383.0f
#define M3508_SPEED_PID_MAX_IOUT 		 2000.0f

#define M3508_POSITION_PID_KP 			 30.0f   
#define M3508_POSITION_PID_KI			   5.0f
#define M3508_POSITION_PID_KD 			 0.2f
#define M3508_POSITION_PID_MAX_OUT   30.0f
#define M3508_POSITION_PID_MAX_IOUT  3.0f


typedef struct
{
	const motor_measure_t *motor_measure;
	fp32 angle_set;
	fp32 angle;
	fp32 speed_set;
	fp32 speed;
	uint16_t offset_ecd;
	int round_cnt;
	int16_t give_current;
}motor_3508_t;


typedef struct
{
	const motor_DM_motor_t *DM_motor_measure;
	fp32 position_set;
}motor_DM_t;


typedef struct
{
	const motor_measure_t *motor_measure;
	fp32 angle_set;
	fp32 angle;
	fp32 speed_set;
	fp32 speed;
	uint16_t offset_ecd;
	int round_cnt;
	int16_t give_current;
}motor_2006_t;

typedef struct
{
	motor_3508_t motor_3508_data;
	motor_DM_t motor_DM_data[3];
	motor_2006_t motor_2006_data[2];
	
	PID_t motor_3508_speed_pid;
	PID_t motor_3508_angle_pid;
	PID_t motor_2006_speed_pid[2];
	PID_t yaw_angle_pid;
	PID_t roll_angle_pid;
	
	TD_t arm_1_TD;
	TD_t arm_2_TD;
	TD_t arm_3_TD;
	TD_t arm_4_TD;
	TD_t arm_5_TD;
	TD_t arm_6_TD;
		
	uint16_t arm_mode;
	uint16_t arm_last_mode;	
	uint8_t  motor_3508_mode;
	uint8_t  motor_2006_mode;
	uint8_t  motor_2006_last_mode;
	
	//ĩ��׶���ֿ��Ƶ������Ƕ�
	fp32 roll_angle;
	fp32 roll_angle_set;
	fp32 yaw_angle;
	fp32 yaw_angle_set;

	float dt;
	uint32_t  DWT_Count;	
}arm_t;


extern void arm_task(void const * argument);
extern arm_t arm;

#endif



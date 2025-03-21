#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "struct_typedef.h"
#include "remote_control.h"
#include "can_communicate.h"
#include "adrc.h"
#include "pid.h"
#include "kalman_filter.h"
#include "TD.h"
#include "controller.h"
#include "gimbal_behaviour.h"
#define GIMBAL_YAW_OFFSET_ECD  836
//������ֵ�Ƕ�
#define GIMBAL_PITCH_ABSOLUTE_ANGLE_MAX 0.6f
#define GIMBAL_PITCH_ABSOLUTE_ANGLE_MIN -0.36f

//yaw�ٶȻ�PID�����Լ�PID���������������
#define YAW_SPEED_PID_KP       	35000.0f 
#define YAW_SPEED_PID_KI        3000.0f 
#define YAW_SPEED_PID_KD        0.5f
#define YAW_SPEED_PID_MAX_OUT   30000.0f
#define YAW_SPEED_PID_MAX_IOUT  1000.0f
//yaw�ǶȻ��Ƕ��������ǽ���PID�����Լ�PID���������������
#define YAW_ABSOLUTE_ANGLE_PID_KP        16.0f   
#define YAW_ABSOLUTE_ANGLE_PID_KI        0.002f  
#define YAW_ABSOLUTE_ANGLE_PID_KD        0.12f  
#define YAW_ABSOLUTE_ANGLE_PID_MAX_OUT   9.0f
#define YAW_ABSOLUTE_ANGLE_PID_MAX_IOUT  0.3f 
//pitch�ǶȻ��Ƕ��������ǽ���PID�����Լ�PID���������������
#define PITCH_ABSOLUTE_ANGLE_PID_KP 			  22.0f
#define PITCH_ABSOLUTE_ANGLE_PID_KI 		    0.0015f
#define PITCH_ABSOLUTE_ANGLE_PID_KD 			  0.15f
#define PITCH_ABSOLUTE_ANGLE_PID_MAX_OUT 	  6.0f
#define PITCH_ABSOLUTE_ANGLE_PID_MAX_IOUT 	0.1f
//pitch�ٶȻ�PID�����Լ�PID���������������
#define PITCH_SPEED_PID_KP        	4.2f
#define PITCH_SPEED_PID_KI        	0.1f
#define PITCH_SPEED_PID_KD        	0.001f
#define PITCH_SPEED_PID_MAX_OUT		  10.0f
#define PITCH_SPEED_PID_MAX_IOUT 	  0.3f


//��̨�Ƿ�ʹ��ADRC����
#define ADRC_MODE 0
// PC�Զ���׼�����ȣ������Ŵ�����
#define YAW_PC_SEN 1
#define PITCH_PC_SEN 1
//yaw,pitch����ͨ���Լ�״̬����ͨ����ң������
#define YAW_CHANNEL			2
#define PITCH_CHANNEL 		3
#define GIMBAL_MODE_CHANNEL	0
//��X��ͷ180��
#define TURN_KEYBOARD 	KEY_PRESSED_OFFSET_X
//��ͷ��̨�ٶ�
#define TURN_SPEED	0.005f
//ң�����������������
#define RC_DEADBAND		10
#define MOUSE_DEADBAND 	0
//ң��������������ȣ������Ŵ�����
#define YAW_RC_SEN 		-0.000013f
#define PITCH_RC_SEN 	0.000008f
#define YAW_MOUSE_SMALLSEN   -0.00015f
#define PITCH_MOUSE_SMALLSEN -0.00015f

//�����ʼ����ʱ
#define GIMBAL_TASK_INIT_TIME	1000
//��̨��������ڣ�1ms��
#define GIMBAL_CONTROL_TIME	1
//yaw,pitch�����������������
#define PITCH_TURN 	1
#define YAW_TURN 	0
//�������ֵ����Լ���ֵ
#define HALF_ECD_RANGE 	4096
#define ECD_RANGE 		8191
//�������ֵת���ɽǶ�ֵ��ת����ʽΪ2*PI/8192
#define MOTOR_ECD_TO_RAD 0.000766990394f
//yaw������ʱ
#define TIMEOUT_MAX 1000
//pitch������
#define PITCH_M                                         1.8f//kg
#define PITCH_L                                         0.05f//m
#define GRAVITY                                         9.8f
#define KT                                              0.87f//N*m/AС�׵�����ص���ת��ϵ��
//�ǶȻ�PID�ṹ��
typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;
    fp32 set;
    fp32 get;
    fp32 err;
    fp32 max_out;
    fp32 max_iout;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 out;
} gimbal_PID_t;
//����ṹ��
typedef struct
{
    const motor_measure_t *gimbal_motor_measure;//������ݣ�����������ֵ��
    gimbal_PID_t gimbal_motor_absolute_angle_pid; //����ֵPID
    PID_t gimbal_motor_gyro_pid;  //������PID
    TD_t PC_Control_Td;
    uint16_t offset_ecd;
	  fp32 absolute_angle_offset;
    //���½Ƕȡ����ٶȾ�Ϊ������
		fp32 max_absolute_angle;
    fp32 min_absolute_angle;
    fp32 absolute_angle;
    fp32 absolute_angle_set;
    fp32 motor_gyro;
    fp32 motor_gyro_set;
		float32_t gravity_feedforward_current;
	
    fp32 raw_cmd_current;
    fp32 current_get;
    fp32 current_last;
    fp32 current_set;
    fp32 given_current;
} gimbal_motor_t;
//��̨�ṹ��
typedef struct
{
	  gimbal_mode_e gimbal_mode_last;
    gimbal_mode_e gimbal_mode;
	  sREMOTE_data *gimbal_remote_data;//ң����
    const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;
    gimbal_motor_t gimbal_yaw_motor;  //Y��ģʽ
    gimbal_motor_t gimbal_pitch_motor; //P��ģʽ
	
		float dt;
		uint32_t dwt_count;
} gimbal_control_t;

extern gimbal_control_t gimbal_control;

#endif

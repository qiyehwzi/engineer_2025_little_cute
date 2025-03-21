#ifndef SHOOT_H
#define SHOOT_H
#include "struct_typedef.h"
#include "remote_control.h"
#include "can_communicate.h"
#include "pid.h"
#include "adrc.h"
#include "controller.h"
#include "pc_task.h"
#include "td.h"
//****************************�̶�ֵ***********************************//
//�벦����صĽǶ�
#define PI_ONE											3.141592653589793f
#define PI_FOUR                     0.78539816339744830961566084581988f
#define SHOOT_ADJUST_ANGLE          PI_FOUR/3
//У׼��ļ��ٱ�
#define REDUCTION_RATIO  						36.0f
//������ֵת��Ϊ���ȵ�ϵ��
#define MOTOR_ECD_TO_ANGLE          0.0007669903939f 

//���rmp �仯�� ��ת�ٶȵı���
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596f

//���Ħ���ּ���� �ر�
#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E

//****************************����ֵ***********************************//
//��곤���ж�
#define PRESS_LONG_TIME             0.0f

//ң����������ش��µ�һ��ʱ��� ���������ӵ� �����嵥
#define RC_S_LONG_TIME              2000
#define PC_TIME                     50
//Ħ���ָ��� ���� ʱ��
#define UP_ADD_TIME                 80
//Ħ�����ٶ�
#define BULLET_SPEED_SET          	28.0f
#define SHOOT_FIRE_SPEED_HIGH       21.0f//����
#define SHOOT_FIRE_SPEED_LOW        20.5f //����
#define BULLET_SPEED_FIX_K          0.1f           
//������俪��ͨ������
#define SHOOT_RC_MODE_CHANNEL       1

//�����ɺ� �ӵ�����ȥ���ж�ʱ�䣬�Է��󴥷�
#define SHOOT_DONE_KEY_OFF_TIME     15

//�����ٶ�
#define SHOOT_ADJUST_SPEED            2.0f 
#define SHOOT_HIGH_TRIGGER_SPEED      15.0f 
#define SHOOT_LOW_TRIGGER_SPEED       11.0f 
//��ת�Ƕ�
#define TRIGGER_BLOCK_ANGLE					0.1f
//��ת�ٶ�
#define TRIGGER_BLOCK_SPEED			    1.0f	
#define TRIGGER_BLOCK_TIME          500

//�����ֵ���ٶȻ����ǶȻ�PID
#define TRIGGER_SPEED_PID_KP        1200.0f
#define TRIGGER_SPEED_PID_KI        800.0f
#define TRIGGER_SPEED_PID_KD        1.0f
#define TRIGGER_SPEED_PID_MAX_OUT   10000.0f
#define TRIGGER_SPEED_PID_MAX_IOUT  2000.0f
#define TRIGGER_ANGLE_PID_KP          20.0f
#define TRIGGER_ANGLE_PID_KI          2.0f
#define TRIGGER_ANGLE_PID_KD          0.1f
#define TRIGGER_ANGLE_PID_MAX_OUT     8.0f
#define TRIGGER_ANGLE_PID_MAX_IOUT    0.5f
//Ħ���ֵ���ٶȻ�PID
#define FRIC_SPEED_PID_KP        	7000.0f
#define FRIC_SPEED_PID_KI        	1.0f
#define FRIC_SPEED_PID_KD        	0.01f
#define FRIC_SPEED_PID_MAX_OUT      16383.0f
#define FRIC_SPEED_PID_MAX_IOUT   	2000.0f

#define FRIC_DEADBAND    0.2

//������������

#define SHOOT_HEAT_REMAIN_VALUE     10
#define SHOOT_HEAT_REMAIN_VALUE_1     40
#define SHOOT_HEAT_REMAIN_VALUE_2     80

typedef enum
{
    SHOOT_STOP = 0,     //����ֹͣ
    SHOOT_READY_FRIC,  //�ȴ�Ħ����
    SHOOT_READY,      //����ȴ�
    SHOOT_BULLET,   //����
    SHOOT_CONTINUE_BULLET,//����
    SHOOT_DONE,   //������
	  SHOOT_ADJUST,  //�������
} shoot_mode_e;
typedef struct{
	const rx_data_t *pc_rx_data;   //����
	uint8_t pc_continue_flag ;
	uint16_t pc_continue_count;
}shoot_pc_t;
typedef struct{
		uint16_t heat_limit;
    uint16_t heat;
	  uint16_t cooling_value;
}shoot_referee_t;
typedef struct
{
    shoot_mode_e shoot_mode;
	  shoot_mode_e shoot_mode_last;
    sREMOTE_data *shoot_remote_data;
	  const RC_ctrl_t *shoot_rc;
    const motor_measure_t *trigger_motor_measure;
    const motor_measure_t *fric1_motor_measure;
    const motor_measure_t *fric2_motor_measure;
    PID_t trigger_speed_pid;
    pid_type_def trigger_angle_pid;
    PID_t fric_motor_pid[2];
    float32_t bullet_speed;                          //����
	  float32_t bullet_speed_delta;                    //������ʵ�ʵ��ٵĲ�
    float32_t bullet_speed_fix;                      //��������ϵ��
  	float32_t bullet_speed_set;                      //�趨����
    fp32 trigger_speed_set;   //�����ֵĲ����ٶ�
	  fp32 trigger_speed;	
		fp32 trigger_angle;
	  fp32 trigger_angle_per;
    fp32 trigger_angle_set;
	  TD_t      fric_speed_set_td;
    fp32 fric_speed[2];
    fp32 fric_speedset[2];		    
    fp32 fric_speed_set;
    int16_t fric_given_current[2];
		 
    int16_t trigger_given_current;

    uint16_t rc_s_time;//���뿪��ʱ��

    uint16_t block_time;
    uint16_t reverse_time;
    bool_t move_flag;
		
		shoot_referee_t  shoot_referee_data;
    shoot_pc_t  shoot_pc_data;
} shoot_control_t;

extern uint8_t shoot_fire_state;	
extern uint8_t block_flag;
extern shoot_control_t shoot_control;//�������
extern void shoot_init(void);
extern int16_t shoot_control_loop(void);
extern int16_t get_fric1(void);
extern int16_t get_fric2(void);
extern void shoot_trigger_angle_update(void);
#endif


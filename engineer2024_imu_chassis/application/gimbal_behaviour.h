#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H
#include "struct_typedef.h"


typedef enum
{
    GIMBAL_MODE_ZERO_FORCE = 0, //����ģʽ
    GIMBAL_MODE_ABSOLUTE_ANGLE, //������ģʽ���������Ƿ���
    GIMBAL_MODE_PC,             //PC������׼
} gimbal_mode_e;
//ͨ��ң�������ò����Ĺ���ģʽ����̨�Ͳ�����ͬ������Բ�ͬ�Ĺ���ģʽ���ڲ�ͬ�ĵ�����Ϊ���ʹ���ģʽ>������Ϊ���ͣ���ͬ�Ĺ���ģʽ����ʹ����ͬ�ĵ�����Ϊģʽ
typedef enum
{
    INFANTRY_MODE_RAW,                   //��������, ��û�ϵ�����
    INFANTRY_MODE_NO_FOLLOW_YAW,                //���̲�������̨�Ƕ�
    INFANTRY_MODE_FOLLOW_YAW,                   //�����������̸�����̨
    INFANTRY_MODE_GYRO,                          //С����ģʽ
	  INFANTRY_MODE_PC,
} infantry_mode_e;

extern void gimbal_behaviour_mode_set(void);
extern void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch);
extern bool_t gimbal_cmd_to_shoot_stop(void);
#endif


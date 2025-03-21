#include "gimbal_behaviour.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "arm_math.h"
#include "laser.h"
#include "pc_task.h"
#include "TD.h"
#include "gimbal_task.h"

//ң������������
#define rc_deadband_limit(input, output, dealine)		\
{		 	                                            \
	if ((input) > (dealine) || (input) < -(dealine)) 	\
	{                                              		\
		(output) = (input);                          	\
	}                                                	\
	else                                             	\
	{                                                	\
		(output) = 0;                                	\
	}                                                	\
}
//��̨�Ƿ����ڷ�ת�ı�־λ
int8_t gimbal_turn_flag = 0;

//���⣬���������Ϊģʽ����
infantry_mode_e infantry_mode = INFANTRY_MODE_RAW;
infantry_mode_e infantry_mode_last = INFANTRY_MODE_RAW;

static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_pc_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
/**
  * @brief          ��Ϊģʽ����
  * @param[in]      none
  * @retval         none
  */
static void infantry_mode_set(void) {
	  //ң����������̨״̬
    //���±�ʾ�ɳ�ģʽ��������ϵ�
    if (switch_is_down(gimbal_control.gimbal_remote_data->rc_ctrl_t->rc.s[GIMBAL_MODE_CHANNEL]))   	//����С����
    {
		  	infantry_mode = INFANTRY_MODE_RAW;

    }
    //���б�ʾ������ģʽ�����õ��̺���̨����ԽǶȷ���
    else if (switch_is_mid(gimbal_control.gimbal_remote_data->rc_ctrl_t->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        infantry_mode = INFANTRY_MODE_FOLLOW_YAW;
		   //���Ҽ����߰�B�����㷨ͨ��
        if(gimbal_control.gimbal_remote_data->rc_ctrl_t->mouse.press_r)
        {
            infantry_mode = INFANTRY_MODE_PC;
        }
    }
    //���ϲ��������Ǿ��ԽǶȿ���
    else if (switch_is_up(gimbal_control.gimbal_remote_data->rc_ctrl_t->rc.s[GIMBAL_MODE_CHANNEL]))
    {
            infantry_mode = INFANTRY_MODE_PC;
    }
       infantry_mode_last = infantry_mode; //��¼��һ��ģʽ
	}
void gimbal_behaviour_mode_set()
{

    //������Ϊ״̬����
    infantry_mode_set();
    //������̨��Ϊ״̬�����õ��״̬
    //��̨�ɳ�ģʽ(����)
    if (infantry_mode == INFANTRY_MODE_RAW)
    {
        gimbal_control.gimbal_mode= GIMBAL_MODE_ZERO_FORCE;
    }
    //������ģʽ���������Ƿ���
    else if (infantry_mode == INFANTRY_MODE_FOLLOW_YAW || infantry_mode == INFANTRY_MODE_NO_FOLLOW_YAW \
			||infantry_mode == INFANTRY_MODE_GYRO)
    {
        gimbal_control.gimbal_mode= GIMBAL_MODE_ABSOLUTE_ANGLE;
    }

    //PC������׼
    else if(infantry_mode == INFANTRY_MODE_PC)
    {
        gimbal_control.gimbal_mode= GIMBAL_MODE_PC;
    }
		
}

//��̨��Ϊ����
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch)
{
    if (add_yaw == NULL || add_pitch == NULL)
    {
        return;
    }
    if ( gimbal_control.gimbal_mode == GIMBAL_MODE_ZERO_FORCE)
    {
        gimbal_zero_force_control(add_yaw, add_pitch, &gimbal_control);
    }
    else if (gimbal_control.gimbal_mode  == GIMBAL_MODE_ABSOLUTE_ANGLE)
    {
        gimbal_absolute_angle_control(add_yaw, add_pitch, &gimbal_control);
    }
    else if ( gimbal_control.gimbal_mode  == GIMBAL_MODE_PC)
    {
        gimbal_pc_angle_control(add_yaw, add_pitch, &gimbal_control);
    }
}

//����ģʽ����
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}

//������ģʽ����
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static int16_t yaw_channel = 0, pitch_channel = 0;
    static float yaw_channel_mouse = 0, pitch_channel_mouse = 0;
 //   static float last_yaw_channel_mouse = 0, last_pitch_channel_mouse = 0;
    //ң������������
    rc_deadband_limit(gimbal_control_set->gimbal_remote_data->rc_ctrl_t->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);//Y ��
    rc_deadband_limit(gimbal_control_set->gimbal_remote_data->rc_ctrl_t->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);//P��
    rc_deadband_limit(gimbal_control_set->gimbal_remote_data->rc_ctrl_t->mouse.x, yaw_channel_mouse, MOUSE_DEADBAND);//���X��仯
    rc_deadband_limit(gimbal_control_set->gimbal_remote_data->rc_ctrl_t->mouse.y, pitch_channel_mouse, MOUSE_DEADBAND);//���Y��仯
     *yaw = yaw_channel * YAW_RC_SEN + yaw_channel_mouse * YAW_MOUSE_SMALLSEN;
     *pitch = pitch_channel * PITCH_RC_SEN + pitch_channel_mouse * YAW_MOUSE_SMALLSEN;
    static uint16_t last_turn_keyboard = 0;//��һ�ΰ���
    static fp32 gimbal_end_angle = 0.0f;
    //��ͷ���ܣ�X����
    if ((gimbal_control_set->gimbal_remote_data->rc_ctrl_t->key.v & TURN_KEYBOARD) && !(last_turn_keyboard & TURN_KEYBOARD))//��ֹ������
    {
        if (gimbal_turn_flag == 0)
        {
            gimbal_turn_flag = 1;
            //�����ͷ��Ŀ��ֵ
            gimbal_end_angle = rad_format(gimbal_control_set->gimbal_yaw_motor.absolute_angle + PI);
        }
    }
    last_turn_keyboard = gimbal_control_set->gimbal_remote_data->rc_ctrl_t->key.v;
    if (gimbal_turn_flag)
    {
        //���Ƶ�ͷ��Ŀ��ֵ
        if (rad_format(gimbal_end_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle) > 0.0f)//���ȸ�ʽ��Ϊ-PI~PI
        {
            *yaw += TURN_SPEED;//ÿ�θı�ĽǶ�
        }
        else
        {
            *yaw -= TURN_SPEED;
        }
    }
    //ת��PI��ֹͣ
    if (gimbal_turn_flag && fabs(rad_format(gimbal_end_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle)) < 0.01f)
    {
        gimbal_turn_flag = 0;
    }
}

//PCģʽ����
static void gimbal_pc_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
	if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
	{
		return;
    }
	static float yaw_channel = 0, pitch_channel = 0 ;

	yaw_channel = pc_receive_msg.rx_data.shoot_yaw;
	pitch_channel = pc_receive_msg.rx_data.shoot_pitch;

//	static const fp32 pc_fliter_num[3] = {1.822751349748f, -0.8371810677374f,  0.01442971798952f};
//	static fp32 pitch_fliter[3] = {0.0f, 0.0f, 0.0f};
//	static fp32 yaw_fliter[3] = {0.0f, 0.0f, 0.0f};
//	pitch_fliter[0] = pitch_fliter[1];
//	pitch_fliter[1] = pitch_fliter[2];
//	pitch_fliter[2] = pitch_fliter[1] * pc_fliter_num[0] + pitch_fliter[0] * pc_fliter_num[1] +  pitch_channel * pc_fliter_num[2];

//	yaw_fliter[0] = yaw_fliter[1];
//	yaw_fliter[1] = yaw_fliter[2];
		
//	yaw_fliter[2] = yaw_fliter[1] * pc_fliter_num[0] + yaw_fliter[0] * pc_fliter_num[1] +  yaw_channel * pc_fliter_num[2];
//	*yaw = yaw_fliter[2] * YAW_PC_SEN; 
//	*pitch = pitch_fliter[2] * PITCH_PC_SEN;	
	TD_calc(&gimbal_control_set->gimbal_yaw_motor.PC_Control_Td,yaw_channel * YAW_PC_SEN);
	TD_calc(&gimbal_control_set->gimbal_pitch_motor.PC_Control_Td,pitch_channel * PITCH_PC_SEN);
	if(fabs(gimbal_control_set->gimbal_yaw_motor.absolute_angle *  pc_receive_msg.rx_data.shoot_yaw) > 9.3f){
		gimbal_control_set->gimbal_yaw_motor.PC_Control_Td.x = pc_receive_msg.rx_data.shoot_yaw;
	}
	*yaw = gimbal_control_set->gimbal_yaw_motor.PC_Control_Td.x;
	*pitch = gimbal_control_set->gimbal_pitch_motor.PC_Control_Td.x;	
//	*yaw = yaw_channel;
//	*pitch = pitch_channel;	
}

//����������ģʽ��ֹͣ���
bool_t gimbal_cmd_to_shoot_stop(void)
{
    if (gimbal_control.gimbal_mode == GIMBAL_MODE_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

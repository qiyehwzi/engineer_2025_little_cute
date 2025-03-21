#include "A_communicate_task.h"
#include "Butterworth_Filter.h"
#include "arm_control_task.h"
#include "CAN_receive.h"
#include "controller.h"
#include "arm_math.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "bsp_dwt.h"
#include "timers.h"
#include "stdio.h"
#include "task.h"
#include "main.h"
#include "TD.h"


static void arm_control_init(arm_t *arm_control_init);
static void arm_feedback_update(arm_t *arm_feedback);
static void arm_set_mode(arm_t *arm_set_mode);
static void arm_set_control(arm_t *arm_set_control);
static void arm_control_loop(arm_t *arm_control_loop);


//��е�۽ṹ��
arm_t arm;
uint8_t  reset_3508_switch;
uint32_t reset_3508_count[2];
uint32_t reset_2006_count[2];
float32_t g_2006 = 249.0f;
float32_t mode2_cnt = 0;
void arm_task(void const * argument)
{	
		//�ȴ������������������������
		vTaskDelay(ARM_TASK_INIT_TIME);
		//��ʼ��arm���
		arm_control_init(&arm);
		//����dwt��ʱ����ʱ����
		uint32_t system_clock = osKernelSysTick();

		while(1)
		{
				arm.dt = DWT_GetDeltaT(&arm.DWT_Count);	

				arm_set_mode(&arm);		
				arm_feedback_update(&arm);
				arm_set_control(&arm);
				arm_control_loop(&arm);
			  //1-3508
				CAN_cmd_3508(arm.motor_3508_data.give_current);			
				//2-4�����ָ�� ��������δд
				if(arm.arm_mode == 0)
				{
						CAN_cmd_4310_1_disable();						
						DWT_Delay(0.0003f);
						CAN_cmd_4310_2_disable();		
						DWT_Delay(0.0003f);
						CAN_cmd_4310_3_disable();		
						DWT_Delay(0.0003f);
				}
				else if(arm.arm_mode != 0)
				{
						if(arm.motor_DM_data[0].DM_motor_measure->motor_enabled == 0)
								CAN_cmd_4310_1_enable();					
						else if(arm.motor_DM_data[0].DM_motor_measure->motor_enabled == 1)	
								CAN_cmd_4310_mit_1(arm.motor_DM_data[0].position_set,0.0f,200.0f,5.0f,0.0f);
						else
								CAN_cmd_4310_1_disable();	
								DWT_Delay(0.0003f);					
					
						if(arm.motor_DM_data[1].DM_motor_measure->motor_enabled == 0)
								CAN_cmd_4310_2_enable();
						else if(arm.motor_DM_data[1].DM_motor_measure->motor_enabled == 1)
								CAN_cmd_4310_mit_2(arm.motor_DM_data[1].position_set,0.0f,120.0f,5.0f,0.0f);
						else
								CAN_cmd_4310_2_disable();
								DWT_Delay(0.0003f);	
						
						if(arm.motor_DM_data[2].DM_motor_measure->motor_enabled == 0)
								CAN_cmd_4310_3_enable();					
						else if(arm.motor_DM_data[2].DM_motor_measure->motor_enabled == 1)	
								CAN_cmd_4310_mit_3(arm.motor_DM_data[2].position_set,0.0f,70.0f,1.0f,0.0f);
						else
								CAN_cmd_4310_3_disable();	
								DWT_Delay(0.0003f);
				}
				// 5-6-2006���͵���ָ��
				CAN_cmd_2006(arm.motor_2006_data[0].give_current,arm.motor_2006_data[1].give_current);			
				osDelay(1);
		}	
}


void arm_control_init(arm_t *arm_control_init)
{	
    if (arm_control_init == NULL)
    {
        return;
    }		
		//��ʼ���������ָ��
		for(int i = 0; i < 3; i++)
		{
				arm_control_init->motor_DM_data[i].DM_motor_measure = return_4310_measure(i);
		}
		for(int i = 0; i < 2; i++)
		{
			arm_control_init->motor_2006_data[i].motor_measure = return_2006_measure(i);
		}
		arm_control_init->motor_3508_data.motor_measure = return_3508_measure();
		//��ʼ��PID
		PID_Init(&arm_control_init->motor_3508_speed_pid, M3508_SPEED_PID_MAX_OUT, M3508_SPEED_PID_MAX_IOUT,0.0f,M3508_SPEED_PID_KP, M3508_SPEED_PID_KI, M3508_SPEED_PID_KD,0.0f,0.0f,0.00106103295394596890512589175582f,0.0f,5,0x11);
		PID_Init(&arm_control_init->motor_3508_angle_pid, M3508_POSITION_PID_MAX_OUT, M3508_POSITION_PID_MAX_IOUT,0.0f,M3508_POSITION_PID_KP, M3508_POSITION_PID_KI, M3508_POSITION_PID_KD,0.0f,0.0f,0.0f,0.0f,4,0x01);		
		PID_Init(&arm_control_init->motor_2006_speed_pid[0], M2006_ROTATE_SPEED_PID_MAX_OUT, M2006_ROTATE_SPEED_PID_MAX_IOUT,0.0f,M2006_SPEED_PID_KP_0, M2006_SPEED_PID_KI, M2006_SPEED_PID_KD,0.0f,0.0f,0.00106103295394596890512589175582f,0.0f,5,0x11);
		PID_Init(&arm_control_init->motor_2006_speed_pid[1], M2006_ROTATE_SPEED_PID_MAX_OUT, M2006_ROTATE_SPEED_PID_MAX_IOUT,0.0f,M2006_SPEED_PID_KP_1, M2006_SPEED_PID_KI, M2006_SPEED_PID_KD,0.0f,0.0f,0.00106103295394596890512589175582f,0.0f,5,0x11);
		PID_Init(&arm_control_init->yaw_angle_pid,ARM_YAW_PID_MAX_OUT,ARM_YAW_PID_MAX_IOUT,0.0f,ARM_YAW_PID_KP, ARM_YAW_PID_KI, ARM_YAW_PID_KD,0.0f,0.0f,0.0f,0.0f,4,0x01);
		PID_Init(&arm_control_init->roll_angle_pid,ARM_ROLL_PID_MAX_OUT,ARM_ROLL_PID_MAX_IOUT,0.0f,ARM_ROLL_PID_KP, ARM_ROLL_PID_KI, ARM_ROLL_PID_KD,0.0f,0.0f,0.0f,0.0f,4,0x01);

		//��������Ҫ�ȷ�����Ϣ���з���֡
		CAN_cmd_4310_1_disable();
		osDelay(1);
		CAN_cmd_4310_2_disable();
		osDelay(1);	
		//���ݸ���
		arm_feedback_update(arm_control_init);
		//��ʼ������������
		arm_control_init->arm_mode = 0;
		arm_control_init->motor_3508_mode = 0;		
		arm_control_init->motor_3508_data.round_cnt = 0;
		arm_control_init->motor_3508_data.offset_ecd = arm_control_init->motor_3508_data.motor_measure->ecd;
		arm_control_init->motor_3508_data.angle = (arm_control_init->motor_3508_data.round_cnt * ECD_RANGE 
				+ arm_control_init->motor_3508_data.offset_ecd - arm_control_init->motor_3508_data.motor_measure->ecd) * MOTOR_ECD_TO_ANGLE_3508;
		arm_control_init->motor_3508_data.angle_set = arm_control_init->motor_3508_data.angle;		
		arm_control_init->motor_DM_data[0].position_set = arm_control_init->motor_DM_data[0].DM_motor_measure->motor_position;
		arm_control_init->motor_DM_data[1].position_set = arm_control_init->motor_DM_data[1].DM_motor_measure->motor_position;	
		arm_control_init->motor_DM_data[2].position_set = arm_control_init->motor_DM_data[2].DM_motor_measure->motor_position;	
		
		arm_control_init->motor_2006_mode = 0;
		for(int16_t i = 0; i < 2; i++)
		{
			arm_control_init->motor_2006_data[i].offset_ecd = arm_control_init->motor_2006_data[i].motor_measure->ecd;
			arm_control_init->motor_2006_data[i].angle_set = arm_control_init->motor_2006_data[i].angle;
			arm_control_init->motor_2006_data[i].speed_set = 0;
		}			
		//��ʼ������΢����TD
		TD_init(&arm_control_init->arm_1_TD,55.0f,2.0f,0.002f,arm_control_init->motor_3508_data.angle);
		TD_init(&arm_control_init->arm_2_TD,10.0f,2.0f,0.002f,arm_control_init->motor_DM_data[0].DM_motor_measure->motor_position);
		TD_init(&arm_control_init->arm_3_TD,12.0f,2.0f,0.002f,arm_control_init->motor_DM_data[1].DM_motor_measure->motor_position);
		TD_init(&arm_control_init->arm_4_TD,14.0f,2.0f,0.002f,arm_control_init->motor_DM_data[2].DM_motor_measure->motor_position);
		TD_init(&arm_control_init->arm_5_TD,18.0f,2.0f,0.002f,arm_control_init->roll_angle);
		TD_init(&arm_control_init->arm_6_TD,18.0f,2.0f,0.002f,arm_control_init->yaw_angle);		
}

void arm_set_mode(arm_t *arm_set_mode)
{	
    if (arm_set_mode == NULL)
    {
        return;
    }	
		
		//���µ����ϴ��ı�־λ
		arm_set_mode->arm_last_mode = arm_set_mode->arm_mode;	
		arm_set_mode->arm_mode = board_message.mode;		
		//���ݵ��̱�־λ����1-3508��5&6-2006��־λ
		if(arm_set_mode->arm_mode == 0)
		{
				arm_set_mode->motor_3508_mode = 0;
				arm_set_mode->motor_2006_mode = 0;
		}
		else if(arm_set_mode->arm_mode != 0 && arm_set_mode->arm_last_mode == 0)
		{
				arm_set_mode->motor_3508_mode = 1;
				arm_set_mode->motor_2006_mode = 0;
		}
		if(arm_set_mode->motor_3508_mode == 2 && arm_set_mode->motor_2006_mode == 0)
			arm_set_mode->motor_2006_mode = 1;
		//���ݳ�ʼ���������3508��־λ
		if(arm_set_mode->motor_3508_mode == 1)
		{
				if(reset_3508_switch == 1)
						reset_3508_count[0] ++;
				else
						reset_3508_count[0] = 0;
				if(fabs(arm_set_mode->motor_3508_data.speed) < 0.8f && Motor_3508_Timeout_Flag ==0)
						reset_3508_count[1] ++;
				else
						reset_3508_count[1] = 0;				
				if (reset_3508_count[0] > 15)
				{
						reset_3508_count[0] = 0;
						reset_3508_count[1] = 0;
						arm_set_mode->motor_3508_mode = 2;	
						arm_set_mode->motor_3508_data.round_cnt = 0;
						arm_set_mode->motor_3508_data.offset_ecd = arm_set_mode->motor_3508_data.motor_measure->ecd;
						arm_set_mode->motor_3508_data.angle = (arm_set_mode->motor_3508_data.round_cnt * ECD_RANGE 
								+ arm_set_mode->motor_3508_data.offset_ecd - arm_set_mode->motor_3508_data.motor_measure->ecd) * MOTOR_ECD_TO_ANGLE_3508;
						TD_set_x(&arm_set_mode->arm_1_TD, arm_set_mode->motor_3508_data.angle);							
				}
				else if(reset_3508_count[1] > 150)
				{
						reset_3508_count[0] = 0;
						reset_3508_count[1] = 0;
						arm_set_mode->motor_3508_mode = 2;	
						arm_set_mode->motor_3508_data.round_cnt = 0;
						arm_set_mode->motor_3508_data.offset_ecd = arm_set_mode->motor_3508_data.motor_measure->ecd;
						arm_set_mode->motor_3508_data.angle = (arm_set_mode->motor_3508_data.round_cnt * ECD_RANGE 
								+ arm_set_mode->motor_3508_data.offset_ecd - arm_set_mode->motor_3508_data.motor_measure->ecd) * MOTOR_ECD_TO_ANGLE_3508;
						TD_set_x(&arm_set_mode->arm_1_TD, arm_set_mode->motor_3508_data.angle);					
				}
		}
		//���ݳ�ʼ���������2006��־λ
		if(arm_set_mode->motor_2006_mode == 1)
		{   
			if(arm_set_mode->motor_2006_last_mode != 1)
			{
				Set_KP(&arm_set_mode->motor_2006_speed_pid[0],M2006_SPEED_PID_KP_0);
				Set_KP(&arm_set_mode->motor_2006_speed_pid[1],M2006_SPEED_PID_KP_0);
			}
				if(fabs(arm_set_mode->motor_2006_data[0].speed) < 0.2f)
						reset_2006_count[0] ++;
				else
						reset_2006_count[0] = 0;
				if(fabs(arm_set_mode->motor_2006_data[1].speed) < 0.2f)
						reset_2006_count[1] ++;
				else
						reset_2006_count[1] = 0;										
				if (reset_2006_count[0] > 150 && reset_2006_count[1] > 150)
				{
						reset_2006_count[0] = 0;
						reset_2006_count[1] = 0;
						arm_set_mode->motor_2006_mode = 2;
						arm_set_mode->motor_2006_data[0].round_cnt = 0;
						arm_set_mode->motor_2006_data[1].round_cnt = 0;
						arm_set_mode->motor_2006_data[0].offset_ecd = arm_set_mode->motor_2006_data[0].motor_measure->ecd;
						arm_set_mode->motor_2006_data[1].offset_ecd = arm_set_mode->motor_2006_data[1].motor_measure->ecd;
						TD_set_x(&arm_set_mode->arm_5_TD,arm_set_mode->roll_angle);
						TD_set_x(&arm_set_mode->arm_6_TD,arm_set_mode->yaw_angle);
						
				}
		}
		//��ʼ�����������ɿ�״̬
		//����
//		if(arm_set_mode->motor_2006_mode == 2)
//		{
//			if(arm_set_mode->motor_2006_last_mode != 2)
//				{
//							arm_set_mode->motor_2006_speed_pid[0].Kp = 2000;
//							arm_set_mode->motor_2006_speed_pid[1].Kp = 2000;
//				}
//			arm_set_mode->motor_2006_mode = 3;
//		}
		
		if(arm_set_mode->motor_3508_mode == 2 && arm_set_mode->motor_2006_mode == 2 
			&& fabs(arm_set_mode->motor_3508_data.angle - 20.0f) < 0.5f )
		{
			if(arm_set_mode->motor_2006_last_mode != 2)
			{
				Set_KP(&arm_set_mode->motor_2006_speed_pid[0],M2006_SPEED_PID_KP_1);
				Set_KP(&arm_set_mode->motor_2006_speed_pid[1],M2006_SPEED_PID_KP_1);
				mode2_cnt = DWT_GetTimeline_ms();
			}
			if(DWT_GetTimeline_ms() - mode2_cnt > 300.0f)
			{
				arm_set_mode->motor_3508_mode = 3;
				arm_set_mode->motor_2006_mode = 3;
			}
		}	
		arm_set_mode->motor_2006_last_mode = arm_set_mode->motor_2006_mode;
}	


void arm_feedback_update(arm_t *arm_feedback)
{
		if(arm_feedback == NULL)
		{
				return;
		}
		//����3508��Can�źŷ�����־λ
		Motor_3508_Timeout_Count ++;
		if(Motor_3508_Timeout_Count >= 30)
		{
				Motor_3508_Timeout_Flag = 1;
		}
		
		//������λ��������
		if((HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0) == GPIO_PIN_RESET))
		{
				reset_3508_switch = 1 ;
		}
		else
		{
				reset_3508_switch = 0;
		}
		
		//����1-3508����
		arm_feedback->motor_3508_data.speed = arm_feedback->motor_3508_data.motor_measure->speed_rpm * RPM_TO_RAD_S * REDUCTION_RATIO_3508;
		arm_feedback->motor_3508_data.angle = (arm_feedback->motor_3508_data.round_cnt * ECD_RANGE 
				+ arm_feedback->motor_3508_data.motor_measure->ecd - arm_feedback->motor_3508_data.offset_ecd) * MOTOR_ECD_TO_ANGLE_3508;		
		//����5&6-2006����
		for(int16_t i = 0; i < 2; i++)
		{
			arm_feedback->motor_2006_data[i].speed = arm_feedback->motor_2006_data[i].motor_measure->speed_rpm * RPM_TO_RAD_S * REDUCTION_RATIO_2006;
			arm_feedback->motor_2006_data[i].angle = (arm_feedback->motor_2006_data[i].round_cnt * ECD_RANGE 
					+ arm_feedback->motor_2006_data[i].motor_measure->ecd - arm_feedback->motor_2006_data[i].offset_ecd) * MOTOR_ECD_TO_ANGLE_2006;	
		}
		arm.yaw_angle = rad_format((arm.motor_2006_data[0].angle + arm.motor_2006_data[1].angle)/(2 * YAW_TO_2006));
	  arm.roll_angle = (arm.motor_2006_data[0].angle - arm.motor_2006_data[1].angle)/(2 * ROLL_TO_2006);//ת����ĩ������Ƕ�

		//����ģʽ����TD
		if(arm_feedback->arm_mode == 1 && board_message.suker_mode == 1)
		{
				TD_set_r(&arm_feedback->arm_1_TD,45.0f);			
				TD_set_r(&arm_feedback->arm_2_TD,9.0f);
				TD_set_r(&arm_feedback->arm_3_TD,10.0f);
				TD_set_r(&arm_feedback->arm_4_TD,10.0f);
				TD_set_r(&arm_feedback->arm_5_TD,14.0f);
				TD_set_r(&arm_feedback->arm_6_TD,14.0f);					
		}
		else
		{
				TD_set_r(&arm_feedback->arm_1_TD,55.0f);			
				TD_set_r(&arm_feedback->arm_2_TD,10.0f);
				TD_set_r(&arm_feedback->arm_3_TD,12.0f);
				TD_set_r(&arm_feedback->arm_4_TD,14.0f);
				TD_set_r(&arm_feedback->arm_5_TD,18.0f);
				TD_set_r(&arm_feedback->arm_6_TD,18.0f);				
		}	
}


void arm_set_control(arm_t *arm_set_control)
{
		if(arm_set_control == NULL)
		{
				return;
		}		
		//1 
		if(arm_set_control->motor_3508_mode == 0)
		{
				TD_set_x(&arm_set_control->arm_1_TD,arm_set_control->motor_3508_data.angle);
		}
		else if(arm_set_control->motor_3508_mode == 1)
		{			
				TD_set_x(&arm_set_control->arm_1_TD,arm_set_control->motor_3508_data.angle);
				arm_set_control->motor_3508_data.speed_set = -20.0f;
		}
		else if(arm_set_control->motor_3508_mode == 2)
		{
				TD_calc(&arm_set_control->arm_1_TD, 20.0f);
				arm_set_control->motor_3508_data.angle_set = arm_set_control->arm_1_TD.x;
		}
		else
		{
				TD_calc(&arm_set_control->arm_1_TD, board_message.target_position[0]);
				arm_set_control->motor_3508_data.angle_set = arm_set_control->arm_1_TD.x;
		}			
		//2-4
		if(arm_set_control->arm_mode == 0)
		{
				TD_set_x(&arm_set_control->arm_2_TD,arm_set_control->motor_DM_data[0].DM_motor_measure->motor_position);
				TD_set_x(&arm_set_control->arm_3_TD,arm_set_control->motor_DM_data[1].DM_motor_measure->motor_position);
				TD_set_x(&arm_set_control->arm_4_TD,arm_set_control->motor_DM_data[2].DM_motor_measure->motor_position);
		}
		else if(arm_set_control->motor_3508_mode != 3)
		{
				TD_calc(&arm_set_control->arm_2_TD,0.0f);			
				TD_calc(&arm_set_control->arm_3_TD,0.0f);		
				TD_calc(&arm_set_control->arm_4_TD,0.0f);
				arm_set_control->motor_DM_data[0].position_set = (fp32)(arm_set_control->arm_2_TD.x);					
				arm_set_control->motor_DM_data[1].position_set = (fp32)(arm_set_control->arm_3_TD.x);					
				arm_set_control->motor_DM_data[2].position_set = (fp32)(arm_set_control->arm_4_TD.x);	
		}
		else
		{				
				TD_calc(&arm_set_control->arm_2_TD,board_message.target_position[1]);			
				TD_calc(&arm_set_control->arm_3_TD,board_message.target_position[2]);		
				TD_calc(&arm_set_control->arm_4_TD,board_message.target_position[3]);
				arm_set_control->motor_DM_data[0].position_set = (fp32)(arm_set_control->arm_2_TD.x);					
				arm_set_control->motor_DM_data[1].position_set = (fp32)(arm_set_control->arm_3_TD.x);					
				arm_set_control->motor_DM_data[2].position_set = (fp32)(arm_set_control->arm_4_TD.x);			
		}
		//5-6		
		if(arm_set_control->motor_2006_mode == 0)
		{
				TD_set_x(&arm_set_control->arm_5_TD,arm_set_control->roll_angle);
				TD_set_x(&arm_set_control->arm_6_TD,arm_set_control->yaw_angle);				
		}
		else if(arm_set_control->motor_2006_mode == 1)
		{			
				TD_set_x(&arm_set_control->arm_5_TD,arm_set_control->roll_angle);
				TD_set_x(&arm_set_control->arm_6_TD,arm_set_control->yaw_angle);		
				arm_set_control->motor_2006_data[0].speed_set =  -2.0f;
				arm_set_control->motor_2006_data[1].speed_set =   2.0f;
		}
		else if(arm_set_control->motor_2006_mode == 2)
		{
				TD_set_x(&arm_set_control->arm_5_TD,arm_set_control->roll_angle);
				TD_set_x(&arm_set_control->arm_6_TD,arm_set_control->yaw_angle);					
		}
		else 
		{
				//׶���ֵĴ���
				TD_calc(&arm_set_control->arm_5_TD, board_message.target_position[4]);
				TD_calc(&arm_set_control->arm_6_TD, board_message.target_position[5]);
				arm_set_control->roll_angle_set = arm_set_control->arm_5_TD.x;
				arm_set_control->yaw_angle_set = arm_set_control->arm_6_TD.x;
		}	
}

void arm_control_loop(arm_t *arm_control_loop)
{
		//1-3508
		if(arm_control_loop->motor_3508_mode == 0)
		{
				arm_control_loop->motor_3508_data.give_current = 0;
		}
		else if(arm_control_loop->motor_3508_mode == 1)
		{
				PID_Calculate(&arm_control_loop->motor_3508_speed_pid, arm_control_loop->motor_3508_data.speed, arm_control_loop->motor_3508_data.speed_set);
				arm_control_loop->motor_3508_data.give_current = int16_constrain((int16_t)(arm_control_loop->motor_3508_speed_pid.Output),-5000,5000);
		}	
		else if(arm_control_loop->motor_3508_mode == 2 || arm_control_loop->motor_3508_mode == 3)
		{
				PID_Calculate(&arm_control_loop->motor_3508_angle_pid, arm_control_loop->motor_3508_data.angle, arm_control_loop->motor_3508_data.angle_set);
				arm_control_loop->motor_3508_data.speed_set = arm_control_loop->motor_3508_angle_pid.Output;
				PID_Calculate(&arm_control_loop->motor_3508_speed_pid, arm_control_loop->motor_3508_data.speed, arm_control_loop->motor_3508_data.speed_set);
				arm_control_loop->motor_3508_data.give_current = (int16_t)(arm_control_loop->motor_3508_speed_pid.Output);
		}			
		//5&6-2006
		if(arm_control_loop->motor_2006_mode == 0)
		{
				arm_control_loop->motor_2006_data[0].give_current = 0;
				arm_control_loop->motor_2006_data[1].give_current = 0;
		}
		else if(arm_control_loop->motor_2006_mode == 1)
		{
				PID_Calculate(&arm_control_loop->motor_2006_speed_pid[0], arm_control_loop->motor_2006_data[0].speed, arm_control_loop->motor_2006_data[0].speed_set);
//				arm_control_loop->motor_2006_data[0].give_current = int16_constrain((int16_t)(arm_control_loop->motor_2006_speed_pid[0].Output),-6000,6000);
				arm_control_loop->motor_2006_data[0].give_current = (int16_t)(arm_control_loop->motor_2006_speed_pid[0].Output);
				PID_Calculate(&arm_control_loop->motor_2006_speed_pid[1], arm_control_loop->motor_2006_data[1].speed, arm_control_loop->motor_2006_data[1].speed_set);
				arm_control_loop->motor_2006_data[1].give_current = (int16_t)(arm_control_loop->motor_2006_speed_pid[1].Output);
//				arm_control_loop->motor_2006_data[1].give_current = int16_constrain((int16_t)(arm_control_loop->motor_2006_speed_pid[0].Output),-6000,6000);

		}
		else if(arm_control_loop->motor_2006_mode == 2)
		{
				arm_control_loop->motor_2006_data[0].give_current = 0;
				arm_control_loop->motor_2006_data[1].give_current = 0;			
		}
		else if(arm_control_loop->motor_2006_mode == 3)
		{
				int16_t current_2006_tem[2];
				PID_Calculate_Angle(&arm_control_loop->yaw_angle_pid,arm_control_loop->yaw_angle,arm_control_loop->yaw_angle_set);
				PID_Calculate(&arm_control_loop->roll_angle_pid,arm_control_loop->roll_angle,arm_control_loop->roll_angle_set);
				
				arm_control_loop->motor_2006_data[0].speed_set = arm_control_loop->roll_angle_pid.Output + arm_control_loop->yaw_angle_pid.Output;
				PID_Calculate(&arm_control_loop->motor_2006_speed_pid[0], arm_control_loop->motor_2006_data[0].speed, arm_control_loop->motor_2006_data[0].speed_set);
		
				arm_control_loop->motor_2006_data[1].speed_set = - arm_control_loop->roll_angle_pid.Output + arm_control_loop->yaw_angle_pid.Output;;
				PID_Calculate(&arm_control_loop->motor_2006_speed_pid[1], arm_control_loop->motor_2006_data[1].speed, arm_control_loop->motor_2006_data[1].speed_set);
			
			if(board_message.suker_mode == 0)
				{				
					current_2006_tem[0] = (int16_t)(arm_control_loop->motor_2006_speed_pid[0].Output);
					current_2006_tem[1] = (int16_t)(arm_control_loop->motor_2006_speed_pid[1].Output);
				}
			else if(board_message.suker_mode == 1)
				{
					current_2006_tem[0] = (int16_t)(arm_control_loop->motor_2006_speed_pid[0].Output + g_2006 * cosf(arm_control_loop->yaw_angle)*cosf(arm_control_loop->roll_angle - 1.92f));
					current_2006_tem[1] = (int16_t)(arm_control_loop->motor_2006_speed_pid[1].Output - g_2006 * cosf(arm_control_loop->yaw_angle)*cosf(arm_control_loop->roll_angle - 1.92f));  									
				}
					arm_control_loop->motor_2006_data[0].give_current = int16_constrain(current_2006_tem[0],-10000,10000);
					arm_control_loop->motor_2006_data[1].give_current = int16_constrain(current_2006_tem[1],-10000,10000);  

		}						
}


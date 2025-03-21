/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务

  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "referee_usart_task.h"
#include "A_communicate_task.h"
#include "chassis_behaviour.h"
#include "arm_control_task.h"
#include "remote_control.h"
#include "chassis_task.h"
#include "CAN_receive.h"
#include "controller.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "gpio.h"
#include "tim.h"


#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

static void chassis_init(all_key_t *chassis_key_init, chassis_t *chassis_init);
static void chassis_set_mode(chassis_t *chassis_mode);
static void chassis_key_check(all_key_t *chassis_key_check);
static void chassis_feedback_update(chassis_t *chassis_update);
static void chassis_set_contorl(chassis_t *chassis_control);
static void chassis_control_loop(chassis_t *chassis_control_loop);


chassis_t chassis;
bool_t  clamp_flag;
uint8_t clamp_mode;
int16_t clamp_reset_count;		
uint8_t picture_mode;
int16_t picture_reset_count;	
uint8_t lift_switch;
uint8_t lift_mode;
int16_t lift_reset_count[2];
uint8_t picture_rotate_flag;
uint32_t G_reset_flag;
int arm_restart_flag;

void chassis_task(void const *pvParameters)
{
    vTaskDelay(CHASSIS_TASK_INIT_TIME); 
    chassis_init(&all_key, &chassis);
	
		uint32_t system_clock = osKernelSysTick();
    while (1)
    {
				chassis.dt = DWT_GetDeltaT(&chassis.DWT_Count);
				
				chassis_set_mode(&chassis);			
				chassis_key_check(&all_key);
				chassis_feedback_update(&chassis);
				chassis_set_contorl(&chassis);
				chassis_control_loop(&chassis);

				//can1
				//空-2006(图传)-2006(夹矿)-3508(抬升)
				//can2
				//底盘4个3508
				if(chassis.chassis_mode == ZERO_POWER_MODE)
				{
						CAN_chassis_can1(0, 0, 0, 0);
						CAN_chassis_can2(0, 0, 0, 0);
				}
				else
				{
//					CAN_chassis_can1(0, 																		 chassis.motor_picture.give_current, 
//													 0, 			 chassis.motor_lift.give_current);
						CAN_chassis_can1(0, 																		 chassis.motor_picture.give_current, 
														chassis.motor_clamp.give_current, 			 chassis.motor_lift.give_current);
//						CAN_chassis_can1(0, 			 chassis.motor_picture.give_current, 
//														 0, 			 0);
						CAN_chassis_can2(chassis.motor_chassis[0].give_current,  chassis.motor_chassis[1].give_current, 
														 chassis.motor_chassis[2].give_current,  chassis.motor_chassis[3].give_current);		
				}
				osDelay(2);
    }
}

static void chassis_init(all_key_t *chassis_key_init, chassis_t *chassis_init)
{
    if (chassis_init == NULL)
    {
        return;
    }
		
		lift_mode = 0;
		lift_reset_count[0] = 0;			
		lift_reset_count[1] = 0;		
		
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
		
		//获取遥控器指针		
    chassis_init->chassis_RC = get_remote_control_point();
		//获取电机指针
    for (uint8_t i = 0; i < 4; i++)
    {
			chassis_init->motor_chassis[i].motor_measure = get_chassis_motor_point(i);
    }		
		chassis_init->motor_picture.motor_measure = get_picture_motor_point();
		chassis_init->motor_clamp.motor_measure = get_clamp_motor_point();
		chassis_init->motor_lift.motor_measure = get_lift_motor_point();
		
		//初始化电机pid
		//底盘
    for (uint8_t i = 0; i < 4; i++)
    {
				PID_Init(&chassis_init->chassis_motor_speed_pid[i], CHASSIS_M3508_SPEED_PID_MAX_OUT, CHASSIS_M3508_SPEED_PID_MAX_IOUT,0.0f,
								CHASSIS_M3508_SPEED_PID_KP, CHASSIS_M3508_SPEED_PID_KI, CHASSIS_M3508_SPEED_PID_KD,0.0f,0.0f,0.000795774715459476f,0.0f,5,0x11);
    }
		//夹矿
		PID_Init(&chassis_init->clamp_motor_position_pid, CLAMP_POSITION_PID_MAX_OUT, CLAMP_POSITION_PID_MAX_KD,0.0f,
						CLAMP_POSITION_PID_KP, CLAMP_POSITION_PID_KI, CLAMP_POSITION_PID_KD,0.0f,0.0f,0.00106103295394596890512589175582f,0.0f,4,0x11);
		PID_Init(&chassis_init->clamp_motor_speed_pid, CLAMP_SPEED_PID_MAX_OUT, CLAMP_SPEED_PID_MAX_KD,0.0f,
						CLAMP_SPEED_PID_KP, CLAMP_SPEED_PID_KI, CLAMP_SPEED_PID_KD,0.0f,0.0f,0.00106103295394596890512589175582f,0.0f,5,0x11);		
		//图传抬升
		PID_Init(&chassis_init->picture_motor_position_pid, PICTURE_POSITION_PID_MAX_OUT, PICTURE_POSITION_PID_MAX_IOUT,0.0f,
						PICTURE_POSITION_PID_KP, PICTURE_POSITION_PID_KI, PICTURE_POSITION_PID_KD,0.0f,0.0f,0.00106103295394596890512589175582f,0.0f,4,0x11);
		PID_Init(&chassis_init->picture_motor_speed_pid, PICTURE_SPEED_PID_MAX_OUT, PICTURE_SPEED_PID_MAX_IOUT,0.0f,
						PICTURE_SPEED_PID_KP, PICTURE_SPEED_PID_KI, PICTURE_SPEED_PID_KD,0.0f,0.0f,0.00106103295394596890512589175582f,0.0f,5,0x11);
		//抬升
		PID_Init(&chassis_init->lift_motor_position_pid, LIFT_POSITION_PID_MAX_OUT, LIFT_POSITION_PID_MAX_IOUT,0.0f,
						LIFT_POSITION_PID_KP, LIFT_POSITION_PID_KI, LIFT_POSITION_PID_KD,0.0f,0.0f,0.00106103295394596890512589175582f,0.0f,4,0x11);
		PID_Init(&chassis_init->lift_motor_speed_pid, LIFT_SPEED_SPEED_PID_MAX_OUT, LIFT_SPEED_SPEED_PID_MAX_IOUT,0.0f,
						LIFT_SPEED_PID_KP, LIFT_SPEED_PID_KI, LIFT_SPEED_PID_KD,0.0f,0.0f,0.00106103295394596890512589175582f,0.0f,5,0x11);	
		//底盘跟随云台
		PID_Init(&chassis_init->chassis_yaw_pid, CHASSIS_YAW_PID_MAX_OUT, CHASSIA_YAW_PID_MAX_IOUT,0.0f,
						CHASSIS_YAW_PID_KP, CHASSIS_YAW_PID_KI, CHASSIS_YAW_PID_KD,0.0f,0.0f,0.00106103295394596890512589175582f,0.0f,4,0x11);		
		//底盘速度设定值滤波
    first_order_filter_init(&chassis_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
		//初始化按键
		key_init(&chassis_key_init->rotate_key_G,G);
		key_init(&chassis_key_init->rotate_key_V,V);
		key_init(&chassis_key_init->picture_rotate_key,B);
		key_init(&chassis_key_init->arm_restart_key1,C);
		key_init(&chassis_key_init->arm_restart_key2,SHIFT);
		//更新一次数据
		chassis_feedback_update(chassis_init);	
		chassis_init->motor_clamp.offset_ecd = chassis_init->motor_clamp.motor_measure->ecd;
		chassis_init->motor_picture.offset_ecd = chassis_init->motor_picture.motor_measure->ecd;
		chassis_init->motor_lift.offset_ecd = chassis_init->motor_lift.motor_measure->ecd;				
		chassis_init->yaw_angle_set = chassis_init->yaw_angle;		
		//初始化TD跟踪微分器
		TD_init(&chassis_init->lift_3508_TD, 95.0f, 2.0f, 0.002f, chassis_init->motor_lift.position);
		TD_init(&chassis_init->chassis_yaw_TD, 12.0f, 2.0f, 0.002f, chassis_init->yaw_angle);
}


static void chassis_set_mode(chassis_t *chassis_set_mode)
{
		if (chassis_set_mode == NULL)
		{
				return;
		}
		
		chassis_set_mode->last_chassis_mode = chassis_set_mode->chassis_mode;
		
		//遥控器右上
		if(switch_is_mid(chassis_set_mode->chassis_RC->rc.s[RC_S_RIGHT]))
		{
				chassis_set_mode->chassis_mode = ONE_KEY_MODE;
		}
		else if(switch_is_up(chassis_set_mode->chassis_RC->rc.s[RC_S_RIGHT]))
		{
				chassis_set_mode->chassis_mode = SELF_CONTROL_MODE;
		}
		else
		{
				chassis_set_mode->chassis_mode = ZERO_POWER_MODE;
		}	

		//遥控器右上模式选择标志位
		if(chassis_set_mode->chassis_mode == ZERO_POWER_MODE)
		{
				clamp_mode = 0;
				lift_mode = 0;
				picture_mode = 0;
		}
		if(chassis_set_mode->last_chassis_mode == ZERO_POWER_MODE && chassis_set_mode->chassis_mode != ZERO_POWER_MODE)
		{
				clamp_mode = 1;
				lift_mode = 1;
				picture_mode = 1;
		}		
		//遥控器左上
		if(switch_is_down(chassis_set_mode->chassis_RC->rc.s[RC_S_LEFT]))
		{			
				clamp_flag = 0;
		}
		else
		{				
				clamp_flag = 1;
		}		
	
	
		//夹矿
		if(clamp_mode == 1)
		{
				if(fabs(chassis_set_mode->motor_clamp.speed) < 1.0f)
						clamp_reset_count++;
				else
						clamp_reset_count = 0;
		}
		if(clamp_mode == 1 && clamp_reset_count >= 150)
		{
				chassis_set_mode->motor_clamp.round_cnt = 0;
				chassis_set_mode->motor_clamp.offset_ecd = chassis_set_mode->motor_clamp.motor_measure->ecd;
				clamp_mode = 2;
				clamp_reset_count = 0;
		}	
		//图传抬升
		if(picture_mode == 1)
		{
				if(fabs(chassis_set_mode->motor_picture.speed) < 1.0f)
						picture_reset_count++;
				else
						picture_reset_count = 0;
		}
		if(picture_mode == 1 && picture_reset_count >= 150)
		{
				chassis_set_mode->motor_picture.round_cnt = 0;
				chassis_set_mode->motor_picture.offset_ecd = chassis_set_mode->motor_picture.motor_measure->ecd;
				picture_mode = 2;
				picture_reset_count = 0;
		}		
		//抬升
		if(lift_mode == 1)
		{	
				if(lift_switch == 1)
						lift_reset_count[0] ++;
				else
						lift_reset_count[0] = 0;
				if(fabs(chassis_set_mode->motor_lift.speed) < 0.8f)
						lift_reset_count[1] ++;
				else
						lift_reset_count[1] = 0;					
				if(lift_reset_count[0] >= 20)
				{
						lift_mode = 2;
						lift_reset_count[0] = 0;	
						lift_reset_count[1] = 0;						
						chassis_set_mode->motor_lift.round_cnt = 0;
						chassis_set_mode->motor_lift.offset_ecd = chassis_set_mode->motor_lift.motor_measure->ecd;
						chassis_set_mode->motor_lift.position = (chassis_set_mode->motor_lift.round_cnt * ECD_RANGE + chassis_set_mode->motor_lift.motor_measure->ecd - chassis_set_mode->motor_lift.offset_ecd) * MOTOR_ECD_TO_ANGLE_3508;
						TD_set_x(&chassis_set_mode->lift_3508_TD, chassis_set_mode->motor_lift.position);

				}		
				else if(lift_reset_count[1] > 150)
				{
						lift_mode = 2;
						lift_reset_count[0] = 0;	
						lift_reset_count[1] = 0;						
						chassis_set_mode->motor_lift.round_cnt = 0;
						chassis_set_mode->motor_lift.offset_ecd = chassis_set_mode->motor_lift.motor_measure->ecd;
						chassis_set_mode->motor_lift.position = (chassis_set_mode->motor_lift.round_cnt * ECD_RANGE + chassis_set_mode->motor_lift.motor_measure->ecd - chassis_set_mode->motor_lift.offset_ecd) * MOTOR_ECD_TO_ANGLE_3508;
						TD_set_x(&chassis_set_mode->lift_3508_TD, chassis_set_mode->motor_lift.position);				
				}
		}	
}


void chassis_key_check(all_key_t *chassis_key_check)
{
		key_itself_press_num(&(chassis_key_check->rotate_key_G),2);
		key_itself_press_num(&(chassis_key_check->rotate_key_V),2);
		key_itself_press_num(&(chassis_key_check->picture_rotate_key),2);
		key_itself_press_num(&(chassis_key_check->arm_restart_key1),2);
		key_itself_press_num(&(chassis_key_check->arm_restart_key2),2);

}


static void chassis_feedback_update(chassis_t *chassis_update)
{
    if (chassis_update == NULL)
    {
        return;
    }
		//更新抬升3508标志位
		if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET))
		{
				lift_switch = 1;
		}
		else
		{
				lift_switch = 0;
		}
		//底盘3508电机
    for (uint8_t i = 0; i < 4; i++)
    {
				chassis_update->motor_chassis[i].speed = chassis_update->motor_chassis[i].motor_measure->speed_rpm * CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;
    }
		//夹矿
		chassis_update->motor_clamp.position = (chassis_update->motor_clamp.round_cnt * ECD_RANGE + chassis_update->motor_clamp.motor_measure->ecd - chassis_update->motor_clamp.offset_ecd) * MOTOR_ECD_TO_ANGLE_2006;
    chassis_update->motor_clamp.speed = chassis_update->motor_clamp.motor_measure->speed_rpm * RPM_TO_RAD_S * REDUCTION_RATIO_2006;		
		//图传抬升
		chassis_update->motor_picture.position = (chassis_update->motor_picture.round_cnt * ECD_RANGE + chassis_update->motor_picture.motor_measure->ecd - chassis_update->motor_picture.offset_ecd) * MOTOR_ECD_TO_ANGLE_2006;
    chassis_update->motor_picture.speed = chassis_update->motor_picture.motor_measure->speed_rpm * RPM_TO_RAD_S * REDUCTION_RATIO_2006;
		//抬升
		chassis_update->motor_lift.position = (chassis_update->motor_lift.round_cnt * ECD_RANGE + chassis_update->motor_lift.motor_measure->ecd - chassis_update->motor_lift.offset_ecd) * MOTOR_ECD_TO_ANGLE_3508;		
		chassis_update->motor_lift.speed = chassis_update->motor_lift.motor_measure->speed_rpm * RPM_TO_RAD_S * REDUCTION_RATIO_3508;				
		//更新底盘imu角度
		chassis_update->yaw_angle = imu_rx_data.rx_data.yaw_angle;
		//更新底盘速度反解算
		chassis_update->vx = -(-chassis_update->motor_chassis[0].speed + chassis_update->motor_chassis[1].speed + chassis_update->motor_chassis[2].speed - chassis_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_update->vy =  (-chassis_update->motor_chassis[0].speed - chassis_update->motor_chassis[1].speed + chassis_update->motor_chassis[2].speed + chassis_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_update->wz =  (-chassis_update->motor_chassis[0].speed - chassis_update->motor_chassis[1].speed - chassis_update->motor_chassis[2].speed - chassis_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
}


void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, fp32 *vz_set, chassis_t *chassis_move_rc_to_vector)
{
		if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
		{
				return;
		}

		int16_t vx_channel, vy_channel, vz_channel, vz_channel_mouse;
		fp32 vx_set_channel, vy_set_channel, vz_set_channel;
		rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL],  vx_channel, CHASSIS_RC_DEADLINE);
		rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL],  vy_channel, CHASSIS_RC_DEADLINE);
		rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL], vz_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->mouse.x, vz_channel_mouse, MOUSE_DEADBAND);//鼠标Y轴变化		
		vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
		vy_set_channel = vy_channel * CHASSIS_VY_RC_SEN;
		//正常是可以使用鼠标的，抓矿模式就把鼠标噶了
		if(picture_rotate_flag == 0)
		{				
				vz_set_channel = vz_channel * CHASSIS_WZ_RC_SEN + vz_channel_mouse * CHASSIS_WZ_MOUSE_SEN;
		}
		else
		{
				vz_set_channel = vz_channel * CHASSIS_WZ_RC_SEN;
		}
		
		//跑路模式
		if(picture_rotate_flag == 0)
	 {
				if(chassis_move_rc_to_vector->chassis_RC->key.SHIFT)
				{		 
						if (chassis_move_rc_to_vector->chassis_RC->key.W)
						{
								vx_set_channel = -SHIFT_NORMAL_MAX_CHASSIS_SPEED_X;
						}
						else if (chassis_move_rc_to_vector->chassis_RC->key.S)
						{
								vx_set_channel =  SHIFT_NORMAL_MAX_CHASSIS_SPEED_X;
						}
						if (chassis_move_rc_to_vector->chassis_RC->key.A)
						{
								vy_set_channel =  SHIFT_NORMAL_MAX_CHASSIS_SPEED_Y;
						}
						else if (chassis_move_rc_to_vector->chassis_RC->key.D)
						{
								vy_set_channel = -SHIFT_NORMAL_MAX_CHASSIS_SPEED_Y;
						}
				}
				else
				{
						if (chassis_move_rc_to_vector->chassis_RC->key.W)
						{
								vx_set_channel = -NORMAL_MAX_CHASSIS_SPEED_X;
						}
						else if (chassis_move_rc_to_vector->chassis_RC->key.S)
						{
								vx_set_channel =  NORMAL_MAX_CHASSIS_SPEED_X;
						}
						if (chassis_move_rc_to_vector->chassis_RC->key.A)
						{
								vy_set_channel =  NORMAL_MAX_CHASSIS_SPEED_Y;
						}
						else if (chassis_move_rc_to_vector->chassis_RC->key.D)
						{
								vy_set_channel = -NORMAL_MAX_CHASSIS_SPEED_Y;
						}				
				}
				//Q和E用来抓矿和兑矿时微调yaw轴角度
				if (chassis_move_rc_to_vector->chassis_RC->key.Q && all_key.adjust_key.itself.flag == 0)
				{
						vz_set_channel += 0.0035f;
				}
				else if (chassis_move_rc_to_vector->chassis_RC->key.E && all_key.adjust_key.itself.flag == 0)
				{
						vz_set_channel -= 0.0035f;
				}							
		}
	 //爬爬模式
		else
		{
					if(chassis_move_rc_to_vector->chassis_mode == SELF_CONTROL_MODE)
					{
							if (chassis_move_rc_to_vector->chassis_RC->key.A)
							{	
									vy_set_channel = SELF_CONTROL_SPEED_Y;
							}
							else if (chassis_move_rc_to_vector->chassis_RC->key.D)
							{
									vy_set_channel =  -SELF_CONTROL_SPEED_Y;
							}
							if (chassis_move_rc_to_vector->chassis_RC->key.S)
							{
									vx_set_channel =  SELF_CONTROL_SPEED_X;
							}
							else if (chassis_move_rc_to_vector->chassis_RC->key.W)
							{
									vx_set_channel = -SELF_CONTROL_SPEED_X;
							}							
					}
					else
					{
							if(chassis_move_rc_to_vector->chassis_RC->key.SHIFT)
							{
									if (chassis_move_rc_to_vector->chassis_RC->key.A)
									{	
											vy_set_channel = SHIFT_SLOW_CHASSIS_SPEED_Y;
									}
									else if (chassis_move_rc_to_vector->chassis_RC->key.D)
									{
											vy_set_channel =  -SHIFT_SLOW_CHASSIS_SPEED_Y;
									}
									if (chassis_move_rc_to_vector->chassis_RC->key.S)
									{
											vx_set_channel =  SHIFT_SLOW_CHASSIS_SPEED_X;
									}
									else if (chassis_move_rc_to_vector->chassis_RC->key.W)
									{
											vx_set_channel = -SHIFT_SLOW_CHASSIS_SPEED_X;
									}							
							}
							else
							{
									if (chassis_move_rc_to_vector->chassis_RC->key.A)
									{	
											vy_set_channel = SLOW_CHASSIS_SPEED_Y;
									}
									else if (chassis_move_rc_to_vector->chassis_RC->key.D)
									{
											vy_set_channel =  -SLOW_CHASSIS_SPEED_Y;
									}
									if (chassis_move_rc_to_vector->chassis_RC->key.S)
									{
											vx_set_channel =  SLOW_CHASSIS_SPEED_X;
									}
									else if (chassis_move_rc_to_vector->chassis_RC->key.W)
									{
											vx_set_channel = -SLOW_CHASSIS_SPEED_X;
									}					
							}					
					}

					//Q和E用来抓矿和兑矿时微调yaw轴角度
					if (chassis_move_rc_to_vector->chassis_RC->key.Q && all_key.adjust_key.itself.flag == 0)
					{
							vz_set_channel += 0.0015f;
					}
					else if (chassis_move_rc_to_vector->chassis_RC->key.E && all_key.adjust_key.itself.flag == 0)
					{
							vz_set_channel -= 0.0015f;
					}										
		}	

				
		first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
		first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);//这里过两个低通滤波器
		if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
		{
				chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
		}

		if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
		{
				chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
		}
		//用CTRL+W和S微调一键点位时不会移动
		if(picture_rotate_flag == 0)
		{
			if (all_key.adjust_key.itself.flag == 1)
			{
					chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
			}
		}
		else
		{
			if (all_key.adjust_key.itself.flag == 1)
			{
					chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
			}
		}
		*vx_set = -chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
		*vy_set =  chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
		*vz_set =  vz_set_channel;
}

static void chassis_set_contorl(chassis_t *chassis_control)
{
		if (chassis_control == NULL)
		{
				return;
		}
		
		fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set_add = 0.0f;		
		chassis_rc_to_control_vector(&vx_set, &vy_set, &angle_set_add, chassis_control);

		chassis_control->vx_set = fp32_constrain(vx_set, -SHIFT_NORMAL_MAX_CHASSIS_SPEED_X, SHIFT_NORMAL_MAX_CHASSIS_SPEED_X);
		chassis_control->vy_set = fp32_constrain(vy_set, -SHIFT_NORMAL_MAX_CHASSIS_SPEED_Y, SHIFT_NORMAL_MAX_CHASSIS_SPEED_Y);//通过两个带通滤波器，限制车速		
		
		//如果按下F就不管底盘跟随云台的闭环模式||无力模式
		if(chassis_control->chassis_RC->mouse.press_r|| chassis_control->chassis_mode==ZERO_POWER_MODE)
		{
				chassis_control->yaw_angle_set = chassis_control->yaw_angle;
				TD_set_x(&chassis_control->chassis_yaw_TD,chassis_control->yaw_angle_set);			
				chassis_control->wz_set = 0.0f;			
		}				
		else
		{
				chassis_control->yaw_angle_set = rad_format(chassis_control->yaw_angle_set + angle_set_add);
				//G是金矿模式，V是银矿模式，B是左转20°(18°)
				if((all_key.rotate_key_G.itself.mode != all_key.rotate_key_G.itself.last_mode)&&(picture_rotate_flag != 2))
				{
						if(picture_rotate_flag == 1)
						{		//-135°
								chassis_control->yaw_angle_set = rad_format(chassis_control->yaw_angle - 2.35f);
						}
						picture_rotate_flag = 1 - picture_rotate_flag;
				}
				if((all_key.rotate_key_V.itself.mode != all_key.rotate_key_V.itself.last_mode)&&(picture_rotate_flag != 1))
				{
						if(picture_rotate_flag == 2)
						{		//180°0
								chassis_control->yaw_angle_set = rad_format(chassis_control->yaw_angle + 3.14f);
						}				
						picture_rotate_flag = 2 - picture_rotate_flag;
				}
				if((all_key.picture_rotate_key.itself.mode != all_key.picture_rotate_key.itself.last_mode))
				{		//14°	
						chassis_control->yaw_angle_set = rad_format(chassis_control->yaw_angle + (1.57f/6.0f));
				}	
				TD_calc_angle(&chassis_control->chassis_yaw_TD, chassis_control->yaw_angle_set);
				PID_Calculate_Angle(&chassis_control->chassis_yaw_pid, chassis_control->yaw_angle, chassis_control->chassis_yaw_TD.x);		
				chassis_control->wz_set = chassis_control->chassis_yaw_pid.Output;				
		}	

		//夹矿
		if(clamp_mode == 1)
		{
				chassis_control->motor_clamp.speed_set =  15.0f;
		}
		else if(clamp_mode ==2)
		{
				if(clamp_flag == 1)
				{
						chassis_control->motor_clamp.position_set = -0.15f;
				}
				else
				{
						chassis_control->motor_clamp.position_set = -2.17f;
				}		
		}		
		//图传抬升
		if(picture_mode ==1)
		{
				chassis_control->motor_picture.speed_set = 0.0f;
		}
		else if(picture_mode ==2)
		{
				if(chassis_control->chassis_mode == SELF_CONTROL_MODE)
				{
						chassis.motor_picture.position_set = 16.5f;
				}
				else
				{		
						if(picture_rotate_flag == 0 || picture_rotate_flag ==2)
						{
								chassis.motor_picture.position_set = 12.5f;
						}
						else
						{
								chassis.motor_picture.position_set = 0.5f;
						}				
				}		
		}
		//图传旋转
		if(picture_rotate_flag == 0)
		{
				__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,1620);
		}
		else
		{ 
				__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,1620);
		}	
		//抬升
		if(lift_mode == 0)
		{
				TD_set_x(&chassis_control->lift_3508_TD,chassis_control->motor_lift.position);
		}	
		else if(lift_mode == 1)
		{
				chassis_control->motor_lift.speed_set = -18.0f;
		}	
		else if(lift_mode == 2)
		{
				TD_calc(&chassis_control->lift_3508_TD, arm_control.motor_1_position);
				chassis_control->motor_lift.position_set = chassis_control->lift_3508_TD.x;
		}		
		
		//shift+c键给机械臂下电再上电
		if(all_key.arm_restart_key1.itself.flag == 1 && all_key.arm_restart_key2.itself.flag == 1)
		{
				arm_restart_flag = 1;
		}				
		if((arm_restart_flag < 250) && (arm_restart_flag > 0))
		{
				HAL_GPIO_WritePin(GPIOH,GPIO_PIN_2,GPIO_PIN_RESET);
				arm_restart_flag++;
		}
		else if (arm_restart_flag >= 250)
		{
				lift_mode = 1;
				G_reset_flag = 0;
				arm_restart_flag = 0;		
		}
		else
		{
				HAL_GPIO_WritePin(GPIOH,GPIO_PIN_2,GPIO_PIN_SET);

		}					
}

static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    wheel_speed[0] = -vx_set - vy_set + ( CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] =  vx_set - vy_set + ( CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] =  vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}

static void chassis_control_loop(chassis_t *chassis_control_loop)
{
		fp32 max_vector = 0.0f, vector_rate = 0.0f;
		fp32 temp = 0.0f;
		fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
		uint8_t i = 0;
		//麦轮速度解算
		chassis_vector_to_mecanum_wheel_speed(-chassis_control_loop->vx_set,
																					chassis_control_loop->vy_set, chassis_control_loop->wz_set, wheel_speed);

		for (i = 0; i < 4; i++)
		{
				chassis_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
				temp = fabs(chassis_control_loop->motor_chassis[i].speed_set);
				if (max_vector < temp)
				{
						max_vector = temp;
				}
		}
		if (max_vector > MAX_WHEEL_SPEED)
		{
				vector_rate = MAX_WHEEL_SPEED / max_vector;
				for (i = 0; i < 4; i++)
				{
						chassis_control_loop->motor_chassis[i].speed_set *= vector_rate;
				}
		}
		
		//底盘电机
		for (i = 0; i < 4; i++)
		{
				PID_Calculate(&chassis_control_loop->chassis_motor_speed_pid[i], chassis_control_loop->motor_chassis[i].speed, chassis_control_loop->motor_chassis[i].speed_set);
				chassis_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_control_loop->chassis_motor_speed_pid[i].Output);
		}		
			
		//夹矿
		if(clamp_mode == 0)
		{
			chassis_control_loop->motor_clamp.give_current = 0;
		}
		else if(clamp_mode == 1)
		{
				PID_Calculate(&chassis_control_loop->clamp_motor_speed_pid, chassis_control_loop->motor_clamp.speed, chassis_control_loop->motor_clamp.speed_set);
				chassis_control_loop->motor_clamp.give_current = int16_constrain((int16_t)chassis_control_loop->clamp_motor_speed_pid.Output,-8000,8000);
		}
		else if(clamp_mode == 2)
		{		
				PID_Calculate(&chassis_control_loop->clamp_motor_position_pid, chassis_control_loop->motor_clamp.position, chassis_control_loop->motor_clamp.position_set);
				chassis_control_loop->motor_clamp.speed_set = chassis_control_loop->clamp_motor_position_pid.Output;
				PID_Calculate(&chassis_control_loop->clamp_motor_speed_pid, chassis_control_loop->motor_clamp.speed, chassis_control_loop->motor_clamp.speed_set);	
				chassis_control_loop->motor_clamp.give_current = (int16_t)(chassis_control_loop->clamp_motor_speed_pid.Output);
		}				
		//图传抬升
		if(picture_mode == 0)
		{
				chassis_control_loop->motor_picture.give_current = 0;
		}
		else if(picture_mode == 1)
		{
//				PID_Calculate(&chassis_control_loop->picture_motor_speed_pid, chassis_control_loop->motor_picture.speed, chassis_control_loop->motor_picture.speed_set);
//				chassis_control_loop->motor_picture.give_current = int16_constrain((int16_t)(chassis_control_loop->picture_motor_speed_pid.Output),-7000,7000);
				chassis_control_loop->motor_picture.give_current = -250;
		}
		else if(picture_mode == 2)
		{	
				PID_Calculate(&chassis_control_loop->picture_motor_position_pid, chassis_control_loop->motor_picture.position, chassis_control_loop->motor_picture.position_set);
				chassis_control_loop->motor_picture.speed_set = chassis_control_loop->picture_motor_position_pid.Output;
				PID_Calculate(&chassis_control_loop->picture_motor_speed_pid, chassis_control_loop->motor_picture.speed, chassis_control_loop->motor_picture.speed_set);
				chassis_control_loop->motor_picture.give_current = (int16_t)(chassis_control_loop->picture_motor_speed_pid.Output);
		}							
		//抬升
		if(lift_mode == 0)
		{
				chassis_control_loop->motor_lift.give_current = 0;
		}
		else if(lift_mode == 1)
		{
				PID_Calculate(&chassis_control_loop->lift_motor_speed_pid, chassis_control_loop->motor_lift.speed, chassis_control_loop->motor_lift.speed_set);
				chassis_control_loop->motor_lift.give_current = int16_constrain((int16_t)(chassis_control_loop->lift_motor_speed_pid.Output - 1800.0f),-12500,12500);
		}
		else if(lift_mode == 2)
		{	
				PID_Calculate(&chassis_control_loop->lift_motor_position_pid, chassis_control_loop->motor_lift.position, chassis_control_loop->motor_lift.position_set);
				chassis_control_loop->motor_lift.speed_set = chassis_control_loop->lift_motor_position_pid.Output;
				PID_Calculate(&chassis_control_loop->lift_motor_speed_pid, chassis_control_loop->motor_lift.speed, chassis_control_loop->motor_lift.speed_set);
				chassis_control_loop->motor_lift.give_current = int16_constrain((int16_t)(chassis_control_loop->lift_motor_speed_pid.Output - 1800.0f),-16383,16383);
		}	
}

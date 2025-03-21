#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "arm_math.h"

void chassis_behaviour_mode_set(chassis_t *chassis_move_mode)
{
	if (chassis_move_mode == NULL)
	{
		return;
	}

		last_mode_flag = mode_flag;
		if(switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
		{
			mode_flag = ONE_KEY_MODE;
		}
		else if(switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
		{
			mode_flag = SELF_CONTROL_MODE;
		}
		else
		{
			mode_flag = ZERO_POWER_MODE;
		}		
			
		if(switch_is_down(chassis_move_mode->chassis_RC->rc.s[1]))
		{
			suspension_flag = 0;
			clamp_flag = 0;
		}
		else
		{
			suspension_flag = 1;
			clamp_flag = 1;
		}
	
		if(mode_flag == ZERO_POWER_MODE)
		{
			lift_mode = 0;
		}
		if(last_mode_flag == ZERO_POWER_MODE && mode_flag != ZERO_POWER_MODE)
		{
			lift_mode = 1;
		}
		if(lift_mode == 1 && fabs(chassis_move_mode->motor_lift.speed) < 20.0f)
		{
			lift_reset_count++;
		}
		if(lift_mode == 1 && lift_reset_count>= 4000)
		{
			chassis_move_mode->motor_lift.round_cnt = 0;
			chassis_move_mode->motor_lift.offset_ecd = chassis_move_mode->motor_lift.motor_measure->ecd;
			chassis_move_mode->motor_lift.position = (chassis_move_mode->motor_lift.round_cnt * ECD_RANGE + chassis_move_mode->motor_lift.motor_measure->ecd - chassis_move_mode->motor_lift.offset_ecd) * MOTOR_ECD_TO_ANGLE_3508;
			TD_set_x(&chassis_move_mode->lift_3508_TD, chassis_move_mode->motor_lift.position);
			lift_mode = 2;
			lift_reset_count = 0;
		}		
}

void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_t *chassis_move_rc_to_vector)
{
	if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
	{
			return;
	}
	
	chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
	if(mode_flag !=  ZERO_POWER_MODE)
	{
		if(chassis_move_rc_to_vector->chassis_RC->key.Q)
		{
			if(chassis_move_rc_to_vector->chassis_RC->key.SHIFT)
			{	
				*angle_set = NORMAL_MAX_CHASSIS_SPEED_Z;
			}
			else
			{
				*angle_set = SLOW_CHASSIS_SPEED_Z;
			}
		}
		else if(chassis_move_rc_to_vector->chassis_RC->key.E)
		{
			if(chassis_move_rc_to_vector->chassis_RC->key.SHIFT)
			{	
				*angle_set = -NORMAL_MAX_CHASSIS_SPEED_Z;
			}
			else
			{
				*angle_set = -SLOW_CHASSIS_SPEED_Z;
			}
		}
	}
	if(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] != 0)
	{
		*angle_set = -CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
	}
	
}

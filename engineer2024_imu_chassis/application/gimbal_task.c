#include "gimbal_task.h"
#include "cmsis_os.h"
#include "ins_task.h"
#include "shoot.h"
#include "user_lib.h"
#include "pc_task.h"
#include "can_communicate.h"
#include "CyberGear.h"
#include "controller.h"
#include "bsp_dwt.h"
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
static void gimbal_init(gimbal_control_t *init);
void pitch_Kalman_Init(void);
void yaw_Kalman_Init(void);
static void gimbal_total_pid_clear(gimbal_control_t *gimbal_clear);
static void gimbal_feedback_update(gimbal_control_t *feedback_update);
static void gimbal_set_mode(gimbal_control_t *set_mode);
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change);
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
static void gimbal_absolute_pc_angle_limit(gimbal_motor_t *gimbal_motor, fp32 pc_angle_set);
static void gimbal_set_control(gimbal_control_t *set_control);
static void gimbal_control_loop(gimbal_control_t *control_loop);
static void gimbal_keyboard_update(void);

//static fp32 motor_angle_limit(fp32 angle, fp32 offset_angle);
//��̨���ƽṹ��
gimbal_control_t gimbal_control;

uint32_t timeout;
uint8_t key_switch_SHIFT;
static uint8_t  can_send_time = 0;
//���͵ĵ������
static int16_t yaw_can_set_current = 0;
static float32_t  pitch_can_set_current = 0;
static int16_t trigger_can_set_current = 0;
static int16_t fric1_can_set_current = 0;
static int16_t fric2_can_set_current = 0;

//��̨����
void gimbal_task(void const *pvParameters)
{
    //�ȴ������������������������
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //��̨��ʼ��
    gimbal_init(&gimbal_control);
    //�����ʼ��
    shoot_init();
    //��̨���ݷ���
    gimbal_feedback_update(&gimbal_control);
    //С�׵����ʼ��
    USART_cybergear_init();
    uint32_t system_clock = osKernelSysTick();
    while (1)
    {
				gimbal_control.dt = DWT_GetDeltaT(&gimbal_control.dwt_count);
        //detect_keyboard();
        gimbal_keyboard_update();//���̰�������
        gimbal_set_mode(&gimbal_control);                  	//������̨����ģʽ
        gimbal_mode_change_control_transit(&gimbal_control);//����ģʽ�л� �������ݹ���
        gimbal_feedback_update(&gimbal_control);          	//��̨���ݷ���
        gimbal_set_control(&gimbal_control);               	//������̨������
        gimbal_control_loop(&gimbal_control);              	//��̨����PID����
        trigger_can_set_current = shoot_control_loop();  	//����������ѭ��
        fric1_can_set_current = get_fric1();
        fric2_can_set_current = get_fric2();
        //����YAW_TURN��PITCH_TURN�Ըı䷢�͵���������
#if YAW_TURN
        yaw_can_set_current = (int16_t)(-gimbal_control.gimbal_yaw_motor.given_current);
#else
        yaw_can_set_current = (int16_t)gimbal_control.gimbal_yaw_motor.given_current;
#endif
#if PITCH_TURN
        pitch_can_set_current = -gimbal_control.gimbal_pitch_motor.given_current;
#else
        pitch_can_set_current = gimbal_control.gimbal_pitch_motor.given_current;
#endif
        CAN_gimbal_yaw_motor_can2(yaw_can_set_current);
        USART_cybergear_current_set(127,0,pitch_can_set_current);
        CAN_gimbal_shoot_motor_can1(fric1_can_set_current, fric2_can_set_current, trigger_can_set_current);
				if(can_send_time%17){
					CAN_cmd_communication();
				}
        vTaskDelayUntil(&system_clock, GIMBAL_CONTROL_TIME);
			 can_send_time++;
    }
}

				
////������ı�����ֵת������ԽǶȣ����ҽ���ԽǶ�������-PI~PI
//static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
//{
//    int32_t relative_ecd = ecd - offset_ecd;
//    if (relative_ecd > HALF_ECD_RANGE)
//    {
//        relative_ecd -= ECD_RANGE;
//    }
//    else if (relative_ecd < -HALF_ECD_RANGE)
//    {
//        relative_ecd += ECD_RANGE;
//    }
//    return relative_ecd * MOTOR_ECD_TO_RAD;
//}
//������ı�����ֵת������ԽǶȣ����ҽ���ԽǶ�������-PI~PI
//static fp32 motor_angle_limit(fp32 angle, fp32 offset_angle)
//{
//    fp32 relative_angle = angle - offset_angle;
//    if (relative_angle > PI_ONE)
//    {
//        relative_angle -= 2.0f * PI_ONE;
//    }
//    else if (relative_angle < -PI_ONE)
//    {
//        relative_angle += 2.0f * PI_ONE;
//    }
//    return relative_angle;
//}
//��̨У׼���ã�����yaw��pitch����ֵ�����ֵ����Сֵ
static void set_gimbal_cali(void)
{
    gimbal_control.gimbal_yaw_motor.offset_ecd = GIMBAL_YAW_OFFSET_ECD;
    gimbal_control.gimbal_pitch_motor.max_absolute_angle = GIMBAL_PITCH_ABSOLUTE_ANGLE_MAX;
    gimbal_control.gimbal_pitch_motor.min_absolute_angle = GIMBAL_PITCH_ABSOLUTE_ANGLE_MIN;
	  gimbal_control.gimbal_yaw_motor.max_absolute_angle = 1000;
    gimbal_control.gimbal_yaw_motor.min_absolute_angle = 1000;
}
//��̨��ʼ��
static void gimbal_init(gimbal_control_t *init)
{
    //���ñ�������ֵ����ԽǶ����ֵ����Сֵ
    set_gimbal_cali();
    //�������ָ���ȡ
    init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
    init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
    //����������ָ���ȡ
    init->gimbal_INT_angle_point = get_angle_data_point();
    init->gimbal_INT_gyro_point =  get_gyro_data_point();
    //ң��������ָ���ȡ
    init->gimbal_remote_data = get_remote_data_point();
    //��ʼ�����ģʽ
    init->gimbal_mode = init->gimbal_mode_last = GIMBAL_MODE_ZERO_FORCE;
   
    //��ʼ��yaw���pid
    gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_ABSOLUTE_ANGLE_PID_MAX_OUT, YAW_ABSOLUTE_ANGLE_PID_MAX_IOUT, YAW_ABSOLUTE_ANGLE_PID_KP, YAW_ABSOLUTE_ANGLE_PID_KI, YAW_ABSOLUTE_ANGLE_PID_KD);

    //��������˲�
    PID_Init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid,YAW_SPEED_PID_MAX_OUT,YAW_SPEED_PID_MAX_IOUT,0.0f,YAW_SPEED_PID_KP,YAW_SPEED_PID_KI,YAW_SPEED_PID_KD,0.0f,0.0f,0.0007957747154595f,0.0f,4,0x11);

    //��ʼ��pitch���pid
    gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_ABSOLUTE_ANGLE_PID_MAX_OUT, PITCH_ABSOLUTE_ANGLE_PID_MAX_IOUT, PITCH_ABSOLUTE_ANGLE_PID_KP, PITCH_ABSOLUTE_ANGLE_PID_KI, PITCH_ABSOLUTE_ANGLE_PID_KD);

    PID_Init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid,PITCH_SPEED_PID_MAX_OUT,PITCH_SPEED_PID_MAX_IOUT,0.0f,PITCH_SPEED_PID_KP,PITCH_SPEED_PID_KI,PITCH_SPEED_PID_KD,0.0f,0.0f,0.001061032953945968f,0.0f,4,0x11);
   	init->gimbal_pitch_motor.gravity_feedforward_current = PITCH_M * PITCH_L * GRAVITY * cos(init->gimbal_pitch_motor.absolute_angle) / KT;
    //�������PID
    gimbal_total_pid_clear(init);
    //���µ�����ݡ�����������
    gimbal_feedback_update(init);
    init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
    init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;
    init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
    init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;

    // ��ʼ��yaw��pitch���PCģʽTD
    TD_init(&init->gimbal_yaw_motor.PC_Control_Td, 500, 2, 0.001, init->gimbal_yaw_motor.absolute_angle);
    TD_init(&init->gimbal_pitch_motor.PC_Control_Td, 500, 2, 0.001, init->gimbal_pitch_motor.absolute_angle);
}

//����ģʽ
static void gimbal_set_mode(gimbal_control_t *set_mode)
{
    if (set_mode == NULL)
    {
        return;
    }
    gimbal_behaviour_mode_set();
}

//���µ�����ݡ�����������
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
    //����������ֵ���£������ͨ�˲�
    static const fp32 current_fliter_num[3] = {0.5966734000599f, -0.1753206407138f, 0.5786472406539};
    static fp32 current_fliter[3] = {0.0f, 0.0f, 0.0f};
    current_fliter[0] = current_fliter[1];
    current_fliter[1] = current_fliter[2];
    current_fliter[2] = current_fliter[1] * current_fliter_num[0] + current_fliter[0] * current_fliter_num[1] + feedback_update->gimbal_pitch_motor.gimbal_motor_measure->given_current * current_fliter_num[2];
    feedback_update->gimbal_pitch_motor.current_get = current_fliter[2];
    if (feedback_update == NULL)
    {
        return;
    }
    //pitch�������Ǿ��ԽǶȡ����ٶȸ���
    feedback_update->gimbal_pitch_motor.absolute_angle =  *(feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);
    //yaw�������Ǿ��ԽǶȡ����ٶȸ���
    feedback_update->gimbal_pitch_motor.motor_gyro =   *(feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET);
    feedback_update->gimbal_yaw_motor.absolute_angle =  *(feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
    //yaw���������ԽǶȸ���
    feedback_update->gimbal_yaw_motor.motor_gyro = (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET));

}

//�ǶȻ�PID��ʼ��
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->err = 0.0f;
    pid->get = 0.0f;
    pid->max_iout = max_iout;
    pid->max_out = maxout;
}
//�ǶȻ�PID����
static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;
    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    if(fabs(pid->err) > 0.03f)
        pid->Iout = 0;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}
//�ǶȻ�PID�������
static void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}
static void gimbal_total_pid_clear(gimbal_control_t *gimbal_clear) {
    gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);
    PID_Clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);
    gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid);
    PID_Clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);
}


//��̨ģʽ�ı�
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
    //������ģʽ�л�����������ģʽ
    if (gimbal_mode_change->gimbal_mode_last != GIMBAL_MODE_ZERO_FORCE && gimbal_mode_change->gimbal_mode == GIMBAL_MODE_ZERO_FORCE)
    {
        gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current;
	    	gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
		}
    //������ģʽ�л���������ģʽ
    else if (gimbal_mode_change->gimbal_mode_last != GIMBAL_MODE_ABSOLUTE_ANGLE && gimbal_mode_change->gimbal_mode == GIMBAL_MODE_ABSOLUTE_ANGLE)
    {
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
		}
    //������ģʽ�л���PC��׼ģʽ
       else if (gimbal_mode_change->gimbal_mode_last != GIMBAL_MODE_PC && gimbal_mode_change->gimbal_mode == GIMBAL_MODE_PC)
		{
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle; 
			  pc_receive_msg.rx_data.shoot_yaw = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
        pc_receive_msg.rx_data.shoot_pitch = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
			  before_yaw = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
			  before_pitch = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
		    TD_init(&gimbal_mode_change->gimbal_yaw_motor.PC_Control_Td, 180, 2, 0.001, gimbal_mode_change->gimbal_yaw_motor.absolute_angle);
			  TD_init(&gimbal_mode_change->gimbal_pitch_motor.PC_Control_Td, 180, 2, 0.001, gimbal_mode_change->gimbal_pitch_motor.absolute_angle);
    }
    //�洢��һ��ģʽ
    gimbal_mode_change->gimbal_mode_last = gimbal_mode_change->gimbal_mode;
}

//�ǶȻ�setֵ���㣬��setֵ����pid����
static void gimbal_set_control(gimbal_control_t *set_control)
{
    if (set_control == NULL)
    {
        return;
    }
    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;
    //������Ƕȱ仯��
    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle);
    //yaw���ģʽ����
    if (set_control->gimbal_mode== GIMBAL_MODE_ZERO_FORCE )
    {
        //ֱ�ӷ��Ϳ���ֵ
        set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
        set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
		}
    else if (set_control->gimbal_mode == GIMBAL_MODE_ABSOLUTE_ANGLE)
    {
        //�����ǽǶȿ���
        gimbal_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
        gimbal_absolute_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
		}
    else if (set_control->gimbal_mode == GIMBAL_MODE_PC)
    {
        //PC��׼ʱ�������ǽǶȿ���
        gimbal_absolute_pc_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
        gimbal_absolute_pc_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
		}
}

//���ԽǶȿ���
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    if(gimbal_motor == &gimbal_control.gimbal_pitch_motor)
    {
        if(gimbal_motor->absolute_angle_set + add > gimbal_motor->max_absolute_angle)
        {
            gimbal_motor->absolute_angle_set = gimbal_motor->max_absolute_angle;
        }
        else if(gimbal_motor->absolute_angle_set + add < gimbal_motor->min_absolute_angle)
        {
            gimbal_motor->absolute_angle_set = gimbal_motor->min_absolute_angle;
        }
        else
        {
            gimbal_motor->absolute_angle_set = rad_format(gimbal_motor->absolute_angle_set + add);
        }
    }
    else
    {
        gimbal_motor->absolute_angle_set = rad_format(gimbal_motor->absolute_angle_set + add);
    }


}
//PC������ƣ����ԽǶȿ��ƣ�
static void gimbal_absolute_pc_angle_limit(gimbal_motor_t *gimbal_motor, fp32 pc_angle_set)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    //��addֵ��Ϊ0ʱ�ı��趨ֵ
    if(gimbal_motor == &gimbal_control.gimbal_pitch_motor)
    {
        //�����̨��ԽǶ�+���Ƕ�+�����Ƕ�>�����ԽǶ�
        if (pc_angle_set > gimbal_control.gimbal_pitch_motor.max_absolute_angle)
        {
            pc_angle_set = gimbal_control.gimbal_pitch_motor.max_absolute_angle;
        }
        else if (pc_angle_set < gimbal_control.gimbal_pitch_motor.min_absolute_angle)
        {
            pc_angle_set = gimbal_control.gimbal_pitch_motor.min_absolute_angle;
        }
    }

    gimbal_motor->absolute_angle_set = rad_format(pc_angle_set);

}


//����ģʽ�·��͵���
static void gimbal_motor_zero_force_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
    gimbal_motor->given_current = gimbal_motor->current_set;
}

//���ԽǶ�PID�������
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

//#if ADRC_MODE
//    if(gimbal_motor == &gimbal_control.gimbal_yaw_motor)
//        gimbal_motor->current_set = ADRC_2_Calc(&gimbal_motor->yaw_motor_adrc, gimbal_motor->absolute_angle, gimbal_motor->motor_gyro, gimbal_motor->absolute_angle_set + 0.032164574 - 0.01005187);
//    else if(gimbal_motor == &gimbal_control.gimbal_pitch_motor)
//        gimbal_motor->current_set = ADRC_2_Calc(&gimbal_motor->pitch_motor_adrc,gimbal_motor->absolute_angle,gimbal_motor->motor_gyro,gimbal_motor->absolute_angle_set );
//    if(gimbal_motor == &gimbal_control.gimbal_pitch_motor)
//        gimbal_motor->given_current = PID_calc(&gimbal_motor->gimbal_motor_current_pid, gimbal_motor->current_get, gimbal_motor->current_set);
//    else if(gimbal_motor == &gimbal_control.gimbal_yaw_motor)
//        gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
//#else

    //�ǶȻ����ٶȻ�����pid����
    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_Calculate(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);//�˴�������δ��
    if(gimbal_motor == &gimbal_control.gimbal_pitch_motor){
			gimbal_motor->current_set += gimbal_motor->gravity_feedforward_current;
		}
		gimbal_motor->given_current = gimbal_motor->current_set;
//#endif
}
//���ݵ��ģʽ�����������
static void gimbal_control_loop(gimbal_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }
		control_loop->gimbal_pitch_motor.gravity_feedforward_current = PITCH_M * PITCH_L * GRAVITY * cos(control_loop->gimbal_pitch_motor.absolute_angle) / KT;
		   if (control_loop->gimbal_mode == GIMBAL_MODE_ZERO_FORCE)
    {
        gimbal_motor_zero_force_control(&control_loop->gimbal_yaw_motor);
			  gimbal_motor_zero_force_control(&control_loop->gimbal_pitch_motor);
    }
    else if (control_loop->gimbal_mode == GIMBAL_MODE_ABSOLUTE_ANGLE || control_loop->gimbal_mode == GIMBAL_MODE_PC)
    {
        gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_motor);
			  gimbal_motor_absolute_angle_control(&control_loop->gimbal_pitch_motor);
    }

}
//���̸���
static void gimbal_keyboard_update(void) {
    remote_key_switch(&(gimbal_control.gimbal_remote_data->key_v),KEY_PRESSED_OFFSET_V);//�ߵ���Ƶ�л�
    remote_key_switch(&(gimbal_control.gimbal_remote_data->key_g),KEY_PRESSED_OFFSET_G);//��������ģʽ�л�

    remote_key_switch(&(gimbal_control.gimbal_remote_data->key_b),KEY_PRESSED_OFFSET_B);//������ģʽ
    remote_key_switch(&(gimbal_control.gimbal_remote_data->key_r),KEY_PRESSED_OFFSET_R);//
}

#include "shoot.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "arm_math.h"
#include "gimbal_behaviour.h"
#include "laser.h"
#include "remote_control.h"
#include "gimbal_task.h"
#define shoot_laser_on() laser_on()//���⿪���궨��
#define shoot_laser_off() laser_off()//����رպ궨��
//extern uint8_t bullet_speed_corret_flag;
shoot_control_t shoot_control;//�������
uint8_t shoot_fire_state;					// Ħ���ֿ��� UI
uint8_t block_flag = 0;					// �����̶�ת ��־λ��UI
//����ģʽ�л�

uint8_t continue_shoot_start_flag;
uint8_t continue_shoot_end_flag;
static  int8_t last_s = RC_SW_UP;
ramp_function_source_t fric_ramp;
uint8_t READY_Flag;
slid_avg_filter_t fire_speed_avg_filter;
static void shoot_feedback_update(void);
static void shoot_set_mode(void);
static void trigger_block_check(void);
static void shoot_bullet_control(void);
static void shoot_adjust_control(void);
//static void shoot_heart_control_low(void);
static void shoot_heart_control_high(void);
//static void cali_shoot_delay_time(void);
int fire_heat_flag = 0;
//���������ʼ��
void shoot_init(void)
{
    //PID������ʼ��
    // static const fp32 Trigger_speed_pid[3] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};
    static const fp32 Trigger_angle_pid[3] = {TRIGGER_ANGLE_PID_KP,TRIGGER_ANGLE_PID_KI,TRIGGER_ANGLE_PID_KD};
    //static const fp32 fric_speed_pid[3] = {FRIC_SPEED_PID_KP, FRIC_SPEED_PID_KI, FRIC_SPEED_PID_KD};
    //����ģʽ��ʼ��
    shoot_control.shoot_mode = SHOOT_STOP;
    //ң����ָ��
    shoot_control.shoot_remote_data = get_remote_data_point();
    shoot_control.shoot_rc = get_remote_control_point();
    //�����־λ
    shoot_control.shoot_pc_data.pc_rx_data = get_pc_rx_point();
    //���ָ��
    shoot_control.trigger_motor_measure = get_trigger_motor_measure_point();
    shoot_control.fric1_motor_measure = get_fric1_motor_measure_point();
    shoot_control.fric2_motor_measure = get_fric2_motor_measure_point();
    //��ʼ��PID
    // PID_init(&shoot_control.trigger_speed_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_SPEED_PID_MAX_OUT, TRIGGER_SPEED_PID_MAX_IOUT);
    PID_init(&shoot_control.trigger_angle_pid, PID_POSITION, Trigger_angle_pid, TRIGGER_ANGLE_PID_MAX_OUT, TRIGGER_ANGLE_PID_MAX_IOUT);
    //  PID_init(&shoot_control.fric_motor_pid, PID_POSITION, fric_speed_pid, FRIC_SPEED_PID_MAX_OUT, FRIC_SPEED_PID_MAX_OUT);

    PID_Init(&shoot_control.trigger_speed_pid,TRIGGER_SPEED_PID_MAX_OUT,TRIGGER_SPEED_PID_MAX_IOUT,0.0f,TRIGGER_SPEED_PID_KP,TRIGGER_SPEED_PID_KI,TRIGGER_SPEED_PID_KD,0.0f,0.0f,0.00159154943091895335768883763373f,0.0f,4,0x11);
    //  PID_Init(&shoot_control.trigger_angle_pid,TRIGGER_ANGLE_PID_MAX_OUT,TRIGGER_ANGLE_PID_MAX_IOUT,0.0f,TRIGGER_ANGLE_PID_KP,TRIGGER_ANGLE_PID_KI,TRIGGER_ANGLE_PID_KD,0.0f,0.0f,0.00159154943091895335768883763373f,0.0f,4,0x11);
    PID_Init(&shoot_control.fric_motor_pid[0],FRIC_SPEED_PID_MAX_OUT,FRIC_SPEED_PID_MAX_IOUT,0.0f,FRIC_SPEED_PID_KP,FRIC_SPEED_PID_KI,FRIC_SPEED_PID_KD,0.0f,0.0f,0.00159154943091895335768883763373f,0.0f,4,0x11);
    PID_Init(&shoot_control.fric_motor_pid[1],FRIC_SPEED_PID_MAX_OUT,FRIC_SPEED_PID_MAX_IOUT,0.0f,FRIC_SPEED_PID_KP,FRIC_SPEED_PID_KI,FRIC_SPEED_PID_KD,0.0f,0.0f,0.00159154943091895335768883763373f,0.0f,4,0x11);
    //��������
    shoot_control.bullet_speed_set = BULLET_SPEED_SET;
    shoot_control.bullet_speed_fix = BULLET_SPEED_FIX_K;
    TD_init(&shoot_control.fric_speed_set_td,10,20,0.001,0);    //��ʼ��TD
    shoot_feedback_update();
    shoot_control.fric_speed_set = 0;
    shoot_control.trigger_angle = 0.0f;
    shoot_control.trigger_angle_set = 0.0f;
    READY_Flag = 0;
    block_flag = 0;
}

/**
  * @brief          ���ѭ��
  * @param[in]      void
  * @retval         ����can����ֵ
  */
int16_t shoot_control_loop(void)
{

    shoot_set_mode();        //����״̬��
    shoot_feedback_update(); //��������
    if(shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_laser_off();  //�رռ���
        //���ò����ֵ��ٶ�
        shoot_control.trigger_speed_set = 0.0f;
        shoot_control.trigger_angle_set = shoot_control.trigger_angle;

        shoot_control.trigger_given_current = 0;
        shoot_control.fric_speedset[0] = 0.0f;
        shoot_control.fric_speedset[1] = 0.0f;
        shoot_fire_state = 0;
    }
    else
    {
        if( shoot_control.shoot_pc_data.pc_rx_data->fire == 1 && gimbal_control.gimbal_mode == GIMBAL_MODE_PC) {
            shoot_control.shoot_pc_data.pc_continue_flag = 1;
            shoot_control.shoot_pc_data.pc_continue_count = 0;
        }
        if (shoot_control.shoot_pc_data.pc_continue_flag == 1 && shoot_control.shoot_pc_data.pc_continue_count < PC_TIME)
        {
            shoot_control.shoot_pc_data.pc_continue_count ++;
        }
        shoot_fire_state = 1;
        shoot_laser_on(); //���⿪��


        if (shoot_control.shoot_mode == SHOOT_READY)
        {
        //�Ƿ���G��
        if (shoot_control.shoot_remote_data->key_g.key_switch == 0)
        {   //Gû����
						shoot_control.fric_speed_set = SHOOT_FIRE_SPEED_LOW;
        }
        else//G������
        {   //û�ж�ת���Ҳ��뿪������ ���� ��������ģʽ������ ���� ����������
						shoot_control.fric_speed_set = SHOOT_FIRE_SPEED_HIGH;
        }						
            

//					if(READY_Flag == 0){
//						shoot_control.shoot_mode = SHOOT_ADJUST;
//						READY_Flag++;
//					}

            shoot_control.trigger_speed_set = shoot_PID_calc(&shoot_control.trigger_angle_pid,shoot_control.trigger_angle,shoot_control.trigger_angle_set);
        }
        else if (shoot_control.shoot_mode == SHOOT_BULLET)//����
        {
            shoot_bullet_control();
        }
        else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)//����
        {
            TD_calc(&shoot_control.fric_speed_set_td, shoot_control.bullet_speed_fix * shoot_control.bullet_speed_delta);
            shoot_control.fric_speed_set = SHOOT_FIRE_SPEED_LOW + shoot_control.fric_speed_set_td.x;
            //��������
//            if(gimbal_control.gimbal_remote_data->key_v.key_switch == 1) { //����
                shoot_heart_control_high();
//            } else { //����
//                shoot_heart_control_low();
//            }

            /* �����������յ���һ�ε�����ָ���������*/
            if( pc_receive_msg.rx_data.fire  == 1 && shoot_control.shoot_pc_data.pc_continue_flag == 1 && gimbal_control.gimbal_mode == GIMBAL_MODE_PC)//pc_receive_msg.rx_data.fire
            {
                shoot_control.shoot_pc_data.pc_continue_count = 0;
            }
        }
        else if(shoot_control.shoot_mode == SHOOT_DONE)//������
        {
            shoot_control.trigger_speed_set = 0.0f;
        }
        else if(shoot_control.shoot_mode == SHOOT_ADJUST)
        {
            shoot_control.trigger_speed_set =  SHOOT_ADJUST_SPEED;//��ת������λ��
            shoot_adjust_control();
        }

        trigger_block_check();
        //���㲦���ֵ��PID
        //������ַ����Լ�С����
        if(fabs(shoot_control.trigger_speed_pid.Err) >= 0.3f)
            shoot_control.trigger_speed_pid.Iout = 0;
        PID_Calculate(&shoot_control.trigger_speed_pid, shoot_control.trigger_speed, shoot_control.trigger_speed_set);
        shoot_control.trigger_given_current = (int16_t)(shoot_control.trigger_speed_pid.Output);
        shoot_control.fric_speedset[0] = - shoot_control.fric_speed_set;
        shoot_control.fric_speedset[1] =  shoot_control.fric_speed_set;
    }
    shoot_control.fric_given_current[0] = PID_Calculate(&shoot_control.fric_motor_pid[0], shoot_control.fric_speed[0], shoot_control.fric_speedset[0]);
    shoot_control.fric_given_current[1] = PID_Calculate(&shoot_control.fric_motor_pid[1], shoot_control.fric_speed[1], shoot_control.fric_speedset[1]);

    return shoot_control.trigger_given_current;
}


//����ģʽ�л�
static void shoot_set_mode(void)
{
    if(shoot_control.shoot_mode == SHOOT_STOP) {
        //�ϲ��жϣ� ��һ�ο���Ħ����
        if (switch_is_up(shoot_control.shoot_remote_data->rc_ctrl_t->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s)) {
            shoot_control.shoot_mode = SHOOT_READY_FRIC;
        }
        if(switch_is_mid(shoot_control.shoot_remote_data->rc_ctrl_t->rc.s[SHOOT_RC_MODE_CHANNEL])\
                &&(shoot_control.shoot_remote_data->rc_ctrl_t->key.v & SHOOT_ON_KEYBOARD)) {
            shoot_control.shoot_mode = SHOOT_READY_FRIC;
        }
    } else {
        if (switch_is_up(shoot_control.shoot_remote_data->rc_ctrl_t->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s)) {
            shoot_control.shoot_mode = SHOOT_STOP;
        }
        if (switch_is_mid(shoot_control.shoot_remote_data->rc_ctrl_t->rc.s[SHOOT_RC_MODE_CHANNEL]) \
                &&(shoot_control.shoot_remote_data->rc_ctrl_t->key.v & SHOOT_OFF_KEYBOARD)) {
            shoot_control.shoot_mode = SHOOT_STOP;
        }
    }
    //���Ħ����׼��ģʽ����Ħ���ִﵽ�趨�ٶ�
    if(shoot_control.shoot_mode == SHOOT_READY_FRIC)
    {
        if(fabs(shoot_control.fric_speed[0]-shoot_control.fric_speedset[0])<FRIC_DEADBAND\
                && fabs(shoot_control.fric_speed[1]-shoot_control.fric_speedset[1])<FRIC_DEADBAND) {
            shoot_control.shoot_mode = SHOOT_READY;//����׼��ģʽ
        }
    }
    else if(shoot_control.shoot_mode == SHOOT_READY)
    {
        if (switch_is_down(shoot_control.shoot_remote_data->rc_ctrl_t->rc.s[SHOOT_RC_MODE_CHANNEL])&& !switch_is_down(last_s)) {
            shoot_control.shoot_mode = SHOOT_BULLET;
        }
        //�Ƿ���G��
        if (shoot_control.shoot_remote_data->key_g.key_switch == 0)
        {   //Gû����
            //���뿪�ص�һ������ ���� ��������ģʽ������ ���� ���һ�����
            if(gimbal_control.gimbal_mode == GIMBAL_MODE_PC && shoot_control.shoot_pc_data.pc_rx_data->fire  == 1) {
                shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
            }
            if(shoot_control.shoot_remote_data->mouse_press_l.press == 1 ) {
                shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
            }
        }
        else//G������
        {   //û�ж�ת���Ҳ��뿪������ ���� ��������ģʽ������ ���� ����������
            if(gimbal_control.gimbal_mode == GIMBAL_MODE_PC && shoot_control.shoot_pc_data.pc_rx_data->fire  == 1) {
                shoot_control.shoot_mode = SHOOT_BULLET;
            }
            if(shoot_control.shoot_remote_data->mouse_press_l.press == 1 && shoot_control.shoot_remote_data->mouse_press_l.last_press == 0 ) {
                shoot_control.shoot_mode = SHOOT_BULLET;
            }
        }
        //����ʱ�䳬����ֵ
        if (shoot_control.rc_s_time >= RC_S_LONG_TIME) {
            shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
        }
        /*pc���Ʒ���*/
        if(shoot_control.shoot_pc_data.pc_continue_flag == 1 && gimbal_control.gimbal_mode == GIMBAL_MODE_PC) {
            shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
        }
    }
    else if(shoot_control.shoot_mode == SHOOT_BULLET) {
        //�������Ҳ��벻����
        if(!switch_is_down(shoot_control.shoot_remote_data->rc_ctrl_t->rc.s[SHOOT_RC_MODE_CHANNEL])&& switch_is_down(last_s)) {
            shoot_control.shoot_mode = SHOOT_DONE;
        }
        if(shoot_control.shoot_remote_data->mouse_press_l.press == 0 && shoot_control.shoot_remote_data->key_g.key_switch == 1) {
            shoot_control.shoot_mode = SHOOT_DONE;
        }
    }//���˴��µ��л���pc����һС��ʱ�䣨��֤��������ӵ�����ֹͣ����
    else if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET ) {
        if(!switch_is_down(shoot_control.shoot_remote_data->rc_ctrl_t->rc.s[SHOOT_RC_MODE_CHANNEL]) && switch_is_down(last_s)) {
            shoot_control.shoot_mode = SHOOT_ADJUST;
        }
//				if(gimbal_control.gimbal_mode == GIMBAL_MODE_PC&&shoot_control.shoot_pc_data.pc_rx_data->fire == 0){
//						shoot_control.shoot_mode = SHOOT_DONE;
//						shoot_control.trigger_angle_set = shoot_control.trigger_angle - PI_FOUR;
//				}
				if(gimbal_control.gimbal_mode != GIMBAL_MODE_PC && shoot_control.shoot_remote_data->mouse_press_l.press == 0 \
                && shoot_control.shoot_remote_data->key_g.key_switch == 0 \
				        && switch_is_mid(shoot_control.shoot_remote_data->rc_ctrl_t->rc.s[SHOOT_RC_MODE_CHANNEL])){
						shoot_control.shoot_mode = SHOOT_DONE;
						shoot_control.trigger_angle_set = shoot_control.trigger_angle - PI_FOUR;
								}
				if(shoot_control.shoot_pc_data.pc_continue_count == PC_TIME) {
            shoot_control.shoot_mode = SHOOT_DONE;
					  shoot_control.trigger_angle_set = shoot_control.trigger_angle - PI_FOUR;
        }
    }
    else if(shoot_control.shoot_mode == SHOOT_DONE) //������
    {
        shoot_control.shoot_mode = SHOOT_READY;
    }

    /*�ⷢ���ӳ�*/
//    cali_shoot_delay_time();
    //�����̨״̬�� ����״̬���͹ر����
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    last_s = shoot_control.shoot_remote_data->rc_ctrl_t->rc.s[SHOOT_RC_MODE_CHANNEL];

}
//void shoot_heart_control_low(void) {

//    if((shoot_control.shoot_referee_data.heat + SHOOT_HEAT_REMAIN_VALUE) < 0.7f * shoot_control.shoot_referee_data.heat_limit)
//        shoot_control.trigger_speed_set = - SHOOT_LOW_TRIGGER_SPEED;
//    else if((shoot_control.shoot_referee_data.heat + SHOOT_HEAT_REMAIN_VALUE) < shoot_control.shoot_referee_data.heat_limit)
//        shoot_control.trigger_speed_set = - (3.0f - 2.857f*(shoot_control.shoot_referee_data.heat + SHOOT_HEAT_REMAIN_VALUE)
//                                             / shoot_control.shoot_referee_data.heat_limit) * SHOOT_LOW_TRIGGER_SPEED;
//    else
//        shoot_control.trigger_speed_set = - 0.15f * SHOOT_LOW_TRIGGER_SPEED;

//}
void  shoot_heart_control_high(void) {

//	if(shoot_control.shoot_referee_data.heat < 0.95 * (shoot_control.shoot_referee_data.heat_limit - shoot_control.shoot_referee_data.cooling_value))
//	{
//		shoot_control.trigger_speed_set = - SHOOT_HIGH_TRIGGER_SPEED;
//	}else if(shoot_control.shoot_referee_data.heat < 0.95 * shoot_control.shoot_referee_data.heat_limit){
//		shoot_control.trigger_speed_set = shoot_control.shoot_referee_data.cooling_value * 2 * PI / 10 / 8;
//
//	}else{
//	   shoot_control.trigger_speed_set = -0.1f * SHOOT_LOW_TRIGGER_SPEED;
//	}
		if(gimbal_control.gimbal_mode == GIMBAL_MODE_PC)
		{
				if(shoot_control.shoot_referee_data.heat_limit - shoot_control.shoot_referee_data.heat > SHOOT_HEAT_REMAIN_VALUE_1
					&& fire_heat_flag == 0)
				{
						shoot_control.trigger_speed_set = - SHOOT_HIGH_TRIGGER_SPEED; 				
				}
		    else
				{
						shoot_control.trigger_speed_set = 0.0f;		
						fire_heat_flag = 1;					
				}
				
				if(shoot_control.shoot_referee_data.heat_limit - shoot_control.shoot_referee_data.heat > SHOOT_HEAT_REMAIN_VALUE_2)
				{
						fire_heat_flag = 0;
				}
							
		}
		else
		{
				if((shoot_control.shoot_referee_data.heat + SHOOT_HEAT_REMAIN_VALUE) < 0.7f * shoot_control.shoot_referee_data.heat_limit)
						shoot_control.trigger_speed_set = - SHOOT_LOW_TRIGGER_SPEED;
		    else if((shoot_control.shoot_referee_data.heat + SHOOT_HEAT_REMAIN_VALUE) < 0.9f*shoot_control.shoot_referee_data.heat_limit)
		        shoot_control.trigger_speed_set = - 0.5f* SHOOT_LOW_TRIGGER_SPEED;
		    else
						shoot_control.trigger_speed_set = - 0.1f * SHOOT_LOW_TRIGGER_SPEED;		
		}
}
//���ݸ��£�����Ħ�����ٶȣ��������ת�٣�����������ʱ
/**
  * @brief          ������ݸ���
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{
    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;
    static float32_t bullet_speed_fliter_1 = 0.0f;
    static float32_t bullet_speed_fliter_2 = 0.0f;
    static float32_t bullet_speed_fliter_3 = 0.0f;
    //�����ֵ���ٶ��˲�һ��
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //���׵�ͨ�˲�
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.trigger_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.trigger_speed = speed_fliter_3;
    bullet_speed_fliter_1 = bullet_speed_fliter_2;
    bullet_speed_fliter_2 = bullet_speed_fliter_3;
    bullet_speed_fliter_3 = bullet_speed_fliter_2 * fliter_num[0] +
                            bullet_speed_fliter_1 * fliter_num[1] +
                            (fire_speed.fire_speed_value) * fliter_num[2];
    shoot_control.bullet_speed = bullet_speed_fliter_3;
    shoot_control.bullet_speed_delta = shoot_control.bullet_speed_set - shoot_control.bullet_speed;
    //	Slid_avg_filter(&fire_speed_avg_filter,shoot_control.bullet_speed_delta);
    shoot_control.fric_speed[0] = shoot_control.fric1_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED;
    shoot_control.fric_speed[1] = shoot_control.fric2_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED;

    //��갴��
    shoot_control.shoot_remote_data->mouse_press_l.last_press = shoot_control.shoot_remote_data->mouse_press_l.press;
    shoot_control.shoot_remote_data->mouse_press_r.last_press = shoot_control.shoot_remote_data->mouse_press_r.press;
    shoot_control.shoot_remote_data->mouse_press_l.press =  shoot_control.shoot_remote_data->rc_ctrl_t->mouse.press_l;
    shoot_control.shoot_remote_data->mouse_press_r.press =  shoot_control.shoot_remote_data->rc_ctrl_t->mouse.press_r;
    //������ʱ
    if (shoot_control.shoot_remote_data->mouse_press_l.press)
    {
        if (shoot_control.shoot_remote_data->mouse_press_l.press_time < PRESS_LONG_TIME)
        {
            shoot_control.shoot_remote_data->mouse_press_l.press_time++;
        }
    }
    else
    {
        shoot_control.shoot_remote_data->mouse_press_l.press_time = 0;
    }
    if (shoot_control.shoot_remote_data->mouse_press_r.press)
    {
        if (shoot_control.shoot_remote_data->mouse_press_r.press_time < PRESS_LONG_TIME)
        {
            shoot_control.shoot_remote_data->mouse_press_r.press_time++;
        }
    }
    else
    {
        shoot_control.shoot_remote_data->mouse_press_r.press_time = 0;
    }
    //��������µ�ʱ���ʱ
    if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_down(shoot_control.shoot_remote_data->rc_ctrl_t->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {

        if (shoot_control.rc_s_time < RC_S_LONG_TIME)
        {
            shoot_control.rc_s_time++;
        }
    }
    else
    {
        shoot_control.rc_s_time = 0;
    }
}

/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)
{

    //ÿ�β��� 1/4PI�ĽǶ�
    if (shoot_control.move_flag == 0)
    {
        shoot_control.trigger_angle_set = rad_format(shoot_control.trigger_angle - PI_FOUR);
        shoot_control.move_flag = 1;
    }
    //����Ƕ��ж�
    if (rad_format(shoot_control.trigger_angle_set - shoot_control.trigger_angle) > 0.05f)
    {
        shoot_control.trigger_speed_set = shoot_PID_calc(&shoot_control.trigger_angle_pid,shoot_control.trigger_angle,shoot_control.trigger_angle_set);
    }
    else
    {
        shoot_control.move_flag = 0;
        shoot_control.shoot_mode = SHOOT_DONE;
    }
}

/**
  * @brief          ��ת��ת���
  * @param[in]      void
  * @retval         void
  */
static void trigger_block_check(void)
{
    if(shoot_control.shoot_mode == SHOOT_BULLET ||shoot_control.shoot_mode == SHOOT_ADJUST)
    {
        //�Ƕ�û�ﵽ�趨�Ƕ�
        if(fabs(shoot_control.trigger_angle - shoot_control.trigger_angle_set ) > TRIGGER_BLOCK_ANGLE)
        {
            shoot_control.block_time++;
            if(shoot_control.block_time == TRIGGER_BLOCK_TIME) {
                block_flag = 1;
            }
        }
        else
        {
            block_flag = 0;
            shoot_control.block_time = 0;
        }
    }
    else if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
    {
        if(fabs(shoot_control.trigger_speed - shoot_control.trigger_speed_set ) > TRIGGER_BLOCK_SPEED)
        {
            shoot_control.block_time++;
            if(shoot_control.block_time == TRIGGER_BLOCK_TIME)
                block_flag = 1;
        }
        else
        {
            block_flag = 0;
            shoot_control.block_time = 0;
        }
    }
}
//����Ħ���ֵ���ֵ
int16_t get_fric1(void)
{
    return shoot_control.fric_given_current[0];
}
int16_t get_fric2(void)
{
    return shoot_control.fric_given_current[1];
}
void shoot_trigger_angle_update(void) {
    if (shoot_control.trigger_motor_measure->ecd - shoot_control.trigger_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        shoot_control.trigger_angle_per -= 2.0f * PI_ONE / REDUCTION_RATIO;//��ת
    }
    else if (shoot_control.trigger_motor_measure->ecd - shoot_control.trigger_motor_measure->last_ecd < -HALF_ECD_RANGE)//��ʱecd����ECD_RANGE����0��ʼ����
    {
        shoot_control.trigger_angle_per += 2.0f * PI_ONE / REDUCTION_RATIO;//��ת
    }
    if (shoot_control.trigger_angle_per >= PI_ONE)//��֤trigger_angle��-PI��PI֮��
    {
        shoot_control.trigger_angle_per -= 2.0f * PI_ONE;
    }
    else if (shoot_control.trigger_angle_per <= -PI_ONE)
    {
        shoot_control.trigger_angle_per += 2.0f * PI_ONE;
    }
    //���������Ƕ�
    shoot_control.trigger_angle = shoot_control.trigger_angle_per + shoot_control.trigger_motor_measure->ecd * MOTOR_ECD_TO_ANGLE / REDUCTION_RATIO;
}
//���������ģʽ�Ƕ��趨ֵ���㣨Ϊ�˴ﵽ�����ĺ��ʽǶȣ�
static void shoot_adjust_control(void)
{
    if(block_flag == 1)
    {
        shoot_control.trigger_angle_set = shoot_control.trigger_angle - PI_FOUR/16;//��תֻ�������ٶȣ�set_angleû�䣬��ʹ�õ�ǰ�Ƕȿ��ƣ��뵥�������߼�����
        shoot_control.shoot_mode = SHOOT_READY;
        block_flag = 0;
    }
}

//uint8_t shoot_num = 0;
//uint16_t shoot_delay_count;
//uint32_t shoot_single_start_cnt;
//uint32_t shoot_continue_start_cnt;
//uint16_t shoot_delay_time_single = 0;
//uint16_t shoot_delay_time_continue = 0;
//static void cali_shoot_delay_time(void)
//{
//	/*���Ե���ģʽ*/
//	/*�����ʼ����*/
//    if((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s)))
//		{
////				shoot_start_flag = 1;
//			  shoot_single_start_cnt = DWT->CYCCNT;
//		}
//	/*�������ʱ*/
//	/*����ϵͳ��⵽��ֹͣ��ʱ*/
//	//��ȡ���������ϵͳ���֮���dwt��ʱ����ʱ���Ӷ����㷢���ӳ�
//    if (bullet_speed_corret_flag)
//		{
//			shoot_delay_time_single = (DWT->CYCCNT - shoot_single_start_cnt)/168000;//��ƵΪ168MHz������Ϊ����
//			if(continue_shoot_end_flag == 0)
//			{
//				shoot_delay_time_continue = (DWT->CYCCNT - shoot_continue_start_cnt)/168000;
//				continue_shoot_end_flag = 1;
//			}
//			bullet_speed_corret_flag = 0;
////			shoot_start_flag = 0;
//
//		}
//
//	 /*��������ģʽ*/
//		if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
//		{
//			if( continue_shoot_start_flag == 0 )
//			{
//			  shoot_continue_start_cnt = DWT->CYCCNT;
//				continue_shoot_start_flag = 1;
//				continue_shoot_end_flag = 0;
//			}
//				}
////				/*����ϵͳ��⵽��ֹͣ��ʱ*/
//
//		if(!switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))	continue_shoot_start_flag = 0;
//}

/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef __CAN_COMMUNICATE_H
#define __CAN_COMMUNICATE_H

#include "struct_typedef.h"
#include "can.h"
#include "arm_math.h"

#define CAN_YAW_TYPE &hcan2
#define CAN_SHOOT_TYPE &hcan1
#define CAN_CHASSIC_COM_TYPE &hcan2
/* CAN send and receive ID */
typedef enum
{
    CAN_SHOOT_ALL_ID    = 0X1FF,// ��������ʶ��
    CAN_FRIC1_M1_ID 		= 0x205,//Ħ����1(��)
    CAN_FRIC2_M2_ID 		= 0x206,//Ħ����2(��)
    CAN_TRIGGER_M3_ID	  = 0x207,//�������

    CAN_YAW_ALL_ID			= 0x2FF,//6020 Y������ʶ��
    CAN_YAW_M5_ID 		  = 0x209,//Y�� 0X204+ID
    CAN_DEMO_ALL_ID     = 0X200,
    CAN_DEMO_ID 		    = 0x204,
		CAN_REFEREE_ID 			= 0x230,
		CAN_COMMUNICATE_ID 		= 0x220,
	  CAN_SPEED_ID  = 0X240,
} can_msg_id_e;
union fire_speed_u
{
    float fire_speed_value;
    uint8_t fire_speed_rx[4];
};

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

extern uint8_t color_flag;
extern union fire_speed_u fire_speed;
extern void CAN_gimbal_yaw_motor_can2(int16_t current);
extern void CAN_gimbal_pitch_motor_can2(int16_t current);
extern void CAN_gimbal_shoot_motor_can1(int16_t Fric1_Motor_Data, int16_t Fric2_Motor_Data,int16_t Trigger_Motor_Data);

extern void CAN_cmd_communication(void);
void USART1_RX_IRQHandler(void);
/**
  * @brief          ����yaw 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);


/**
  * @brief          ����pitch 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
  * @brief          ���ز������ 2006�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);


/**
  * @brief          ���ص��̵�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,3]
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);
extern const motor_measure_t *get_fric2_motor_measure_point(void);
extern const motor_measure_t *get_fric1_motor_measure_point(void);
extern const motor_measure_t *get_trigger_motor_measure_point(void);

#endif

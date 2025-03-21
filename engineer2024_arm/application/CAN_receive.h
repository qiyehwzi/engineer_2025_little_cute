/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
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

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define ARM_FOUR_SIX_CAN hcan1
#define ARM_3508_board_CAN hcan2

/* CAN send and receive ID */
typedef enum
{
	MST_1_ID = 0x02,
	MST_2_ID = 0x04,
	MST_3_ID = 0X06,
	DM_4310_M1_TX_ID = 0x01,
	DM_4310_M2_TX_ID = 0x03,
	DM_4310_M3_TX_ID = 0X05,
	CAN_ARM_CMD_ID = 0X1ff,
	CAN_2006_M1_ID = 0x206,
	CAN_2006_M2_ID = 0x207,
	CAN_3508_M1_ID = 0x205,
} can_msg_id_3508_e;

typedef enum
{
	CAN_COMMUNICATE_RX_ID_0 = 0x220,
	CAN_COMMUNICATE_RX_ID_1 = 0x221,
	CAN_COMMUNICATE_TX_ID_0 = 0x230,
	CAN_COMMUNICATE_TX_ID_1 = 0x231,
}can_board_communicate;

typedef struct
{
	fp32 target_position[6];
	uint8_t mode;
	uint8_t suker_mode;
}board_communicate;

//rm motor data
typedef struct
{
	uint16_t ecd;
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperate;
	int16_t last_ecd;
} motor_measure_t;

typedef struct
{
	uint8_t motor_err;
	uint8_t motor_tx_id;
	uint8_t motor_enabled;
	fp32 motor_position;
	fp32 motor_speed;
	fp32 motor_T;
}motor_DM_motor_t;

///**
//  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
//  * @param[in]      none
//  * @retval         none
//  */
//extern void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          发送底盘电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */

extern void CAN_cmd_4310_1(fp32 DM_motor_position, fp32 DM_motor_speed);
extern void CAN_cmd_4310_2(fp32 DM_motor_position, fp32 DM_motor_speed);
extern void CAN_cmd_4310_3(fp32 DM_motor_position, fp32 DM_motor_speed);

extern void CAN_cmd_4310_mit_1(fp32 DM_motor_position,fp32 DM_motor_speed,fp32 Kp,fp32 Kd,fp32 torq);
extern void CAN_cmd_4310_mit_2(fp32 DM_motor_position,fp32 DM_motor_speed,fp32 Kp,fp32 Kd,fp32 torq);
extern void CAN_cmd_4310_mit_3(fp32 DM_motor_position,fp32 DM_motor_speed,fp32 Kp,fp32 Kd,fp32 torq);

extern void CAN_cmd_2006(int16_t motor_2006_current_1,int16_t motor_2006_current_2);
extern void CAN_cmd_3508(int16_t motor_3508_current);
extern motor_measure_t *return_3508_measure(void);
extern motor_measure_t *return_2006_measure(uint8_t i);
extern motor_DM_motor_t *return_4310_measure(uint8_t i);
extern void CAN_cmd_4310_1_enable(void);
extern void CAN_cmd_4310_2_enable(void);
extern void CAN_cmd_4310_3_enable(void);

extern void CAN_cmd_4310_1_disable(void);
extern void CAN_cmd_4310_2_disable(void);
extern void CAN_cmd_4310_3_disable(void);

extern void CAN_board_communicate_can2_0(fp32 arm_position_message[6]);
extern void	CAN_board_communicate_can2_1(fp32 arm_position_message[6]);
extern board_communicate board_message_temp;
extern board_communicate board_message;
extern uint32_t Motor_3508_Timeout_Count;
extern uint8_t  Motor_3508_Timeout_Flag;
#endif

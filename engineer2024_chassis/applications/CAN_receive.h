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

#define CHASSIS_CAN1 hcan1
#define CHASSIS_CAN2 hcan2


extern void CAN_cmd_chassis_all(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4, int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8, int16_t motor9, int16_t motor10);
extern void CAN_board_communicate_can1_0(fp32 board_position_message[6]);
extern void CAN_board_communicate_can1_1(fp32 board_position_message[6], uint16_t mode);

/**
	* 发送底盘总数据，包括4个麦克纳姆轮，两个锁悬挂2006， 一个夹矿2006和一个机械臂升降3508
	*motor1-motor4为麦克纳姆轮
	*motor5-motor6为锁悬挂
	*motor7为夹矿
	*motor8为机械臂升降电机
*/

typedef struct
{
	fp32 target_position[6];
}arm_communicate;

typedef enum
{
	CAN_CHASSIS_ALL_ID = 0x200,
	CAN_M1_ID = 0x201,
	CAN_M2_ID = 0x202,
	CAN_M3_ID = 0x203, 
	CAN_M4_ID = 0x204, //can2上是轮电机， can1上是抬升电机
} can_msg_id_3508_e;

typedef enum
{
	CAN_PIC_ID = 0x202,
	CAN_CLAMP_ID = 0x203,
}can_msg_id_2006_e;

typedef enum
{
	CAN_COMMUNICATE_TX_ID_0 = 0x220,
	CAN_COMMUNICATE_TX_ID_1 = 0x221,
	CAN_COMMUNICATE_RX_ID_0 = 0x230,
	CAN_COMMUNICATE_RX_ID_1 = 0x231,
}can_board_communicate;

typedef struct
{
	uint16_t ecd;
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperate;
	int16_t last_ecd;
	uint16_t count;
} motor_measure_t;


//重置电机ID

	
extern arm_communicate arm_message;
extern arm_communicate arm_message_temp;

extern void CAN_chassis_can1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN_chassis_can2(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN_chassis_can2_add(int16_t motor1, int16_t motor2);
extern const motor_measure_t *get_chassis_motor_point(uint8_t i);
extern const motor_measure_t *get_suspension_motor_point(uint8_t i);
extern const motor_measure_t *get_clamp_motor_point(void);
extern const motor_measure_t *get_picture_motor_point(void);
extern const motor_measure_t *get_lift_motor_point(void);

#endif

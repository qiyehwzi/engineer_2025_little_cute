#include "A_communicate_task.h"
#include "can.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "string.h"
#include "crc_ccitt.h"
#include "arm_control_task.h"
#include "CAN_receive.h"

static void TX_init(void);

Tx_Message_t tx_message_mp;

void A_communicate_task(void const * argument)
{
	while(1)
	{
		TX_init();
		CAN_board_communicate_can1_0(tx_message_mp.target_position);
		osDelay(1);
		CAN_board_communicate_can1_1(tx_message_mp.target_position, tx_message_mp.mode);
		osDelay(1);
	}
}

void TX_init(void)
{
	tx_message_mp.mode = chassis.chassis_mode;	
	tx_message_mp.target_position[0] = arm_control.motor_2_position;
	tx_message_mp.target_position[1] = arm_control.motor_3_position;
	tx_message_mp.target_position[2] = arm_control.motor_4_position;
	tx_message_mp.target_position[3] = arm_control.motor_5_position;
	tx_message_mp.target_position[4] = arm_control.motor_6_position;
	tx_message_mp.target_position[5] = arm_control.motor_7_position;
}

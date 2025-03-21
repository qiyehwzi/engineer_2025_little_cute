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
#include "A_communicate_task.h"
#include "arm_control_task.h"
#include "chassis_task.h"
#include "CAN_receive.h"
#include "cmsis_os.h"
#include "main.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;


#define get_motor_measure(ptr, data)                                    \
{                                                                   		\
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
}

#define get_board_communicate_data_0(ptr, data)                     			\
{                                                                   			\
    (ptr)->target_position[0] = (uint16_t)((data)[0] << 8 | (data)[1]);  	\
    (ptr)->target_position[1] = (uint16_t)((data)[2] << 8 | (data)[3]);   \
    (ptr)->target_position[2] = (uint16_t)((data)[4] << 8 | (data)[5]);   \
    (ptr)->target_position[3] = (uint16_t)((data)[6] << 8 | (data)[7]);  	\
}

#define get_board_communicate_data_1(ptr, data)                     			\
{                                                                   			\
    (ptr)->target_position[4] = (uint16_t)((data)[0] << 8 | (data)[1]);  	\
    (ptr)->target_position[5] = (uint16_t)((data)[2] << 8 | (data)[3]);   \
}		


motor_measure_t CAN1_rx_buffer[4];//0-空，1-图传2006，2-夹矿2006，3-抬升3508
motor_measure_t CAN2_rx_buffer[4];//0~3-底盘4个3508

arm_communicate arm_message;
arm_communicate arm_message_temp;

static CAN_TxHeaderTypeDef  chassis_can1_tx_message;
static uint8_t              chassis_can1_send_data[8];	
static CAN_TxHeaderTypeDef  chassis_can2_tx_message;
static uint8_t              chassis_can2_send_data[8];
static CAN_TxHeaderTypeDef  board_communicate_can1_tx_message;
static uint8_t 							board_communicate_can1_send_data[8];

int float_to_uint(float x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
  float offset = x_min;
  return (int) ((x - offset)*((float)((1<<bits)-1))/span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
		
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		
		if(hcan == &CHASSIS_CAN1)
		{
				switch (rx_header.StdId)
				{
						case CAN_PIC_ID:
						case CAN_CLAMP_ID:
						case CAN_M4_ID:
						{
								static uint8_t i = 0;
								i = rx_header.StdId - CAN_M1_ID;
								get_motor_measure(&CAN1_rx_buffer[i], rx_data)
								if(i == 1)
								{
									if(CAN1_rx_buffer[1].ecd - CAN1_rx_buffer[1].last_ecd > HALF_ECD_RANGE)
									{
										chassis.motor_picture.round_cnt--;
									}
									else if(CAN1_rx_buffer[1].ecd - CAN1_rx_buffer[1].last_ecd < -HALF_ECD_RANGE)
									{
										chassis.motor_picture.round_cnt++;
									}
								}
								if(i == 2)
								{
									if(CAN1_rx_buffer[2].ecd - CAN1_rx_buffer[2].last_ecd > HALF_ECD_RANGE)
									{
										chassis.motor_clamp.round_cnt--;
									}
									else if(CAN1_rx_buffer[2].ecd - CAN1_rx_buffer[2].last_ecd < -HALF_ECD_RANGE)
									{
										chassis.motor_clamp.round_cnt++;
									}
								}
								else if(i == 3)
								{
									if(CAN1_rx_buffer[3].ecd - CAN1_rx_buffer[3].last_ecd > HALF_ECD_RANGE)
									{
										chassis.motor_lift.round_cnt--;
									}
									else if(CAN1_rx_buffer[3].ecd - CAN1_rx_buffer[3].last_ecd < -HALF_ECD_RANGE)
									{
										chassis.motor_lift.round_cnt++;
									}
								}
								break;
						}
            case CAN_COMMUNICATE_RX_ID_0:
						{
								get_board_communicate_data_0(&arm_message_temp, rx_data);
								arm_message.target_position[0] = uint_to_float(arm_message_temp.target_position[0], -50.0f, 50.0f, 16);
								arm_message.target_position[1] = uint_to_float(arm_message_temp.target_position[1], -6.2831852f, 6.2831852f, 16);
								arm_message.target_position[2] = uint_to_float(arm_message_temp.target_position[2], -6.2831852f, 6.2831852f, 16);
								arm_message.target_position[3] = uint_to_float(arm_message_temp.target_position[3], -6.2831852f, 6.2831852f, 16);
								break;
						}
						case CAN_COMMUNICATE_RX_ID_1:
						{
								get_board_communicate_data_1(&arm_message_temp, rx_data);
								arm_message.target_position[4] = uint_to_float(arm_message_temp.target_position[4], -6.2831852f, 6.2831852f, 16);
								arm_message.target_position[5] = uint_to_float(arm_message_temp.target_position[5], -6.2831852f, 6.2831852f, 16);
								break;
						}
						default:
						{
								break;
						}
				}
		}
		else if(hcan == &CHASSIS_CAN2)
		{
				switch (rx_header.StdId)
				{
						case CAN_M1_ID:
						case CAN_M2_ID:
						case CAN_M3_ID:
						case CAN_M4_ID:
						{
								static uint8_t i = 0;
								i = rx_header.StdId - CAN_M1_ID;
								get_motor_measure(&CAN2_rx_buffer[i], rx_data);
								break;
						}
						default:
						{
								break;
						}
				}
		}
}


void CAN_chassis_can1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
		uint32_t send_mail_box;
    chassis_can1_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_can1_tx_message.IDE = CAN_ID_STD;
    chassis_can1_tx_message.RTR = CAN_RTR_DATA;
    chassis_can1_tx_message.DLC = 0x08;
    chassis_can1_send_data[0] = motor1 >> 8;
    chassis_can1_send_data[1] = motor1;
    chassis_can1_send_data[2] = motor2 >> 8;
    chassis_can1_send_data[3] = motor2;
    chassis_can1_send_data[4] = motor3 >> 8;
    chassis_can1_send_data[5] = motor3;
    chassis_can1_send_data[6] = motor4 >> 8;
    chassis_can1_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN1, &chassis_can1_tx_message, chassis_can1_send_data, &send_mail_box);
}


void CAN_chassis_can2(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
		uint32_t send_mail_box;
    chassis_can2_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_can2_tx_message.IDE = CAN_ID_STD;
    chassis_can2_tx_message.RTR = CAN_RTR_DATA;
    chassis_can2_tx_message.DLC = 0x08;
    chassis_can2_send_data[0] = motor1 >> 8;
    chassis_can2_send_data[1] = motor1;
    chassis_can2_send_data[2] = motor2 >> 8;
    chassis_can2_send_data[3] = motor2;
    chassis_can2_send_data[4] = motor3 >> 8;
    chassis_can2_send_data[5] = motor3;
    chassis_can2_send_data[6] = motor4 >> 8;
    chassis_can2_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN2, &chassis_can2_tx_message, chassis_can2_send_data, &send_mail_box);
}


void CAN_board_communicate_can1_0(fp32 board_position_message[6])
{
		uint16_t board_position_message_tmp[6];
	  board_position_message_tmp[0] = float_to_uint(board_position_message[0], -50.0f, 50.0f, 16);
    board_position_message_tmp[1] = float_to_uint(board_position_message[1], -6.2831852f, 6.2831852f, 16);
    board_position_message_tmp[2] = float_to_uint(board_position_message[2], -6.2831852f, 6.2831852f, 16);
    board_position_message_tmp[3] = float_to_uint(board_position_message[3], -6.2831852f, 6.2831852f, 16);
		uint32_t send_mail_box;
    board_communicate_can1_tx_message.StdId = CAN_COMMUNICATE_TX_ID_0;
    board_communicate_can1_tx_message.IDE = CAN_ID_STD;
    board_communicate_can1_tx_message.RTR = CAN_RTR_DATA;
    board_communicate_can1_tx_message.DLC = 0x08;
		board_communicate_can1_send_data[0] = (board_position_message_tmp[0] >> 8);
    board_communicate_can1_send_data[1] = board_position_message_tmp[0];
		board_communicate_can1_send_data[2] = (board_position_message_tmp[1] >> 8);
    board_communicate_can1_send_data[3] = board_position_message_tmp[1];		
		board_communicate_can1_send_data[4] = (board_position_message_tmp[2] >> 8);
		board_communicate_can1_send_data[5] = board_position_message_tmp[2];		
		board_communicate_can1_send_data[6] = (board_position_message_tmp[3] >> 8);
    board_communicate_can1_send_data[7] = board_position_message_tmp[3];

    HAL_CAN_AddTxMessage(&CHASSIS_CAN1, &board_communicate_can1_tx_message, board_communicate_can1_send_data, &send_mail_box);
}

void CAN_board_communicate_can1_1(fp32 board_position_message[6],uint16_t mode)
{
		uint16_t board_position_message_tmp[6];
	  board_position_message_tmp[4] = float_to_uint(board_position_message[4], -6.2831852f, 6.2831852f, 16);
    board_position_message_tmp[5] = float_to_uint(board_position_message[5], -6.2831852f, 6.2831852f, 16);
		uint32_t send_mail_box;
    board_communicate_can1_tx_message.StdId = CAN_COMMUNICATE_TX_ID_1;
    board_communicate_can1_tx_message.IDE = CAN_ID_STD;
    board_communicate_can1_tx_message.RTR = CAN_RTR_DATA;
    board_communicate_can1_tx_message.DLC = 0x08;
		board_communicate_can1_send_data[0] = (board_position_message_tmp[4] >> 8);
    board_communicate_can1_send_data[1] = board_position_message_tmp[4];
		board_communicate_can1_send_data[2] = (board_position_message_tmp[5] >> 8);
    board_communicate_can1_send_data[3] = board_position_message_tmp[5];
		board_communicate_can1_send_data[4] = mode >> 8;
		board_communicate_can1_send_data[5] = mode;
		board_communicate_can1_send_data[6] = (suker_key_flag >> 8);
		board_communicate_can1_send_data[7] = suker_key_flag;
 
    HAL_CAN_AddTxMessage(&CHASSIS_CAN1, &board_communicate_can1_tx_message, board_communicate_can1_send_data, &send_mail_box);
}

const motor_measure_t *get_chassis_motor_point(uint8_t i)
{
  return &CAN2_rx_buffer[(i & 0x03)];
}

const motor_measure_t *get_picture_motor_point()
{
	return &CAN1_rx_buffer[1];
}

const motor_measure_t *get_clamp_motor_point()
{
	return &CAN1_rx_buffer[2];
}

const motor_measure_t *get_lift_motor_point()
{
	return &CAN1_rx_buffer[3];
}

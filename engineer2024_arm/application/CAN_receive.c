#include "arm_control_task.h"
#include "CAN_receive.h"
#include "main.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;


#define get_motor_measure(ptr, data)                                    	\
    {                                                                   	\
        (ptr)->last_ecd = (ptr)->ecd;                                   	\
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            	\
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      	\
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  	\
        (ptr)->temperate = (data)[6];                                   	\
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
    (ptr)->mode = (uint16_t)((data)[4] << 8 | (data)[5]);      						\
    (ptr)->suker_mode = (uint16_t)((data)[6] << 8 | (data)[7]);      						\
}																																					\

int float_to_uint(float x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
  float offset = x_min;
  return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}		

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}


static void get_DM_motor_1_measure(motor_DM_motor_t *DM_motor, uint8_t data[8]);
static void get_DM_motor_2_measure(motor_DM_motor_t *DM_motor, uint8_t data[8]);
static void get_DM_motor_3_measure(motor_DM_motor_t *DM_motor, uint8_t data[8]);

motor_measure_t motor_2006[2];
motor_measure_t motor_3508;
motor_DM_motor_t DM_motor_4310[3];
board_communicate board_message_temp;
board_communicate board_message;

static CAN_TxHeaderTypeDef  arm_tx_message;
static uint8_t              arm_can_send_data[8];
static uint8_t 							DM_can_send_data[8];
static CAN_TxHeaderTypeDef  board_communicate_can2_tx_message;
static uint8_t 							board_communicate_can2_send_data[8];
uint32_t Motor_3508_Timeout_Count;
uint8_t  Motor_3508_Timeout_Flag;

//can1 2006 * 2 DM2��3
//can2 DM1 ���ͨ�š�����3508 
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
		
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		
		if(hcan == &ARM_FOUR_SIX_CAN)
		{
			if(rx_header.StdId == CAN_2006_M1_ID)
			{
					get_motor_measure(&motor_2006[0],rx_data);
					if(motor_2006[0].ecd - motor_2006[0].last_ecd > HALF_ECD_RANGE)
					{
							arm.motor_2006_data[0].round_cnt--;
					}
					else if(motor_2006[0].ecd - motor_2006[0].last_ecd < -HALF_ECD_RANGE)
					{
							arm.motor_2006_data[0].round_cnt++;
					}
			}
			else if(rx_header.StdId == CAN_2006_M2_ID)
			{
					get_motor_measure(&motor_2006[1],rx_data);
					if(motor_2006[1].ecd - motor_2006[1].last_ecd > HALF_ECD_RANGE)
					{
							arm.motor_2006_data[1].round_cnt--;
					}
					else if(motor_2006[1].ecd - motor_2006[1].last_ecd < -HALF_ECD_RANGE)
					{
							arm.motor_2006_data[1].round_cnt++;
					}
			}
			else if(rx_header.StdId == MST_1_ID)
			{
					get_DM_motor_1_measure(&DM_motor_4310[0], rx_data);
			}
			else if(rx_header.StdId == MST_2_ID)
			{
					get_DM_motor_2_measure(&DM_motor_4310[1], rx_data);
			}
			else if(rx_header.StdId == MST_3_ID)
			{
					get_DM_motor_3_measure(&DM_motor_4310[2], rx_data);
			}
		}
		else if(hcan == &ARM_3508_board_CAN)
		{
			if(rx_header.StdId == CAN_COMMUNICATE_RX_ID_0) 
			{
					get_board_communicate_data_0(&board_message_temp, rx_data);
					board_message.target_position[0] = uint_to_float(board_message_temp.target_position[0], -50.0f, 50.0f, 16);
					board_message.target_position[1] = uint_to_float(board_message_temp.target_position[1], -6.2831852f, 6.2831852f, 16);
					board_message.target_position[2] = uint_to_float(board_message_temp.target_position[2], -6.2831852f, 6.2831852f, 16);
					board_message.target_position[3] = uint_to_float(board_message_temp.target_position[3], -6.2831852f, 6.2831852f, 16);
			}
			else if(rx_header.StdId == CAN_COMMUNICATE_RX_ID_1)
			{
					get_board_communicate_data_1(&board_message_temp, rx_data);
					board_message.target_position[4] = uint_to_float(board_message_temp.target_position[4], -6.2831852f, 6.2831852f, 16);
					board_message.target_position[5] = uint_to_float(board_message_temp.target_position[5], -6.2831852f, 6.2831852f, 16);
					board_message.mode = board_message_temp.mode;
					board_message.suker_mode = board_message_temp.suker_mode;
			}
			else if(rx_header.StdId == CAN_3508_M1_ID)
			{
					Motor_3508_Timeout_Count = 0;
					Motor_3508_Timeout_Flag = 0;
					get_motor_measure(&motor_3508, rx_data);
					//�Ƴ����ĽǶ�ʵ�������෴��
					if(motor_3508.ecd - motor_3508.last_ecd > HALF_ECD_RANGE)
					{
							arm.motor_3508_data.round_cnt--;
					}
					else if(motor_3508.ecd - motor_3508.last_ecd < -HALF_ECD_RANGE)
					{
							arm.motor_3508_data.round_cnt++;
					}
			}
		}
}


void CAN_cmd_2006(int16_t motor_2006_current_1,int16_t motor_2006_current_2)
{
		uint32_t send_mail_box;
    arm_tx_message.StdId = CAN_ARM_CMD_ID;
    arm_tx_message.IDE = CAN_ID_STD;
    arm_tx_message.RTR = CAN_RTR_DATA;
    arm_tx_message.DLC = 0x08;
    arm_can_send_data[0] = 0;
    arm_can_send_data[1] = 0;
    arm_can_send_data[2] = motor_2006_current_1 >> 8;
    arm_can_send_data[3] = motor_2006_current_1;
    arm_can_send_data[4] = motor_2006_current_2 >> 8;
    arm_can_send_data[5] = motor_2006_current_2;
    arm_can_send_data[6] = 0;
    arm_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&ARM_FOUR_SIX_CAN, &arm_tx_message, arm_can_send_data, &send_mail_box);
}
void CAN_cmd_3508(int16_t motor_3508_current)
{
		uint32_t send_mail_box;
    arm_tx_message.StdId = CAN_ARM_CMD_ID;
    arm_tx_message.IDE = CAN_ID_STD;
    arm_tx_message.RTR = CAN_RTR_DATA;
    arm_tx_message.DLC = 0x08;
    arm_can_send_data[0] = motor_3508_current >> 8;
    arm_can_send_data[1] = motor_3508_current;
    arm_can_send_data[2] = 0;
    arm_can_send_data[3] = 0;
    arm_can_send_data[4] = 0;
    arm_can_send_data[5] = 0;
    arm_can_send_data[6] = 0;
    arm_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&ARM_3508_board_CAN, &arm_tx_message, arm_can_send_data, &send_mail_box);
}
void get_DM_motor_1_measure(motor_DM_motor_t *DM_motor, uint8_t data[8])
{
	uint16_t int_position,int_speed,int_T;

	int_position = (uint16_t)((data[1]<<8)|data[2]);
	int_speed = (uint16_t)((data[3]<<4)|((data[4]&0xF0)>>4));
  int_T = (uint16_t)(((data[4]&0x0F)<<8)|data[5]);
	
	DM_motor->motor_err = data[0] & 0xF0;
	DM_motor->motor_tx_id = data[0] & 0x0F;
	DM_motor->motor_enabled = data[0]>>4;
	DM_motor->motor_position = ((fp32)int_position)*3.141593f*2.0f/((fp32)((1<<16)-1))-3.141593f;
	DM_motor->motor_speed = ((fp32)int_speed)*20.0f*2.0f/((fp32)((1<<12)-1))-20.0f;
	DM_motor->motor_T = ((fp32)int_T)*10.0f*2.0f/((fp32)((1<<12)-1))-10.0f;

}

void get_DM_motor_2_measure(motor_DM_motor_t *DM_motor, uint8_t data[8])
{
	uint16_t int_position,int_speed,int_T;
	
	int_position = (uint16_t)((data[1]<<8)|data[2]);
	int_speed = (uint16_t)((data[3]<<4)|((data[4]&0xF0)>>4));
	int_T = (uint16_t)(((data[4]&0x0F)<<8)|data[5]);
	
	DM_motor->motor_err = data[0] & 0xF0;
	DM_motor->motor_tx_id = data[0] & 0x0F;
	DM_motor->motor_enabled = data[0]>>4;
	DM_motor->motor_position = ((fp32)int_position)*3.141593f*2.0f/((fp32)((1<<16)-1))-3.141593f;
	DM_motor->motor_speed = ((fp32)int_speed)*20.0f*2.0f/((fp32)((1<<12)-1))-20.0f;
	DM_motor->motor_T = ((fp32)int_T)*10.0f*2.0f/((fp32)((1<<12)-1))-10.0f;
}

void get_DM_motor_3_measure(motor_DM_motor_t *DM_motor, uint8_t data[8])
{
	uint16_t int_position,int_speed,int_T;
	
	int_position = (uint16_t)((data[1]<<8)|data[2]);
	int_speed = (uint16_t)((data[3]<<4)|((data[4]&0xF0)>>4));
	int_T = (uint16_t)(((data[4]&0x0F)<<8)|data[5]);
	
	DM_motor->motor_err = data[0] & 0xF0;
	DM_motor->motor_tx_id = data[0] & 0x0F;
	DM_motor->motor_enabled = data[0]>>4;
	DM_motor->motor_position = ((fp32)int_position)*3.141593f*2.0f/((fp32)((1<<16)-1))-3.141593f;
	DM_motor->motor_speed = ((fp32)int_speed)*20.0f*2.0f/((fp32)((1<<12)-1))-20.0f;
	DM_motor->motor_T = ((fp32)int_T)*10.0f*2.0f/((fp32)((1<<12)-1))-10.0f;
}


void CAN_cmd_4310_1_enable(void)
{
	uint32_t send_mail_box;
  arm_tx_message.StdId = DM_4310_M1_TX_ID;
  arm_tx_message.IDE = CAN_ID_STD;
  arm_tx_message.RTR = CAN_RTR_DATA;
	arm_tx_message.DLC = 0x08;
	DM_can_send_data[0] = 0xFF;
	DM_can_send_data[1] = 0xFF;
	DM_can_send_data[2] = 0xFF;
	DM_can_send_data[3] = 0xFF;
	DM_can_send_data[4] = 0xFF;
	DM_can_send_data[5] = 0xFF;
	DM_can_send_data[6] = 0xFF;
	DM_can_send_data[7] = 0xFC;
	HAL_CAN_AddTxMessage(&ARM_FOUR_SIX_CAN, &arm_tx_message, DM_can_send_data, &send_mail_box);
}


void CAN_cmd_4310_1_disable(void)
{
	uint32_t send_mail_box;
  arm_tx_message.StdId = DM_4310_M1_TX_ID;
  arm_tx_message.IDE = CAN_ID_STD;
  arm_tx_message.RTR = CAN_RTR_DATA;
	arm_tx_message.DLC = 0x08;
	DM_can_send_data[0] = 0xFF;
	DM_can_send_data[1] = 0xFF;
	DM_can_send_data[2] = 0xFF;
	DM_can_send_data[3] = 0xFF;
	DM_can_send_data[4] = 0xFF;
	DM_can_send_data[5] = 0xFF;
	DM_can_send_data[6] = 0xFF;
	DM_can_send_data[7] = 0xFD;
	HAL_CAN_AddTxMessage(&ARM_FOUR_SIX_CAN, &arm_tx_message, DM_can_send_data, &send_mail_box);
}


void CAN_cmd_4310_1(fp32 DM_motor_position, fp32 DM_motor_speed)
{
	uint8_t *pbuf,*vbuf;
	pbuf = (uint8_t*) &DM_motor_position;
	vbuf = (uint8_t*) &DM_motor_speed;
	uint32_t send_mail_box;
  arm_tx_message.StdId = DM_4310_M1_TX_ID;
  arm_tx_message.IDE = CAN_ID_STD;
  arm_tx_message.RTR = CAN_RTR_DATA;
	arm_tx_message.DLC = 0x08;
	DM_can_send_data[0] = *pbuf;
	DM_can_send_data[1] = *(pbuf+1);
	DM_can_send_data[2] = *(pbuf+2);
	DM_can_send_data[3] = *(pbuf+3);
	DM_can_send_data[4] = *vbuf;
	DM_can_send_data[5] = *(vbuf+1);
	DM_can_send_data[6] = *(vbuf+2);
	DM_can_send_data[7] = *(vbuf+3);
	HAL_CAN_AddTxMessage(&ARM_FOUR_SIX_CAN, &arm_tx_message, DM_can_send_data, &send_mail_box);
}


void CAN_cmd_4310_mit_1(fp32 DM_motor_position,fp32 DM_motor_speed,fp32 Kp,fp32 Kd,fp32 torq)
{
	  uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	  pos_tmp = float_to_uint(DM_motor_position, -3.141593f, 3.141593f, 16);
    vel_tmp = float_to_uint(DM_motor_speed, -20.0f, 20.0f, 12);
    kp_tmp  = float_to_uint(Kp, 0, 500.0f, 12);
    kd_tmp  = float_to_uint(Kd, 0, 5.0f, 12);
    tor_tmp = float_to_uint(torq, -10.0f, 10.0f, 12);
    uint32_t send_mail_box;
    arm_tx_message.StdId = DM_4310_M1_TX_ID;
    arm_tx_message.IDE = CAN_ID_STD;
    arm_tx_message.RTR = CAN_RTR_DATA;
    arm_tx_message.DLC = 0x08;
    DM_can_send_data[0] = (pos_tmp >> 8);
    DM_can_send_data[1] =  pos_tmp;
    DM_can_send_data[2] = (vel_tmp >> 4);
    DM_can_send_data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    DM_can_send_data[4] = kp_tmp;
    DM_can_send_data[5] = (kd_tmp >> 4);
    DM_can_send_data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    DM_can_send_data[7] = tor_tmp;
    HAL_CAN_AddTxMessage(&ARM_FOUR_SIX_CAN, &arm_tx_message, DM_can_send_data, &send_mail_box);
}	


void CAN_cmd_4310_2_enable(void)
{
	uint32_t send_mail_box;
  arm_tx_message.StdId = DM_4310_M2_TX_ID;
  arm_tx_message.IDE = CAN_ID_STD;
  arm_tx_message.RTR = CAN_RTR_DATA;
	arm_tx_message.DLC = 0x08;
	DM_can_send_data[0] = 0xFF;
	DM_can_send_data[1] = 0xFF;
	DM_can_send_data[2] = 0xFF;
	DM_can_send_data[3] = 0xFF;
	DM_can_send_data[4] = 0xFF;
	DM_can_send_data[5] = 0xFF;
	DM_can_send_data[6] = 0xFF;
	DM_can_send_data[7] = 0xFC;
	HAL_CAN_AddTxMessage(&ARM_FOUR_SIX_CAN, &arm_tx_message, DM_can_send_data, &send_mail_box);
}

void CAN_cmd_4310_2_disable(void)
{
	uint32_t send_mail_box;
  arm_tx_message.StdId = DM_4310_M2_TX_ID;
  arm_tx_message.IDE = CAN_ID_STD;
  arm_tx_message.RTR = CAN_RTR_DATA;
	arm_tx_message.DLC = 0x08;
	DM_can_send_data[0] = 0xFF;
	DM_can_send_data[1] = 0xFF;
	DM_can_send_data[2] = 0xFF;
	DM_can_send_data[3] = 0xFF;
	DM_can_send_data[4] = 0xFF;
	DM_can_send_data[5] = 0xFF;
	DM_can_send_data[6] = 0xFF;
	DM_can_send_data[7] = 0xFD;
	HAL_CAN_AddTxMessage(&ARM_FOUR_SIX_CAN, &arm_tx_message, DM_can_send_data, &send_mail_box);
}

void CAN_cmd_4310_2(fp32 DM_motor_position, fp32 DM_motor_speed)
{
	uint8_t *pbuf,*vbuf;
	pbuf = (uint8_t*) &DM_motor_position;
	vbuf = (uint8_t*) &DM_motor_speed;
	uint32_t send_mail_box;
  arm_tx_message.StdId = DM_4310_M2_TX_ID;
  arm_tx_message.IDE = CAN_ID_STD;
  arm_tx_message.RTR = CAN_RTR_DATA;
	arm_tx_message.DLC = 0x08;
	DM_can_send_data[0] = *pbuf;
	DM_can_send_data[1] = *(pbuf+1);
	DM_can_send_data[2] = *(pbuf+2);
	DM_can_send_data[3] = *(pbuf+3);
	DM_can_send_data[4] = *vbuf;
	DM_can_send_data[5] = *(vbuf+1);
	DM_can_send_data[6] = *(vbuf+2);
	DM_can_send_data[7] = *(vbuf+3);
	HAL_CAN_AddTxMessage(&ARM_FOUR_SIX_CAN, &arm_tx_message, DM_can_send_data, &send_mail_box);
}

void CAN_cmd_4310_mit_2(fp32 DM_motor_position,fp32 DM_motor_speed,fp32 Kp,fp32 Kd,fp32 torq)
{
	  uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	  pos_tmp = float_to_uint(DM_motor_position, -3.141593f, 3.141593f, 16);
    vel_tmp = float_to_uint(DM_motor_speed, -20.0f, 20.0f, 12);
    kp_tmp  = float_to_uint(Kp, 0, 500.0f, 12);
    kd_tmp  = float_to_uint(Kd, 0, 5.0f, 12);
    tor_tmp = float_to_uint(torq, -10.0f, 10.0f, 12);
    uint32_t send_mail_box;
    arm_tx_message.StdId = DM_4310_M2_TX_ID;
    arm_tx_message.IDE = CAN_ID_STD;
    arm_tx_message.RTR = CAN_RTR_DATA;
    arm_tx_message.DLC = 0x08;
    DM_can_send_data[0] = (pos_tmp >> 8);
    DM_can_send_data[1] =  pos_tmp;
    DM_can_send_data[2] = (vel_tmp >> 4);
    DM_can_send_data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    DM_can_send_data[4] = kp_tmp;
    DM_can_send_data[5] = (kd_tmp >> 4);
    DM_can_send_data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    DM_can_send_data[7] = tor_tmp;
    HAL_CAN_AddTxMessage(&ARM_FOUR_SIX_CAN, &arm_tx_message, DM_can_send_data, &send_mail_box);
}	

void CAN_cmd_4310_3_enable(void)
{
	uint32_t send_mail_box;
  arm_tx_message.StdId = DM_4310_M3_TX_ID;
  arm_tx_message.IDE = CAN_ID_STD;
  arm_tx_message.RTR = CAN_RTR_DATA;
	arm_tx_message.DLC = 0x08;
	DM_can_send_data[0] = 0xFF;
	DM_can_send_data[1] = 0xFF;
	DM_can_send_data[2] = 0xFF;
	DM_can_send_data[3] = 0xFF;
	DM_can_send_data[4] = 0xFF;
	DM_can_send_data[5] = 0xFF;
	DM_can_send_data[6] = 0xFF;
	DM_can_send_data[7] = 0xFC;
	HAL_CAN_AddTxMessage(&ARM_FOUR_SIX_CAN, &arm_tx_message, DM_can_send_data, &send_mail_box);
}

void CAN_cmd_4310_3_disable(void)
{
	uint32_t send_mail_box;
  arm_tx_message.StdId = DM_4310_M3_TX_ID;
  arm_tx_message.IDE = CAN_ID_STD;
  arm_tx_message.RTR = CAN_RTR_DATA;
	arm_tx_message.DLC = 0x08;
	DM_can_send_data[0] = 0xFF;
	DM_can_send_data[1] = 0xFF;
	DM_can_send_data[2] = 0xFF;
	DM_can_send_data[3] = 0xFF;
	DM_can_send_data[4] = 0xFF;
	DM_can_send_data[5] = 0xFF;
	DM_can_send_data[6] = 0xFF;
	DM_can_send_data[7] = 0xFD;
	HAL_CAN_AddTxMessage(&ARM_FOUR_SIX_CAN, &arm_tx_message, DM_can_send_data, &send_mail_box);
}

void CAN_cmd_4310_3(fp32 DM_motor_position, fp32 DM_motor_speed)
{
	uint8_t *pbuf,*vbuf;
	pbuf = (uint8_t*) &DM_motor_position;
	vbuf = (uint8_t*) &DM_motor_speed;
	uint32_t send_mail_box;
  arm_tx_message.StdId = DM_4310_M3_TX_ID;
  arm_tx_message.IDE = CAN_ID_STD;
  arm_tx_message.RTR = CAN_RTR_DATA;
	arm_tx_message.DLC = 0x08;
	DM_can_send_data[0] = *pbuf;
	DM_can_send_data[1] = *(pbuf+1);
	DM_can_send_data[2] = *(pbuf+2);
	DM_can_send_data[3] = *(pbuf+3);
	DM_can_send_data[4] = *vbuf;
	DM_can_send_data[5] = *(vbuf+1);
	DM_can_send_data[6] = *(vbuf+2);
	DM_can_send_data[7] = *(vbuf+3);
	HAL_CAN_AddTxMessage(&ARM_FOUR_SIX_CAN, &arm_tx_message, DM_can_send_data, &send_mail_box);
}

void CAN_cmd_4310_mit_3(fp32 DM_motor_position,fp32 DM_motor_speed,fp32 Kp,fp32 Kd,fp32 torq)
{
	  uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	  pos_tmp = float_to_uint(DM_motor_position, -3.141593f, 3.141593f, 16);
    vel_tmp = float_to_uint(DM_motor_speed, -20.0f, 20.0f, 12);
    kp_tmp  = float_to_uint(Kp, 0, 500.0f, 12);
    kd_tmp  = float_to_uint(Kd, 0, 5.0f, 12);
    tor_tmp = float_to_uint(torq, -10.0f, 10.0f, 12);
    uint32_t send_mail_box;
    arm_tx_message.StdId = DM_4310_M3_TX_ID;
    arm_tx_message.IDE = CAN_ID_STD;
    arm_tx_message.RTR = CAN_RTR_DATA;
    arm_tx_message.DLC = 0x08;
    DM_can_send_data[0] = (pos_tmp >> 8);
    DM_can_send_data[1] =  pos_tmp;
    DM_can_send_data[2] = (vel_tmp >> 4);
    DM_can_send_data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    DM_can_send_data[4] = kp_tmp;
    DM_can_send_data[5] = (kd_tmp >> 4);
    DM_can_send_data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    DM_can_send_data[7] = tor_tmp;
    HAL_CAN_AddTxMessage(&ARM_FOUR_SIX_CAN, &arm_tx_message, DM_can_send_data, &send_mail_box);
}	

void CAN_board_communicate_can2_0(fp32 arm_position_message[6])
{
		uint16_t board_position_message_tmp[6];
	  board_position_message_tmp[0] = float_to_uint(arm_position_message[0], -50.0f, 50.0f, 16);
    board_position_message_tmp[1] = float_to_uint(arm_position_message[1], -6.2831852f, 6.2831852f, 16);
    board_position_message_tmp[2] = float_to_uint(arm_position_message[2], -6.2831852f, 6.2831852f, 16);
    board_position_message_tmp[3] = float_to_uint(arm_position_message[3], -6.2831852f, 6.2831852f, 16);
		uint32_t send_mail_box;
    board_communicate_can2_tx_message.StdId = CAN_COMMUNICATE_TX_ID_0;
    board_communicate_can2_tx_message.IDE = CAN_ID_STD;
    board_communicate_can2_tx_message.RTR = CAN_RTR_DATA;
    board_communicate_can2_tx_message.DLC = 0x08;
		board_communicate_can2_send_data[0] = (board_position_message_tmp[0] >> 8);
    board_communicate_can2_send_data[1] = board_position_message_tmp[0];
		board_communicate_can2_send_data[2] = (board_position_message_tmp[1] >> 8);
    board_communicate_can2_send_data[3] = board_position_message_tmp[1];
		board_communicate_can2_send_data[4] = (board_position_message_tmp[2] >> 8);
		board_communicate_can2_send_data[5] = board_position_message_tmp[2];
		board_communicate_can2_send_data[6] = (board_position_message_tmp[3] >> 8);
    board_communicate_can2_send_data[7] = board_position_message_tmp[3];
	
		HAL_CAN_AddTxMessage(&ARM_3508_board_CAN, &board_communicate_can2_tx_message, board_communicate_can2_send_data, &send_mail_box);
}

void CAN_board_communicate_can2_1(fp32 arm_position_message[6])
{
		uint16_t board_position_message_tmp[6];
	  board_position_message_tmp[4] = float_to_uint(arm_position_message[4], -6.2831852f, 6.2831852f, 16);
    board_position_message_tmp[5] = float_to_uint(arm_position_message[5], -6.2831852f, 6.2831852f, 16);
		uint32_t send_mail_box;
    board_communicate_can2_tx_message.StdId = CAN_COMMUNICATE_TX_ID_1;
    board_communicate_can2_tx_message.IDE = CAN_ID_STD;
    board_communicate_can2_tx_message.RTR = CAN_RTR_DATA;
    board_communicate_can2_tx_message.DLC = 0x08;
		board_communicate_can2_send_data[0] = (board_position_message_tmp[4] >> 8);
    board_communicate_can2_send_data[1] = board_position_message_tmp[4];
		board_communicate_can2_send_data[2] = (board_position_message_tmp[5] >> 8);
    board_communicate_can2_send_data[3] = board_position_message_tmp[5];
		board_communicate_can2_send_data[4]	= (uint16_t)0;
		board_communicate_can2_send_data[5]	= (uint16_t)0;
		board_communicate_can2_send_data[6]	= (uint16_t)0;
		board_communicate_can2_send_data[7]	= (uint16_t)0;
	
		HAL_CAN_AddTxMessage(&ARM_3508_board_CAN, &board_communicate_can2_tx_message, board_communicate_can2_send_data, &send_mail_box);
}

motor_measure_t *return_2006_measure(uint8_t i)
{
	return &motor_2006[i];
}

motor_DM_motor_t *return_4310_measure(uint8_t i)
{
	return &DM_motor_4310[i];
}

motor_measure_t *return_3508_measure(void)
{
	return &motor_3508;
}

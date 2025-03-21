#ifndef A_COMMUNICATE_TASK_H
#define A_COMMUNICATE_TASK_H

#include "struct_typedef.h"
//#include "bsp_rs485.h"//���ͨ��Э��

#define COMMUNICATE_HEADER1  0xFE
#define COMMUNICATE_HEADER2  0xFE
#define COMMUNICATE_TAIL 0xA3
#define COMMUNICATE_TX_LEN 26
#define COMMUNICATE_RX_LEN 17


//typedef struct 
//{  
//	//�� 4���ֽ�һ������ ����Ȼ�����������
//	 uint8_t head[2];
//}Head_t;

typedef struct
{
	//�� 4���ֽ�һ������ ����Ȼ�����������
	 fp32 motor_position[6];
}Rx_Message_t;

typedef struct
{
	//�� 4���ֽ�һ������ ����Ȼ�����������
//	 Head_t Head;
	 uint16_t mode;
	 fp32 target_position[6];
	 //fp32 max_speed[5];
}Tx_Message_t;

extern fp32 rx_motor_position[5];  //��ǰ��������Ƕ�


extern void A_communicate_task(void const * argument);

extern Tx_Message_t tx_message_mp;

#endif


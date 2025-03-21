#ifndef A_COMMUNICATE_TASK_H
#define A_COMMUNICATE_TASK_H

#include "struct_typedef.h"


#define COMMUNICATE_HEADER1  0xFE
#define COMMUNICATE_HEADER2  0xFE
#define COMMUNICATE_TAIL 0xA3
#define COMMUNICATE_TX_LEN 28
#define COMMUNICATE_RX_LEN 30

typedef struct 
{  
	//�� 4���ֽ�һ������ ����Ȼ�����������
	 fp32 motor_position[6];
}Tx_Message_t;

typedef struct 
{  
	//�� 4���ֽ�һ������ ����Ȼ�����������
	 uint16_t mode;
	 fp32 target_position[6];
}Rx_Message_t;


extern void A_communicate_task(void const * argument);
extern Rx_Message_t rx_message_mp; 
extern Tx_Message_t tx_message_mp;
extern uint16_t last_mode;
#endif




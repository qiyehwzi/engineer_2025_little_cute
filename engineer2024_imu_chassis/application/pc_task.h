#ifndef PC_TASK_H
#define PC_TASK_H
#include "struct_typedef.h"
#include "usart.h"
#define SOF_ADDR 	 0			//֡ͷ����ʽ�ֽ�ƫ����

#define FRAME_HEADER 		0xA5//֡ͷ����ʽ
#define LEN_FRAME_HEADER 	2	//֡ͷ����
#define	LEN_TX_DATA 		11	//�������ݶγ���
#define	LEN_RX_DATA 		18	//�������ݶγ���
#define	LEN_FRAME_TAILER 	2	//֡βCRC16
#define	LEN_TX_PACKET		15	//���Ͱ���������
#define	LEN_RX_PACKET	  13	//���հ���������

//��󻺳��ֽ���
#define USART1_MAX_RECV_LEN 100

#define PC_RX_BUF_NUM   26u
#define PC_RX_BUF_LENGTH 13u
//֡ͷ�ṹ��
typedef __packed struct
{
	uint8_t  	sof;			// ֡ͷ����ʽ
	uint8_t  	crc8;			// CRC8У����
} frame_header_t;
//֡β�ṹ��
typedef __packed struct 
{
	uint16_t crc16;				// CRC16У����
} frame_tailer_t;
//�������ݽṹ��
typedef __packed struct 
{
	float curr_yaw;         	//��ǰ��̨yaw�Ƕ�
	float curr_pitch;       	//��ǰ��̨pitch��
	uint8_t state;          	//��ǰ״̬������-���-С��
	uint8_t pc_state;        //����ģʽ
	uint8_t enemy_color;    	//�з���ɫ
}tx_data_t;
//�������ݽṹ��
typedef __packed struct 
{
	uint8_t fire;            	//����ָ��
	float shoot_yaw;       		//����ƫת��
	float shoot_pitch;
}rx_data_t;
//���Ͱ��ṹ��
typedef __packed struct 
{
	frame_header_t 		frame_header;	
	tx_data_t	  		   tx_data;	
	frame_tailer_t 		frame_tailer;	
} send_msg_t;
//���ܰ��ṹ��
typedef __packed struct 
{
	frame_header_t	 	frame_header;	
	rx_data_t	  		  rx_data;	
	frame_tailer_t 		frame_tailer;	
} receive_msg_t;

extern float before_yaw;
extern float before_pitch;
extern receive_msg_t pc_receive_msg;
extern void pc_task(void const *argu);
extern void USART6_RX_IRQHandler(void);
const rx_data_t *get_pc_rx_point(void);
#endif

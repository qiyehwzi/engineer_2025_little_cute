#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "usart.h"

//��󻺳��ֽ���
#define USART2_MAX_RECV_LEN 100

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

typedef  __packed struct
{
    float yaw_angle;
}rx_data_t;

//���Ͱ��ṹ��
typedef __packed struct 
{
	frame_header_t 		frame_header;	
	rx_data_t	  			rx_data;	
	frame_tailer_t 		frame_tailer;	
} receive_msg_t;


extern receive_msg_t imu_rx_data;

//���ջ���
extern uint8_t USART2_RX_BUF[USART2_MAX_RECV_LEN];
//��ǰ���峤��
extern uint16_t USART2_RX_STA;

extern void usart2_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void uart5_init(void);
extern void uart6_init(void);
extern void uart7_init(void);
extern void uart8_init(void);


#endif

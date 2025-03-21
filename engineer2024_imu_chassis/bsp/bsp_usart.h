#ifndef __BSP_USART_H
#define __BSP_USART_H
#include "struct_typedef.h"

//��󻺳��ֽ���
#define USART6_MAX_RECV_LEN 100
//���ջ���
extern uint8_t USART6_RX_BUF[USART6_MAX_RECV_LEN];
//��ǰ���峤��
extern uint16_t USART6_RX_STA;



//ң����
extern void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void RC_unable(void);
extern void RC_restart(uint16_t dma_buf_num);
//PC
extern void usart1_tx_dma_init(void);
extern void usart1_tx_dma_enable(uint8_t *data, uint16_t len);
//extern void usart1_idle_init(void);
extern void usart_printf(const char *fmt,...);

//PC
extern void usart6_tx_dma_init(void);
extern void usart6_idle_init(void);
extern void usart6_tx_dma_enable(uint8_t *data, uint16_t len);

extern void usart1_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
#endif

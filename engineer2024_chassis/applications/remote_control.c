/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      ң��������ң������ͨ������SBUS��Э�鴫�䣬����DMA���䷽ʽ��ԼCPU
  *             ��Դ�����ô��ڿ����ж�������������ͬʱ�ṩһЩ��������DMA������
  *             �ķ�ʽ��֤�Ȳ�ε��ȶ��ԡ�
  * @note       ��������ͨ�������ж�����������freeRTOS����
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-01-2019     RM              1. ���
  *   
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "remote_control.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);


RC_ctrl_t rc_ctrl;
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];


void remote_control_init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}


const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}



void usart1_IRQHandler(void)
{
    if(huart1.Instance->SR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if(USART1->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;


            hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;

            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            }
        }
        else
        {
            __HAL_DMA_DISABLE(&hdma_usart1_rx);


            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
					
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
            }
        }
    }
}


static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }
		
		uint16_t key_information;
		key_information = (sbus_buf[14] | (sbus_buf[15] << 8));

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
//    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
		rc_ctrl->key.W  = (key_information >> 0) & 1;  //����W
		rc_ctrl->key.S  = (key_information >> 1) & 1;  //����S
		rc_ctrl->key.A  = (key_information >> 2) & 1;  //����A
		rc_ctrl->key.D  = (key_information >> 3) & 1;  //����D
		rc_ctrl->key.SHIFT  = (key_information >> 4) & 1;  //����SHIFT
		rc_ctrl->key.CTRL  = (key_information >> 5) & 1;  //����CTRL
		rc_ctrl->key.Q  = (key_information >> 6) & 1;  //����Q
		rc_ctrl->key.E  = (key_information >> 7) & 1;  //����E
		rc_ctrl->key.R  = (key_information >> 8) & 1;  //����R
		rc_ctrl->key.F  = (key_information >> 9) & 1;  //����F
		rc_ctrl->key.G  = (key_information >> 10) & 1;  //����G
		rc_ctrl->key.Z  = (key_information >> 11) & 1;  //����Z
		rc_ctrl->key.X  = (key_information >> 12) & 1;  //����X
		rc_ctrl->key.C  = (key_information >> 13) & 1;  //����C
		rc_ctrl->key.V  = (key_information >> 14) & 1;  //����V
		rc_ctrl->key.B  = (key_information >> 15) & 1;  //����B
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

#include "usart_communicate.h"
#include "usart.h"
#include "string.h"
#include "motor_task.h"
#include "6dof_kinematic.h"
#include "ins_task.h"


send_msg_t imu_send_msg;
uint8_t tx_buffer[8];
const fp32 *gimbal_INT_angle_point;
const fp32 *gimbal_INT_gyro_point;

void usart_communicate_task(void const * argument)
{

    //����������ָ���ȡ
   gimbal_INT_angle_point = get_angle_data_point();
	
	while(1)
	{
		imu_send_msg.frame_header.sof 		= 0xA5;
		imu_send_msg.tx_data.yaw_angle    = *(gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
		
		append_crc8_check_sum(&imu_send_msg.frame_header.sof, 2);		
		append_crc16_check_sum(&imu_send_msg.frame_header.sof, 8);
		memcpy(tx_buffer,&imu_send_msg, 8);

		HAL_UART_Transmit_DMA(&huart6,tx_buffer,8);
		osDelay(2);		
	}
}

void USART6_RX_IRQHandler(void)
{
//    if(huart6.Instance->SR & UART_FLAG_RXNE)//���յ�����
//    {
//        __HAL_UART_CLEAR_PEFLAG(&huart6);
//    }
//    else if(USART6->SR & UART_FLAG_IDLE)
//    {
//        static uint16_t this_time_rx_len = 0;

//        __HAL_UART_CLEAR_PEFLAG(&huart6);

//        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
//        {
//            /* Current memory buffer used is Memory 0 */

//            //disable DMA
//            //ʧЧDMA
//            __HAL_DMA_DISABLE(&hdma_usart6_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
//            this_time_rx_len = PC_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;

//            //reset set_data_lenght
//            //�����趨���ݳ���
//            hdma_usart6_rx.Instance->NDTR = PC_RX_BUF_NUM;

//            //set memory buffer 1
//            //�趨������1
//            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;

//            //enable DMA
//            //ʹ��DMA
//            __HAL_DMA_ENABLE(&hdma_usart6_rx);

//            if(this_time_rx_len == PC_RX_BUF_LENGTH)
//            {
//                data_solve(&pc_receive_msg,PC_rx_buf[0]);
//            }
//        }
//        else
//        {
//            /* Current memory buffer used is Memory 1 */
//            //disable DMA
//            //ʧЧDMA
//            __HAL_DMA_DISABLE(&hdma_usart6_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
//            this_time_rx_len = PC_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;

//            //reset set_data_lenght
//            //�����趨���ݳ���
//            hdma_usart6_rx.Instance->NDTR =PC_RX_BUF_NUM;

//            //set memory buffer 0
//            //�趨������0
//            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

//            //enable DMA
//            //ʹ��DMA
//            __HAL_DMA_ENABLE(&hdma_usart6_rx);

//            if(this_time_rx_len == PC_RX_BUF_LENGTH)
//            {
//                //����ң��������
//                data_solve(&pc_receive_msg,PC_rx_buf[1]);

//            }
//        }
//    }
}



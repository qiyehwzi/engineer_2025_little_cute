#include "pc_task.h"
#include "cmsis_os.h"
#include "shoot.h"
#include "gimbal_task.h"
#include "can_communicate.h"
#include "protocol.h"
#include "bsp_usart.h"

send_msg_t pc_send_msg;
receive_msg_t pc_receive_msg;
//����ԭʼ���ݣ�Ϊ12���ֽڣ�����24���ֽڳ��ȣ���ֹDMA����Խ��
//static uint8_t PC_rx_buf[2][PC_RX_BUF_NUM];
uint8_t PC_SEND_BUF[LEN_TX_PACKET + 1];
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
void pc_task(void const *argu)
{
	 // usart6_init(PC_rx_buf[0],PC_rx_buf[1],PC_RX_BUF_NUM);
	 usart6_idle_init();
   usart6_tx_dma_init();
	  vTaskDelay(100);
	//��ȡ��ǰϵͳʱ�䣬������ʱ��
    uint32_t mode_wake_time = osKernelSysTick();
    while(1)
    {
        //�Ը����ݸ�ֵ
        pc_send_msg.frame_header.sof = FRAME_HEADER; //֡ͷ
        pc_send_msg.tx_data.curr_yaw = gimbal_control.gimbal_yaw_motor.absolute_angle;     //��ǰ��̨yaw�Ƕ�
        pc_send_msg.tx_data.curr_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle; //��ǰ��̨pitch�Ƕ�
        pc_send_msg.tx_data.state = '0' + gimbal_control.gimbal_remote_data->key_b.key_switch; //��ǰ״̬��0����-1���
        if(gimbal_control.gimbal_mode == GIMBAL_MODE_PC)
		    pc_send_msg.tx_data.pc_state = 1;
		    else
		    pc_send_msg.tx_data.pc_state = 0;			
			  if(color_flag == 0) pc_send_msg.tx_data.enemy_color = 0;
        if(color_flag == 1) pc_send_msg.tx_data.enemy_color = 1;
        //��֡ͷ��֡β����CRCУ�飬����CRC8У�麯�����Բ�����
        append_crc8_check_sum(&pc_send_msg.frame_header.sof, LEN_FRAME_HEADER);
        append_crc16_check_sum(&pc_send_msg.frame_header.sof, LEN_TX_PACKET);
        //��������
        memcpy(PC_SEND_BUF, &pc_send_msg, sizeof(pc_send_msg));
        PC_SEND_BUF[LEN_TX_PACKET]= '\n';
        usart6_tx_dma_enable(PC_SEND_BUF,LEN_TX_PACKET+1);
        osDelayUntil(&mode_wake_time, 7);
    }
}
float before_yaw = 0, before_pitch = 0;int for_pitch = 0;
static void data_solve(receive_msg_t *pc_receive_msg, uint8_t *rx_data)
{
	  static uint8_t res = 0;
	if(rx_data[SOF_ADDR] == FRAME_HEADER) 
	{	
		//֡βCRC16У��
		res = verify_crc16_check_sum(rx_data, LEN_RX_PACKET);
		if(res == 1) 
		{
			//������ȷ�򿽱���������
			memcpy(pc_receive_msg, rx_data, LEN_RX_PACKET);	
		}
	}
	//��������Ч������ԭ���ݲ���
	if(res == 0) 
	{
		pc_receive_msg->rx_data.shoot_yaw = before_yaw;
		pc_receive_msg->rx_data.shoot_pitch = before_pitch;
	}
	else
	{
		//��¼��ν��յ��ĽǶ�ֵ
		before_yaw = pc_receive_msg->rx_data.shoot_yaw;
		before_pitch = pc_receive_msg->rx_data.shoot_pitch;
		for_pitch = 1000*pc_receive_msg->rx_data.shoot_pitch;
	}
}

//���������ж�ʱ�������ݵ�У���뿽��
//void USART6_RX_IRQHandler(void)
//{
//    if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE)) //&& __HAL_UART_GET_IT_SOURCE(&huart6, UART_IT_IDLE))
//    {
//        __HAL_UART_CLEAR_IDLEFLAG(&huart6);
//        uint32_t DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(huart6.hdmarx);
//        USART6_RX_STA = USART6_MAX_RECV_LEN - huart6.hdmarx->Instance->NDTR;
//        __HAL_DMA_DISABLE(huart6.hdmarx);
//        data_solve(&pc_receive_msg,USART6_RX_BUF);
//        __HAL_DMA_CLEAR_FLAG(huart6.hdmarx, DMA_FLAGS);
//        __HAL_DMA_SET_COUNTER(huart6.hdmarx,USART6_MAX_RECV_LEN);
//        __HAL_DMA_ENABLE(huart6.hdmarx);
//    }
//}
//�����ж�
//void USART6_RX_IRQHandler(void)
//{
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
//}
const rx_data_t *get_pc_rx_point(void)
{
    return &pc_receive_msg.rx_data;
}

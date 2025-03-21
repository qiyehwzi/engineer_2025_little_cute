/**
  ******************************************************************************
  * @file	 CyberGear.c
  * @author
  * @version V1.0.0
  * @date    2024/3/14
  * @brief   С�׵�����ƴ���
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#include "CyberGear.h"
#include "usart.h"
#include "bsp_usart.h"
//CAN
struct Tx_exCanIdInfo txCanIdEx;
union Tx_exId_uint32 trans_tx;
static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static uint32_t send_mail_box;

static uint8_t cybergear_runmode;
static uint16_t cybergear_index;
static float32_t cybergear_param_ref;

struct Rx_exCanIdInfo rxCanIdEx;
union Rx_exId_uint32 trans_rx;

cybergear_measure_t cybergear_measure;
//static cybergear_measure_single_t cybergear_measure_single;
cybergear_measure_single_t cybergear_measure_single;
static uint8_t cybergear_tx_data[12];

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
//����ԭʼ���ݣ�Ϊ12���ֽڣ�����24���ֽڳ��ȣ���ֹDMA����Խ��
static uint8_t cybergear_rx_buf[2][cybergear_RX_BUF_NUM];
/*****************************CAN*************************************/

//ͨ������7�����õ��ID
//void Can_crbergear_canID_set()
//ͨ������18�����õ�������
void Can_cybergert_send(CAN_TxHeaderTypeDef *pHeader, uint8_t aData[]) {
    HAL_CAN_AddTxMessage(CYBERGEAT_CAN, pHeader, aData, &send_mail_box);
}
void Can_cybergear_runmode_set(uint8_t id, uint16_t master_id, uint8_t runmode)
{
    cybergear_index = 0x7005;
    cybergear_runmode = runmode;

    txCanIdEx.id = id;
    txCanIdEx.data = master_id;
    txCanIdEx.mode = 18;
    txCanIdEx.res = 0;
    trans_tx.exid = txCanIdEx;
    gimbal_tx_message.ExtId = trans_tx.aa;//��չID
    gimbal_tx_message.IDE = CAN_ID_EXT; // ��չID  ����׼ID������չID��
    gimbal_tx_message.RTR = CAN_RTR_DATA;//����֡   (����֡��ң��֡)
    gimbal_tx_message.DLC = 0x08;//���ݳ���
    for(uint8_t i=0; i<8; i++)
    {
        gimbal_can_send_data[i] = 0;
    }
    memcpy(&gimbal_can_send_data[0], &cybergear_index, 2);
    memcpy(&gimbal_can_send_data[4], &cybergear_runmode, 1);
    Can_cybergert_send(&gimbal_tx_message, gimbal_can_send_data);
}
//ͨ������3��ʹ�ܵ��
void Can_cybergear_enable(uint8_t id, uint16_t master_id)
{

    txCanIdEx.id = id;
    txCanIdEx.data = master_id;
    txCanIdEx.mode = 3;
    txCanIdEx.res = 0;
    txCanIdEx.data = 0;
    trans_tx.exid = txCanIdEx;
    gimbal_tx_message.ExtId = trans_tx.aa;
    gimbal_tx_message.IDE = CAN_ID_EXT;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    for(uint8_t i=0; i<8; i++)
    {
        gimbal_can_send_data[i] = 0;
    }
    Can_cybergert_send(&gimbal_tx_message, gimbal_can_send_data);
}
//���Ʋ���д��iq_ref, loc_ref, spd_ref��ͨ������ 18�����Ʋ���д�룩
void Can_cybergear_param_write(uint8_t id, uint16_t master_id, uint16_t index, float32_t param_ref)
{
    cybergear_index = index;
    cybergear_param_ref = param_ref;

    txCanIdEx.id = id;
    txCanIdEx.data = master_id;
    txCanIdEx.mode = 18;
    txCanIdEx.res = 0;

    trans_tx.exid = txCanIdEx;
    gimbal_tx_message.ExtId = trans_tx.aa;
    gimbal_tx_message.IDE = CAN_ID_EXT;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;

    for(uint8_t i=0; i<8; i++)
    {
        gimbal_can_send_data[i] = 0;
    }
    memcpy(&gimbal_can_send_data[0], &cybergear_index, 2);
    memcpy(&gimbal_can_send_data[4], &cybergear_param_ref, 4);

    Can_cybergert_send(&gimbal_tx_message, gimbal_can_send_data);
}
//���ֹͣ���У�ͨ������ 4��
void Can_cybergear_stop(uint8_t id,uint16_t master_id)
{

    txCanIdEx.id = id;
    txCanIdEx.data = master_id;
    txCanIdEx.mode = 4;
    txCanIdEx.res = 0;
    trans_tx.exid = txCanIdEx;
    gimbal_tx_message.ExtId = trans_tx.aa;
    gimbal_tx_message.IDE = CAN_ID_EXT;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    for(uint8_t i=0; i<8; i++)
    {
        gimbal_can_send_data[i] = 0;
    }
    Can_cybergert_send(&gimbal_tx_message, gimbal_can_send_data);
}
/*����������ȡ*/
void Can_cybergear_param_read(uint8_t id, uint16_t master_id, uint16_t index)
{
    cybergear_index = index;
    uint32_t send_mail_box;
    txCanIdEx.id = id;
    txCanIdEx.data = master_id;
    txCanIdEx.mode = 17;
    txCanIdEx.res = 0;

    trans_tx.exid = txCanIdEx;
    gimbal_tx_message.ExtId = trans_tx.aa;
    gimbal_tx_message.IDE = CAN_ID_EXT;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;

    for(uint8_t i=0; i<8; i++)
    {
        gimbal_can_send_data[i] = 0;
    }
    memcpy(&gimbal_can_send_data[0], &cybergear_index, 2);
    HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/*��õ���cybergear����*/
void get_cybergear_motor_single_para(struct Rx_exCanIdInfo exid, uint8_t rx_data[8])
{
    if(exid.communication_mode == 17)
    {
        cybergear_measure_single.index = (rx_data[1] << 8 | rx_data[0]);
        cybergear_measure_single.param_one = (rx_data[5] << 8 | rx_data[4]);
    }
}
//���cybergear����
void get_cybergear_motor_measure(struct Rx_exCanIdInfo exid, uint8_t rx_data[8])
{
    if(exid.communication_mode == 2 && exid.id == 0 && exid.calibration_err == 0
            && exid.hall_coding_err == 0 && exid.hightemperature_err == 0
            && exid.magnetic_coding_err == 0 && exid.overcurrent_err == 0
            && exid.overcurrent_err == 0 && exid.undervoltage_err == 0)
    {
        cybergear_measure.mode = exid.mode;
        cybergear_measure.angle_i = (rx_data[0] << 8 | rx_data[1])^0x8000;
        cybergear_measure.gyro_i = (rx_data[2] << 8 | rx_data[3])^0x8000;
        cybergear_measure.torque_i = (rx_data[4] << 8 | rx_data[5])^0x8000;
        cybergear_measure.temperature_i = rx_data[6] << 8 | rx_data[7];

        cybergear_measure.angle = cybergear_measure.angle_i * angle_param ;
        cybergear_measure.gyro = cybergear_measure.gyro_i * spd_param ;
        cybergear_measure.torque = cybergear_measure.torque_i* torque_param ;
        cybergear_measure.temp = cybergear_measure.temperature_i * temp_param;
    }

}
/*****************************USART*************************************/

void USART_cybergear_init (void) {
    usart1_init(cybergear_rx_buf[0],cybergear_rx_buf[1],cybergear_RX_BUF_NUM);
    usart1_tx_dma_init();
    while(cybergear_measure_single.param_one != 3 || cybergear_measure.mode!= 2)
    {
        USART_cybergear_runmode_set(127,0,current_mode);
        HAL_Delay(50);
        USART_cybergear_enable(127,0);
        HAL_Delay(50);
        USART_cybergear_param_read(127,0,0x7005);
        HAL_Delay(50);
    }
}
static void USART_cybergear_send(uint8_t *data) {
    usart1_tx_dma_enable(data,12);
}
void USART_cybergear_enable(uint8_t id, uint16_t master_id)
{
    //29λID
    cybergear_tx_data[0] = 0X03;
    cybergear_tx_data[1] = 0X00;
    cybergear_tx_data[2] = master_id;
    cybergear_tx_data[3] = id;

    //8Byte������
    for(int i = 4 ; i < 12; i++) {
        cybergear_tx_data[i] = 0X00;
    }
    USART_cybergear_send(cybergear_tx_data);
}
//���ֹͣ���У�ͨ������ 4��
void USART_cybergear_stop(uint8_t id,uint16_t master_id)
{
    //29λID
    cybergear_tx_data[0] = 0X04;
    cybergear_tx_data[1] = 0X00;
    cybergear_tx_data[2] = master_id;
    cybergear_tx_data[3] = id;

    //8Byte������
    for(int i = 4 ; i < 12; i++) {
        cybergear_tx_data[i] = 0X00;
    }
    USART_cybergear_send(cybergear_tx_data);
}
//���Ʋ���д��iq_ref, loc_ref, spd_ref��ͨ������ 18�����Ʋ���д�룩
void USART_cybergear_param_write(uint8_t id, uint16_t master_id, uint16_t index, float32_t param_ref)
{
    //29λID
    cybergear_tx_data[0] = 0X12;//ͨ������ 18
    cybergear_tx_data[1] = 0X00;
    cybergear_tx_data[2] = master_id;
    cybergear_tx_data[3] = id;

    for(uint8_t i=4; i<12; i++)
    {
        cybergear_tx_data[i] = 0;
    }
    memcpy(&cybergear_tx_data[4], &index, 2);
    memcpy(&cybergear_tx_data[8], &param_ref, 4);
    USART_cybergear_send(cybergear_tx_data);
}
//���Ʋ�����ȡ��ͨ������ 17��
void USART_cybergear_param_read(uint8_t id, uint16_t master_id, uint16_t index)
{
    //29λID
    cybergear_tx_data[0] = 0X11;//ͨ������ 17
    cybergear_tx_data[1] = 0X00;
    cybergear_tx_data[2] = master_id;
    cybergear_tx_data[3] = id;

    for(uint8_t i=4; i<12; i++)
    {
        cybergear_tx_data[i] = 0;
    }
    memcpy(&cybergear_tx_data[4], &index, 2);
    USART_cybergear_send(cybergear_tx_data);
}
void USART_cybergear_runmode_set(uint8_t id, uint16_t master_id, uint8_t runmode)
{
    cybergear_index = 0x7005;
    cybergear_runmode = runmode;
    //29λID
    cybergear_tx_data[0] = 0X12;//ͨ������ 18
    cybergear_tx_data[1] = 0X00;
    cybergear_tx_data[2] = master_id;
    cybergear_tx_data[3] = id;

    for(uint8_t i=4; i<12; i++)
    {
        cybergear_tx_data[i] = 0;
    }
    memcpy(&cybergear_tx_data[4], &cybergear_index, 2);
    memcpy(&cybergear_tx_data[8], &cybergear_runmode, 1);
    USART_cybergear_send(cybergear_tx_data);
}
void USART_cybergear_current_set(uint8_t id, uint16_t master_id, float32_t current_num)
{
    USART_cybergear_param_write(id,master_id,0X7006,current_num);
}
//���cybergear����
static void USART_get_cybergear_motor_measure(uint8_t rx_data[12])
{
    trans_rx.aa = (rx_data[0] << 24) | (rx_data[1]  << 16) | (rx_data[2]  << 8) | rx_data[3];
    rxCanIdEx = trans_rx.exid;
    if(rxCanIdEx.communication_mode == 2 && rxCanIdEx.id == 0 && rxCanIdEx.calibration_err == 0
            && rxCanIdEx.hall_coding_err == 0 && rxCanIdEx.hightemperature_err == 0
            && rxCanIdEx.magnetic_coding_err == 0 && rxCanIdEx.overcurrent_err == 0
            && rxCanIdEx.overcurrent_err == 0 && rxCanIdEx.undervoltage_err == 0)
    {
        cybergear_measure.mode = rxCanIdEx.mode;
        cybergear_measure.angle_i = (rx_data[4] << 8 | rx_data[5])^0x8000;
        cybergear_measure.gyro_i = (rx_data[6] << 8 | rx_data[7])^0x8000;
        cybergear_measure.torque_i = (rx_data[8] << 8 | rx_data[9])^0x8000;
        cybergear_measure.temperature_i = rx_data[10] << 8 | rx_data[11];

        cybergear_measure.angle = cybergear_measure.angle_i * angle_param ;
        cybergear_measure.gyro = cybergear_measure.gyro_i * spd_param ;
        cybergear_measure.torque = cybergear_measure.torque_i* torque_param ;
        cybergear_measure.temp = cybergear_measure.temperature_i * temp_param;
    } else if(rxCanIdEx.communication_mode == 17) {
        {
            cybergear_measure_single.index = (rx_data[5] << 8 | rx_data[4]);
            cybergear_measure_single.param_one = rx_data[8];
        }
    }
}
//�����ж�
void USART1_RX_IRQHandler(void)
{
    if(huart1.Instance->SR & UART_FLAG_RXNE)//���յ�����
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if(USART1->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = cybergear_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart1_rx.Instance->NDTR = cybergear_RX_BUF_NUM;

            //set memory buffer 1
            //�趨������1
            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;

            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == cybergear_RX_BUF_LENGTH)
            {
                USART_get_cybergear_motor_measure(cybergear_rx_buf[0]);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = cybergear_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart1_rx.Instance->NDTR = cybergear_RX_BUF_NUM;

            //set memory buffer 0
            //�趨������0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == cybergear_RX_BUF_LENGTH)
            {
                //����ң��������
                USART_get_cybergear_motor_measure(cybergear_rx_buf[1]);

            }
        }
    }
}

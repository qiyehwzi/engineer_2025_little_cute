#include "bsp_can.h"
#include "main.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;//ʹ��
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK; //����ģʽCAN_FILTERMODE_IDLIST���б�ģʽCAN_FILTERMODE_IDMASK
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;//32λ��16λ
    can_filter_st.FilterIdHigh = 0x0000;           //CAN_FiR1�Ĵ����ĸ�16λ     �洢Ҫɸѡ��ID
    can_filter_st.FilterIdLow = 0x0000;             //CAN_FiR1�Ĵ����ĵ�16λ
    can_filter_st.FilterMaskIdHigh = 0x0000;      //CAN_FiR2�Ĵ����ĸ�16λ    ����ģʽΪ����
    can_filter_st.FilterMaskIdLow = 0x0000;       //CAN_FiR2�Ĵ����ĵ�16λ		�б�ģʽΪҪɸѡ��ID
    can_filter_st.FilterBank = 0;                //�˴������õ����ĸ�ɸѡ�����õ�CAN��ȡֵΪ0-13
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;//ͨ��ɸѡ���ı��Ĵ���FIFO0����FIFO1��
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//�����ж�


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);


}

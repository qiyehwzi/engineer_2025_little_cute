#include "VOFA_task.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "usart.h"
#include "gimbal_task.h"
#define CH_COUNT 4 //ͨ����
#define CH_DATA_LEN CH_COUNT*4+4

float fdata[CH_COUNT];

uint8_t tail[4]= {0x00, 0x00, 0x80, 0x7f}; //֡β
extern gimbal_control_t gimbal_control;
static void VOFA_Send_data(fp32 *fdata);
static void Float_to_Byte(float fvalue,uint8_t *arr) ;
void VOFA_task(void const * argument) {
     usart1_tx_dma_init();
	   vTaskDelay(1000);
    while(1) {
				fdata[0] = gimbal_control.gimbal_yaw_motor.absolute_angle_set;
        fdata[1] = gimbal_control.gimbal_yaw_motor.absolute_angle;
        fdata[2] = gimbal_control.gimbal_yaw_motor.motor_gyro_set;
        fdata[3] = gimbal_control.gimbal_yaw_motor.motor_gyro;
        VOFA_Send_data(fdata);
        vTaskDelay(50);
    }
}

static void VOFA_Send_data(fp32 *send_data) {

    static uint8_t tempData[CH_DATA_LEN];
    uint8_t byte[4];

    int len = 0;
    for(int i =0; i < CH_COUNT ; i++) {
        Float_to_Byte(fdata[i],byte);
        for(int j =0; j<4; j++) {
            tempData[len++] = byte[j];
        }
    }
    tempData[CH_DATA_LEN-4] = 0x00;                    //д��β����
    tempData[CH_DATA_LEN-3] = 0x00;
    tempData[CH_DATA_LEN-2] = 0x80;
    tempData[CH_DATA_LEN-1] = 0x7f;

    usart1_tx_dma_enable(tempData,CH_DATA_LEN);

}
/*
��������fת��Ϊ4���ֽ����ݴ����byte[4]��
*/
void Float_to_Byte(float fvalue,uint8_t *arr)
{
    unsigned char  *pf;
    unsigned char *px;
    unsigned char i;   //������
    pf =(unsigned char *)&fvalue;            /*unsigned char��ָ��ȡ�ø��������׵�ַ*/
    px = arr;                               /*�ַ�����arr׼���洢���������ĸ��ֽ�,pxָ��ָ���ֽ�����arr*/

    for(i=0; i<4; i++)
    {
        *(px+i)=*(pf+i);     /*ʹ��unsigned char��ָ��ӵ͵�ַһ���ֽ�һ���ֽ�ȡ��*/
    }
}

/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       IST8310.c/h
  * @brief      IST8310����������������������ʼ���������������ݺ�����ͨ�Ŷ�ȡ����
  *             �������ǽ�MPU6500 IIC_SLV0����Ϊ�Զ���ȡIST8310���ݣ���ȡ
  *             MPU_EXT_SENS_DATA_00������IST8310��Status��ͨ���жϱ�־λ��������
  *             ���ݡ�
  * @note       IST8310ֻ֧��IIC��ȡ
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#ifndef IST8310DRIVER_H
#define IST8310DRIVER_H

#include "struct_typedef.h"

#define IST8310_DATA_READY_BIT 2
//T�Ÿ�Ӧǿ��
#define MAG_SEN 0.3f //ת���� uT

#define IST8310_IIC_ADDRESS (0x0E << 1)  //IST8310��IIC��ַ

#define IST8310_WHO_AM_I 0x00       //ist8310 who am I ID�Ĵ���
#define IST8310_WHO_AM_I_VALUE 0x10 //�豸 ID

#define IST8310_NO_ERROR 0x00 //û�д���

#define IST8310_NO_SENSOR 0x40//û�д�����

typedef struct ist8310_real_data_t
{
    uint8_t status;   //״̬
    fp32 mag[3];    //ֵ
} ist8310_real_data_t;
extern uint8_t ist8310_init(void);

#endif

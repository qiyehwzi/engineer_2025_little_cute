#ifndef _UI_TASK_H
#define _UI_TASK_H
#include "stm32f4xx_hal.h"
#include "referee.h"

//����Ƶ���Ƽ�������2Hz
#define UI_TIME 50

//����ID data_cmd_id ���ȣ�ͷ�ṹ����+�������ݶγ��ȣ�    ����˵��
//0x0200~0x02FF      6+n                                  ���������˼�ͨ��
//0x0100             6+2                                  �ͻ���ɾ��ͼ��
//0x0101             6+15                                 �ͻ��˻���һ��ͼ��
//0x0102             6+30                                 �ͻ��˻��ƶ���ͼ��
//0x0103             6+75                                 �ͻ��˻������ͼ��
//0x0104             6+105                                �ͻ��˻����߸�ͼ��
//0x0110             6+45                                 �ͻ��˻����ַ�ͼ

//������ID
typedef __packed struct
{
uint16_t data_cmd_id;					
uint16_t sender_ID;                   
uint16_t receiver_ID;
}ext_student_interactive_header_data_t;  

//ͼ������
typedef __packed struct
{
	uint8_t graphic_name[3];
	uint32_t operate_tpye:3;
	uint32_t graphic_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t radius:10;
	uint32_t end_x:11;
	uint32_t end_y:11;
}graphic_data_struct_t;
//����������
typedef __packed struct
{
	uint8_t graphic_name[3];
	uint32_t operate_tpye:3;
	uint32_t graphic_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	int32_t    num;
}int_data_struct_t;   

//�ַ�������
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
uint8_t data[30];
} ext_client_custom_character_t;
//����һ��ͼ��
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
}ext_client_custom_graphic_single_t;     
//��������ͼ��
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;
//�������ͼ��
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[5];
}ext_client_custom_graphic_five_t;        
//��ͼ�ṹ��
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[7];
}ext_client_custom_graphic_seven_t; 

//ɾ��ͼ��
typedef __packed struct
{
	uint8_t operate_tpye;         						
	uint8_t layer;
}ext_client_custom_graphic_delete_t;

//��ͼ�ṹ��
typedef __packed struct

{
	tFrameHeader                           FrameHead;                 //֡ͷ
	uint16_t                               CmdId;                     //������ 
	ext_student_interactive_header_data_t  Interactive_header_data;   //�������ݽ�����Ϣ�����ݶΣ�
	ext_client_custom_graphic_single_t     Client_graphic_single;     //ͼ�����ݣ����ݶΣ�
 	uint16_t                               CRC16;
}graphic_single_t;      
//��ͼ�ṹ��
typedef __packed struct
	
{
	tFrameHeader                           FrameHead;                 //֡ͷ
	uint16_t                               CmdId;                     //������ 
	ext_student_interactive_header_data_t  Interactive_header_data;   //�������ݽ�����Ϣ�����ݶΣ�
	ext_client_custom_graphic_double_t     Client_graphic_double;     //ͼ�����ݣ����ݶΣ�
 	uint16_t                               CRC16;
}graphic_double_t;      
//��ͼ�ṹ��
typedef __packed struct
{
	tFrameHeader                           FrameHead;                 //֡ͷ
	uint16_t                               CmdId;                     //������ 
	ext_student_interactive_header_data_t  Interactive_header_data;   //�������ݽ�����Ϣ�����ݶΣ�
	ext_client_custom_graphic_five_t       Client_graphic_five;       //ͼ�����ݣ����ݶΣ�
 	uint16_t                               CRC16;
}graphic_five_t;                        
//ɾ��ͼ��ṹ��
typedef __packed struct
{
	tFrameHeader                           FrameHead;                 //֡ͷ
	uint16_t                               CmdId;                     //������ 
	ext_student_interactive_header_data_t  Interactive_header_data;   //�������ݽ�����Ϣ�����ݶΣ�
	ext_client_custom_graphic_delete_t     Client_Dele;               //ͼ�����ݣ����ݶΣ�
 	uint16_t                               CRC16;
}graphic_dele_t;                      
//�������ṹ��
typedef __packed struct
{
	tFrameHeader                           FrameHead;              	//֡ͷ
	uint16_t                               CmdId;                  	//������ 
	ext_student_interactive_header_data_t  Interactive_header_data; //�������ݽ�����Ϣ�����ݶΣ�
	int_data_struct_t                    Client_float_single;       //ͼ�����ݣ����ݶΣ�
 	uint16_t                               CRC16;
}float_single_t;               
//�ַ��ͽṹ��
typedef __packed struct
{
	tFrameHeader                           FrameHead;             	//֡ͷ
	uint16_t                               CmdId;                  	//������ 
	ext_student_interactive_header_data_t  Interactive_header_data; //�������ݽ�����Ϣ�����ݶΣ�
	ext_client_custom_character_t          Client_character;       	//ͼ�����ݣ����ݶΣ�
 	uint16_t                               CRC16;
}character_t;                      
//��ͼ�ṹ��
typedef __packed struct
{
	tFrameHeader                           FrameHead;                 //֡ͷ
	uint16_t                               CmdId;                     //������ 
	ext_student_interactive_header_data_t  Interactive_header_data;   //�������ݽ�����Ϣ�����ݶΣ�
	ext_client_custom_graphic_seven_t       Client_graphic_seven;       //ͼ�����ݣ����ݶΣ�
 	uint16_t                               CRC16;
}graphic_seven_t;  

//UI��Ϣ
typedef __packed struct
{
	float_single_t float_single;
	graphic_five_t graphic_five;
	character_t    character;
}UI_message_t;	

extern void ui_task(void const *argu);

extern uint8_t opto_switch1_flag;
extern uint8_t opto_switch2_flag;

#endif

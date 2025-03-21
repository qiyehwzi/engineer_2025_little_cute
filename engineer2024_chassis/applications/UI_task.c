//#include "UI_task.h"
//#include "bsp_usart.h"
//#include "CAN_receive.h"
//#include "chassis_behaviour.h"
//#include "usart.h"
//#include "stdio.h"
//#include "protocol.h"
//#include "cmsis_os.h"
////#include "ore_task.h"
////#include "catch_task.h"
//#include "small_task.h"

//uint8_t seqcount = 0;
//uint8_t clear_time = 0;
//uint8_t opto_switch1_flag;
//uint8_t opto_switch2_flag;

//UI_message_t 	 	UI_message;				//UIͼ�νṹ��  //����ģʽ ��翪����X2  �������� ״̬����	3δʹ��	
//graphic_seven_t message_car_collimator_1; 	// ̧�����ס��п�����״̬�� 								6δʹ��
//graphic_seven_t message_car_collimator_2;	//һ��������̧��״̬��									
//graphic_five_t message_car_collimator_3;	//���ľ�Ԯץ���Լ���Ԯ����ץ�ֵ��������״̬��
//graphic_double_t message_car_arc; 			//����ץ���Լ�ͼ�����״̬ͼ��


//UI_message_t 	 	UI_message_float;	//UI���ֽṹ��
//character_t    		message_character;	//�ַ�
//graphic_dele_t 		message_dele;		//ɾ��ͼ��
//float_single_t 		message_onefloat;	//������


//static void init_message(UI_message_t *UI_message,UI_message_t *UI_message_float);
//static void init_collimator_1(void);
//static void init_collimator_2(void);
//static void init_collimator_3(void);
//static void init_arc(void);

//static void send_message(UI_message_t *UI_message,UI_message_t *UI_message_float);
//static void send_dele(uint8_t layer);
//static void send_collimator_1(void);
//static void send_collimator_2(void);
//static void send_collimator_3(void);
//static void send_arc(void);


////UI����
//void ui_task(void const *argu)
//{	
//	osDelay(1000);
//	//��ʼ��
//	init_message(&UI_message,&UI_message_float);
//	init_collimator_1();
//	init_collimator_2();
//	init_collimator_3();
//	init_arc();

//	uint32_t UI_wake_time = osKernelSysTick();	
//	
//	while(1)
//	{
//		//��ȡ��翪�ر�־λ
//		opto_switch1_flag = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
//		opto_switch2_flag = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
//		//Ϊ��ֹUI������ֹ��ϣ�ÿ��5s����һ������
//		clear_time++;
//		if(clear_time == 30)
//		{
//			send_dele(0);
//			send_dele(1);
//			//�������ٽ����ʼ��
//			init_message(&UI_message, &UI_message_float);
//			init_collimator_1();
//			init_collimator_2();
//			init_collimator_3();
//			init_arc();
//			osDelay(UI_TIME);
//			clear_time = 0;
//		}
//		
//		//����UI��������
//		send_collimator_1();
//		send_collimator_2();
//		send_collimator_3();
//		send_arc();
//		send_message(&UI_message,&UI_message_float);
//		osDelay(UI_TIME);
//	}
//}
////����ģʽ ��翪���� ����ģʽ �������� ״̬����
//static void init_message(UI_message_t *UI_message, UI_message_t *UI_message_float)
//{
//	
//	UI_message->graphic_five.FrameHead.sof = 0xA5;		
//	UI_message->graphic_five.FrameHead.dataLenth = 81;
//	UI_message->graphic_five.FrameHead.seq = (seqcount++) & 0xFF;  									//֡ͷ���
//	UI_message->graphic_five.CmdId = 0x0301;                     									//����������
//	UI_message->graphic_five.Interactive_header_data.data_cmd_id = 0x0103;   						//UIͼ��������
//	UI_message->graphic_five.Interactive_header_data.sender_ID = robot_state.robot_id;				//�����˷���ID
//	UI_message->graphic_five.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;	//�����˽���ID
//	
//	//����ģʽָʾ��
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[0] = 0;			//ͼ��ID����Ҫ�ظ�����
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[1] = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[2] = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].operate_tpye = 1;			//��Ϊ����
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_tpye = 2;			//ͼ�����ʹ���
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].layer = 1;					//ͼ�����
//	if(chassis_behaviour_flag == 0)
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 4;
//	else if(chassis_behaviour_flag == 1)
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 2;
//	else if(chassis_behaviour_flag == 2)
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 3;
//	else
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 7;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_angle = 0;				//��ʼ�Ƕ�	
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_angle = 0;				//��ֹ�Ƕ�
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].width = 5;					//�߿�
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_x = 1820;				//x�������
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_y = 830;				//y�������
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].radius = 15;					//Բ�뾶
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_x = 0;					//x�յ�����
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_y = 0;					//y�յ�����
//	
//	
//	//��翪��1
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[0] = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[1] = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[2] = 1;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].operate_tpye = 1;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_tpye = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].layer = 1;
//	if(opto_switch1_flag == 0)
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 4;  //ǰ����⵽���ϰ�ʱ��ʾ��ɫ
//	else if(opto_switch1_flag == 1)
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 2;  //ǰ�����û���ϰ�ʱ��ʾ��ɫ
//	else
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 1;   //�쳣ʱ��ʾ��ɫ
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].width = 2;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].start_x = 760;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].start_y = 540;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].end_x = 760;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].end_y = 270;
//	
//	
//	//��翪��2
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_name[0] = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_name[1] = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_name[2] = 2;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].operate_tpye = 1;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_tpye = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].layer = 1;
//	if(opto_switch2_flag == 0)
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].color = 4;
//	else if(opto_switch2_flag == 1)
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].color = 2;
//	else
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].color = 1;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].width = 2;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].start_x = 1160;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].start_y = 540;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].end_x = 1160;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].end_y = 270;
//	
//	
//	//����ģʽ��־λ
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].graphic_name[0] = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].graphic_name[1] = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].graphic_name[2] = 3;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].operate_tpye = 0;
//	
//	
//	//��������
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[4].graphic_name[0] = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[4].graphic_name[1] = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[4].graphic_name[2] = 4;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[4].operate_tpye = 1;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[4].graphic_tpye = 1;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[4].layer = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[4].color = 7;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[4].width = 2;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[4].start_x = 100;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[4].start_y = 820;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[4].end_x = 250;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[4].end_y = 650;
//	
//	append_crc8_check_sum(&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five.FrameHead));
//	append_crc16_check_sum(&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five.FrameHead)
//	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + UI_message->graphic_five.FrameHead.dataLenth);
//	HAL_UART_Transmit(&JUDGE_HUART,&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five),0xffff);
//	
//	
//	//����״̬����
//	UI_message->character.FrameHead.sof = 0xA5;
//	UI_message->character.FrameHead.dataLenth = 51;
//	UI_message->character.FrameHead.seq = (seqcount++) & 0xFF;
//	UI_message->character.CmdId = 0x0301;   
//	UI_message->character.Interactive_header_data.data_cmd_id = 0x0110;
//	UI_message->character.Interactive_header_data.sender_ID = robot_state.robot_id;
//	UI_message->character.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;
//	UI_message->character.Client_character.grapic_data_struct.graphic_name[0] = 0;
//	UI_message->character.Client_character.grapic_data_struct.graphic_name[1] = 1;
//	UI_message->character.Client_character.grapic_data_struct.graphic_name[2] = 0;
//	UI_message->character.Client_character.grapic_data_struct.operate_tpye = 1;
//	UI_message->character.Client_character.grapic_data_struct.graphic_tpye = 7;
//	UI_message->character.Client_character.grapic_data_struct.layer = 0;
//	UI_message->character.Client_character.grapic_data_struct.color = 7;
//	UI_message->character.Client_character.grapic_data_struct.start_angle = 20;
//	UI_message->character.Client_character.grapic_data_struct.end_angle = 3;
//	UI_message->character.Client_character.grapic_data_struct.width = 3;
//	UI_message->character.Client_character.grapic_data_struct.start_x = 1735;
//	UI_message->character.Client_character.grapic_data_struct.start_y = 840;
//	UI_message->character.Client_character.grapic_data_struct.radius = 0;
//	UI_message->character.Client_character.grapic_data_struct.end_x = 0;
//	UI_message->character.Client_character.grapic_data_struct.end_y = 0;
//	UI_message->character.Client_character.data[0] = 'c';
//	UI_message->character.Client_character.data[1] = 'h';
//	UI_message->character.Client_character.data[2] = 'a';
//	
//	append_crc8_check_sum(&UI_message->character.FrameHead.sof,sizeof(UI_message->character.FrameHead));
//	append_crc16_check_sum(&UI_message->character.FrameHead.sof,sizeof(UI_message->character.FrameHead)
//	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + UI_message->character.FrameHead.dataLenth);	
//	HAL_UART_Transmit(&JUDGE_HUART,&UI_message->character.FrameHead.sof,sizeof(UI_message->character),0xffff);
//	
//	
//	//����ģʽ״̬����
//	UI_message->character.FrameHead.sof = 0xA5;
//	UI_message->character.FrameHead.dataLenth = 51;
//	UI_message->character.FrameHead.seq = (seqcount++) & 0xFF;
//	UI_message->character.CmdId = 0x0301;   
//	UI_message->character.Interactive_header_data.data_cmd_id = 0x0110;
//	UI_message->character.Interactive_header_data.sender_ID = robot_state.robot_id;
//	UI_message->character.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;
//	UI_message->character.Client_character.grapic_data_struct.graphic_name[0] = 0;
//	UI_message->character.Client_character.grapic_data_struct.graphic_name[1] = 1;
//	UI_message->character.Client_character.grapic_data_struct.graphic_name[2] = 1;
//	UI_message->character.Client_character.grapic_data_struct.operate_tpye = 1;
//	UI_message->character.Client_character.grapic_data_struct.graphic_tpye = 7;
//	UI_message->character.Client_character.grapic_data_struct.layer = 0;
//	UI_message->character.Client_character.grapic_data_struct.color = 7;
//	UI_message->character.Client_character.grapic_data_struct.start_angle = 20;
//	UI_message->character.Client_character.grapic_data_struct.end_angle = 3;
//	UI_message->character.Client_character.grapic_data_struct.width = 3;
//	UI_message->character.Client_character.grapic_data_struct.start_x = 1735;
//	UI_message->character.Client_character.grapic_data_struct.start_y = 800;
//	UI_message->character.Client_character.grapic_data_struct.radius = 0;
//	UI_message->character.Client_character.grapic_data_struct.end_x = 0;
//	UI_message->character.Client_character.grapic_data_struct.end_y = 0;
//	UI_message->character.Client_character.data[0] = 's';
//	UI_message->character.Client_character.data[1] = 'h';
//	UI_message->character.Client_character.data[2] = 'i';
//	
//	append_crc8_check_sum(&UI_message->character.FrameHead.sof,sizeof(UI_message->character.FrameHead));
//	append_crc16_check_sum(&UI_message->character.FrameHead.sof,sizeof(UI_message->character.FrameHead)
//	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + UI_message->character.FrameHead.dataLenth);	
//	HAL_UART_Transmit(&JUDGE_HUART,&UI_message->character.FrameHead.sof,sizeof(UI_message->character),0xffff);
//	
//	//����ģʽ��־λ 123Ϊ��Ӧģʽ 0Ϊ�쳣
////	UI_message_float->float_single.FrameHead.sof = 0xA5;
////	UI_message_float->float_single.FrameHead.dataLenth = 21;
////	UI_message_float->float_single.FrameHead.seq = (seqcount++) & 0xFF;
////	UI_message_float->float_single.CmdId = 0x0301;   
////	UI_message_float->float_single.Interactive_header_data.data_cmd_id = 0x0101;
////	UI_message_float->float_single.Interactive_header_data.sender_ID = robot_state.robot_id;
////	UI_message_float->float_single.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;
////	UI_message_float->float_single.Client_float_single.graphic_name[0] = 0;
////	UI_message_float->float_single.Client_float_single.graphic_name[1] = 1;
////	UI_message_float->float_single.Client_float_single.graphic_name[2] = 2;
////	UI_message_float->float_single.Client_float_single.operate_tpye = 1;
////	UI_message_float->float_single.Client_float_single.graphic_tpye = 6;
////	UI_message_float->float_single.Client_float_single.layer = 1;
////	UI_message_float->float_single.Client_float_single.color = 4;
////	UI_message_float->float_single.Client_float_single.start_angle = 20;
////	UI_message_float->float_single.Client_float_single.end_angle = 0;
////	UI_message_float->float_single.Client_float_single.width = 3;
////	UI_message_float->float_single.Client_float_single.start_x = 1815;
////	UI_message_float->float_single.Client_float_single.start_y = 800;
////	if(mode_control_flag == FIRST_MODE)
////		UI_message_float->float_single.Client_float_single.num = 1;
////	else if(mode_control_flag == SECOND_MODE)
////		UI_message_float->float_single.Client_float_single.num = 2;
////	else if(mode_control_flag == THIRD_MODE)
////		UI_message_float->float_single.Client_float_single.num = 3;
////	else
////		UI_message_float->float_single.Client_float_single.num = 0;

//	
//	append_crc8_check_sum(&UI_message_float->float_single.FrameHead.sof,sizeof(UI_message_float->float_single.FrameHead));
//	append_crc16_check_sum(&UI_message_float->float_single.FrameHead.sof,sizeof(UI_message_float->float_single.FrameHead)
//	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + UI_message_float->float_single.FrameHead.dataLenth);
//	HAL_UART_Transmit(&JUDGE_HUART,&UI_message_float->float_single.FrameHead.sof,sizeof(UI_message_float->float_single),0xffff);
//	
//}

////���µ���״̬ ��翪�� ����ģʽ��־λ������
//void send_message(UI_message_t *UI_message, UI_message_t *UI_message_float)
//{
//	
//	UI_message->graphic_five.FrameHead.sof = 0xA5;
//	UI_message->graphic_five.FrameHead.dataLenth = 81;
//	UI_message->graphic_five.FrameHead.seq = (seqcount++) & 0xFF;
//	UI_message->graphic_five.CmdId = 0x0301;
//	UI_message->graphic_five.Interactive_header_data.data_cmd_id = 0x0103;   
//	UI_message->graphic_five.Interactive_header_data.sender_ID = robot_state.robot_id;
//	UI_message->graphic_five.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;
//	
//	
//	//����״̬��־λ����
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[0] = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[1] = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[2] = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].operate_tpye = 2;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_tpye = 2;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].layer = 1;
//	if(chassis_behaviour_flag == 0)
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 4;
//	else if(chassis_behaviour_flag == 1)
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 2;
//	else if(chassis_behaviour_flag == 2)
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 3;
//	else
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 7;
//	
//	
//	//ǰ����翪�ر�־λ����
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[0] = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[1] = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[2] = 1;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].operate_tpye = 2;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_tpye = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].layer = 1;
//	if(opto_switch1_flag == 0)
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 4;  //ǰ����⵽���ϰ�ʱ��ʾ��ɫ
//	else if(opto_switch1_flag == 1)
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 2;  //ǰ�����û���ϰ�ʱ��ʾ��ɫ
//	else
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 1;   //�쳣ʱ��ʾ��ɫ
//	
//	
//	
//	//�󷽹�翪��־λ����
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_name[0] = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_name[1] = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_name[2] = 2;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].operate_tpye = 2;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_tpye = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].layer = 1;
//	if(opto_switch2_flag == 0)
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].color = 4;
//	else if(opto_switch2_flag == 1)
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].color = 2;
//	else
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].color = 1;
//	
//	
//	
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].graphic_name[0] = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].graphic_name[1] = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].graphic_name[2] = 3;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].operate_tpye = 0;
//	
//	
//	append_crc8_check_sum(&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five.FrameHead));
//	append_crc16_check_sum(&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five.FrameHead)
//	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + UI_message->graphic_five.FrameHead.dataLenth);
//	HAL_UART_Transmit(&JUDGE_HUART,&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five),0xffff);
//	
//	
//	//����ģʽ��־λ����
//	UI_message_float->float_single.FrameHead.sof = 0xA5;
//	UI_message_float->float_single.FrameHead.dataLenth = 21;
//	UI_message_float->float_single.FrameHead.seq = (seqcount++) & 0xFF;
//	UI_message_float->float_single.CmdId = 0x0301;   
//	UI_message_float->float_single.Interactive_header_data.data_cmd_id = 0x0101;
//	UI_message_float->float_single.Interactive_header_data.sender_ID = robot_state.robot_id;
//	UI_message_float->float_single.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;
//	UI_message_float->float_single.Client_float_single.graphic_name[0] = 0;
//	UI_message_float->float_single.Client_float_single.graphic_name[1] = 1;
//	UI_message_float->float_single.Client_float_single.graphic_name[2] = 2;
//	UI_message_float->float_single.Client_float_single.operate_tpye = 2;
////	if(mode_control_flag == FIRST_MODE)
////		UI_message_float->float_single.Client_float_single.num = 1;
////	else if(mode_control_flag == SECOND_MODE)
////		UI_message_float->float_single.Client_float_single.num = 2;
////	else if(mode_control_flag == THIRD_MODE)
////		UI_message_float->float_single.Client_float_single.num = 3;
////	else
////		UI_message_float->float_single.Client_float_single.num = 0;

//	
//	append_crc8_check_sum(&UI_message_float->float_single.FrameHead.sof,sizeof(UI_message_float->float_single.FrameHead));
//	append_crc16_check_sum(&UI_message_float->float_single.FrameHead.sof,sizeof(UI_message_float->float_single.FrameHead)
//	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + UI_message_float->float_single.FrameHead.dataLenth);
//	HAL_UART_Transmit(&JUDGE_HUART,&UI_message_float->float_single.FrameHead.sof,sizeof(UI_message_float->float_single),0xffff);


//}

////���͹����ϰ�����״̬�� ̧�󡢼п����׵�״ֱ̬�߶�����г�ʼ��
//void init_collimator_1(void)
//{
//	message_car_collimator_1.FrameHead.sof = 0xA5;
//	message_car_collimator_1.FrameHead.dataLenth = 111;
//	message_car_collimator_1.FrameHead.seq = (seqcount++) & 0xFF;
//	message_car_collimator_1.CmdId = 0x0301;                    
//	message_car_collimator_1.Interactive_header_data.data_cmd_id = 0x0104;
//	message_car_collimator_1.Interactive_header_data.sender_ID = robot_state.robot_id;                
//	message_car_collimator_1.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;     
//	
//	
//	//��0����3Ϊ�����ϰ�����״̬�ߣ���4Ϊ�п�����״̬�ߣ���5Ϊ̧������״̬�� 6δʹ��
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[0].graphic_name[0] = 1;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[0].graphic_name[1] = 0;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[0].graphic_name[2] = 0;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[0].operate_tpye = 1;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[0].graphic_tpye = 0;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[0].layer = 1;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[0].color = 2;
//	if(servo_flag == 3)
//		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[0].color = 2;
//	else if(servo_flag == 0 || servo_flag == 1 ||servo_flag == 2)
//		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[0].color = 4;
//	else
//		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[0].color = 1;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[0].width = 2;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[0].start_x = 120;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[0].start_y = 570;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[0].end_x = 230;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[0].end_y = 570;
//	
//	
//	//����ڶ����µ���
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[1].graphic_name[0] = 1;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[1].graphic_name[1] = 0;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[1].graphic_name[2] = 1;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[1].operate_tpye = 1;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[1].graphic_tpye = 0;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[1].layer = 1;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[1].color = 2;
//	if(servo_flag == 2)
//		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[1].color = 2;
//	else if(servo_flag == 0 || servo_flag == 1 ||servo_flag == 3)
//		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[1].color = 4;
//	else
//		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[1].color = 1;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[1].width = 2;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[1].start_x = 120;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[1].start_y = 590;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[1].end_x = 230;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[1].end_y = 570;
//	
//	
//	//���̧���ϰ����Ժ����
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[2].graphic_name[0] = 1;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[2].graphic_name[1] = 0;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[2].graphic_name[2] = 2;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[2].operate_tpye = 1;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[2].graphic_tpye = 0;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[2].layer = 1;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[2].color = 2;
//	if(servo_flag == 1)
//		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[2].color = 2;
//	else if(servo_flag == 0 || servo_flag == 3 ||servo_flag == 2)
//		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[2].color = 4;
//	else
//		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[2].color = 1;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[2].width = 2;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[2].start_x = 120;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[2].start_y = 610;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[2].end_x = 230;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[2].end_y = 570;
//	
//	
//	//����ջ��Ժ����
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[3].graphic_name[0] = 1;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[3].graphic_name[1] = 0;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[3].graphic_name[2] = 3;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[3].operate_tpye = 1;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[3].graphic_tpye = 0;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[3].layer = 1;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[3].color = 2;
//	if(servo_flag == 0)
//		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[3].color = 2;
//	else if(servo_flag == 3 || servo_flag == 1 ||servo_flag == 2)
//		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[3].color = 4;
//	else
//		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[3].color = 1;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[3].width = 2;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[3].start_x = 230;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[3].start_y = 630;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[3].end_x = 230;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[3].end_y = 570;
//	
//	
//	//̧������
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].graphic_name[0] = 1;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].graphic_name[1] = 0;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].graphic_name[2] = 4;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].operate_tpye = 1;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].graphic_tpye = 0;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].layer = 1;
////	if(ore_store2_flag == 0)
////		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].color = 4;  //̧�����׹ر�ʱ��ʾ��ɫ
////	else if(ore_store2_flag == 1)
////		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].color = 2;   //̧�����״�ʱ��ʾ��ɫ
////	else
////		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].color = 1;    //�����쳣ʱ��ɫ��ɫ
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].width = 2;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].start_x = 220;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].start_y = 670;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].end_x = 220;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].end_y = 750;
//		
//	
//	//�п�����
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].graphic_name[0] = 1;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].graphic_name[1] = 0;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].graphic_name[2] = 5; 
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].operate_tpye = 1;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].graphic_tpye = 0;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].layer = 1;
////	if(ore_store1_flag == 0)
////		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].color = 4;  //�п����׹ر�ʱ��ʾ��ɫ
////	else if(ore_store1_flag == 1)
////		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].color = 2;  //�п����״�ʱ��ʾ��ɫ
////	else
////		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].color = 1;   //�쳣ʱ��ʾ��ɫ	
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].width = 2;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].start_x = 150;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].start_y = 660;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].end_x = 220;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].end_y = 660;
//	
//	
//	
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[6].graphic_name[0] = 1;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[6].graphic_name[0] = 0;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[6].graphic_name[0] = 6;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[6].operate_tpye = 0;
//	
//	append_crc8_check_sum(&message_car_collimator_1.FrameHead.sof,sizeof(message_car_collimator_1.FrameHead));
//	append_crc16_check_sum(&message_car_collimator_1.FrameHead.sof,sizeof(message_car_collimator_1.FrameHead)
//	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + message_car_collimator_1.FrameHead.dataLenth);
//	HAL_UART_Transmit(&JUDGE_HUART,&message_car_collimator_1.FrameHead.sof,sizeof(message_car_collimator_1),0xffff);
//}


////����ֱ�߶Գ��ļп�̧������״̬�߽��и���
//void send_collimator_1(void)
//{
//	message_car_collimator_1.FrameHead.sof = 0xA5;
//	message_car_collimator_1.FrameHead.dataLenth = 111;
//	message_car_collimator_1.FrameHead.seq = (seqcount++) & 0xFF;
//	message_car_collimator_1.CmdId = 0x0301;                    
//	message_car_collimator_1.Interactive_header_data.data_cmd_id = 0x0104;
//	message_car_collimator_1.Interactive_header_data.sender_ID = robot_state.robot_id;                
//	message_car_collimator_1.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;
//	
//	//��0����3Ϊ�����ϰ�����״̬�ߣ���4Ϊ�п�����״̬�ߣ���5Ϊ̧������״̬�� 6δʹ��
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[0].graphic_name[0] = 1;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[0].graphic_name[1] = 0;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[0].graphic_name[2] = 0;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[0].operate_tpye = 2;
//	if(servo_flag == 3)
//		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[0].color = 2;
//	else if(servo_flag == 0 || servo_flag == 1 ||servo_flag == 2)
//		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[0].color = 4;
//	else

//	
//	//����ڶ����µ���
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[1].graphic_name[0] = 1;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[1].graphic_name[1] = 0;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[1].graphic_name[2] = 1;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[1].operate_tpye = 2;
//	if(servo_flag == 2)
//		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[1].color = 2;
//	else if(servo_flag == 0 || servo_flag == 1 ||servo_flag == 3)
//		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[1].color = 4;
//	else

//	
//	//���̧���ϰ����Ժ����
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[2].graphic_name[0] = 1;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[2].graphic_name[1] = 0;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[2].graphic_name[2] = 2;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[2].operate_tpye = 2;
//	if(servo_flag == 1)
//		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[2].color = 2;
//	else if(servo_flag == 0 || servo_flag == 3 ||servo_flag == 2)
//		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[2].color = 4;
//	else

//	
//	//����ջ��Ժ����
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[3].graphic_name[0] = 1;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[3].graphic_name[1] = 0;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[3].graphic_name[2] = 3;
//	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[3].operate_tpye = 2;
//	if(servo_flag == 0)
//		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[3].color = 2;
//	else if(servo_flag == 3 || servo_flag == 1 ||servo_flag == 2)
//		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[3].color = 4;
//	else
//		
//	
//	//̧������
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].graphic_name[0] = 1;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].graphic_name[1] = 0;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].graphic_name[2] = 4;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].operate_tpye = 2;
////	if(ore_store2_flag == 0)
////		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].color = 4;  //̧�����׹ر�ʱ��ʾ��ɫ
////	else if(ore_store2_flag == 1)
////		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].color = 2;   //̧�����״�ʱ��ʾ��ɫ
////	else
////		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].color = 1;    //�����쳣ʱ��ɫ��ɫ

////	
////	//�п�����
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].graphic_name[0] = 1;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].graphic_name[1] = 0;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].graphic_name[2] = 5; 
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].operate_tpye = 2;
////	if(ore_store1_flag == 0)
////		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].color = 4;  //�п����׹ر�ʱ��ʾ��ɫ
////	else if(ore_store1_flag == 1)
////		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].color = 2;  //�п����״�ʱ��ʾ��ɫ
////	else
////		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].color = 1;   //�쳣ʱ��ʾ��ɫ		

//	append_crc8_check_sum(&message_car_collimator_1.FrameHead.sof,sizeof(message_car_collimator_1.FrameHead));
//	append_crc16_check_sum(&message_car_collimator_1.FrameHead.sof,sizeof(message_car_collimator_1.FrameHead)
//	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + message_car_collimator_1.FrameHead.dataLenth);
//	HAL_UART_Transmit(&JUDGE_HUART,&message_car_collimator_1.FrameHead.sof,sizeof(message_car_collimator_1),0xffff);
//}

////		��0Ϊһ��̧������״̬�ߣ���1Ϊ����̧������״̬��
//void init_collimator_2(void)
//{
//	message_car_collimator_2.FrameHead.sof = 0xA5;
//	message_car_collimator_2.FrameHead.dataLenth = 36;
//	message_car_collimator_2.FrameHead.seq = (seqcount++) & 0xFF;
//	message_car_collimator_2.CmdId = 0x0301;                    
//	message_car_collimator_2.Interactive_header_data.data_cmd_id = 0x0102;
//	message_car_collimator_2.Interactive_header_data.sender_ID = robot_state.robot_id;                
//	message_car_collimator_2.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;     
//	

//	//һ��̧��������
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].graphic_name[0] = 2;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].graphic_name[1] = 0;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].graphic_name[2] = 0;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].operate_tpye = 1;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].graphic_tpye = 0;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].layer = 1;
////	if(ore_lift1_flag == 0)
////		message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].color = 4;  //һ��̧�����׹ر�ʱ��ʾ��ɫ
////	else if(ore_lift1_flag == 1)
////		message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].color = 2;   //һ��̧�����״�ʱ��ʾ��ɫ
////	else
////		message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].color = 1;    //�����쳣ʱ��ɫ��ɫ
////	
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].width = 2;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].start_x = 260;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].start_y = 670;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].end_x = 360;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].end_y = 670;
////		
////	//����̧��������
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].graphic_name[0] = 2;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].graphic_name[1] = 0;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].graphic_name[2] = 1; 
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].operate_tpye = 1;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].graphic_tpye = 0;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].layer = 1;
////	if(ore_store2_flag == 0)
////		message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].color = 4;  //����̧�����׹ر�ʱ��ʾ��ɫ
////	else if(ore_store2_flag == 1)
////		message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].color = 2;  //����̧�����״�ʱ��ʾ��ɫ
////	else
////		message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].color = 1;   //�쳣ʱ��ʾ��ɫ
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].width = 2;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].start_x = 260;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].start_y = 740;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].end_x = 360;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].end_y = 740;
//	
//	append_crc8_check_sum(&message_car_collimator_2.FrameHead.sof,sizeof(message_car_collimator_2.FrameHead));
//	append_crc16_check_sum(&message_car_collimator_2.FrameHead.sof,sizeof(message_car_collimator_2.FrameHead)
//	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + message_car_collimator_2.FrameHead.dataLenth);
//	HAL_UART_Transmit(&JUDGE_HUART,&message_car_collimator_2.FrameHead.sof,sizeof(message_car_collimator_2),0xffff);
//}


////���¹������ʼ��������ͬ
////	��0Ϊһ��̧������״̬�ߣ���1Ϊ����̧������״̬��
//void send_collimator_2(void)
//{
//	message_car_collimator_2.FrameHead.sof = 0xA5;
//	message_car_collimator_2.FrameHead.dataLenth =36;
//	message_car_collimator_2.FrameHead.seq = (seqcount++) & 0xFF;
//	message_car_collimator_2.CmdId = 0x0301;                    
//	message_car_collimator_2.Interactive_header_data.data_cmd_id = 0x0102;
//	message_car_collimator_2.Interactive_header_data.sender_ID = robot_state.robot_id;                
//	message_car_collimator_2.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;
//	
//	//�ϰ���ת����Ӧ��λ�ö�Ӧ��ͼ�α�Ϊ��ɫ����������Ϊ��ɫ
//	//������±�λ�õ���
//		
//	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].graphic_name[0] = 2;
//	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].graphic_name[1] = 0;
//	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].graphic_name[2] = 4;
//	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].operate_tpye = 2;
//	if(ore_lift1_flag == 0)
//		message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].color = 4;  //һ��̧�����׹ر�ʱ��ʾ��ɫ
//	else if(ore_lift1_flag == 1)
//		message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].color = 2;   //һ��̧�����״�ʱ��ʾ��ɫ
//	else
//		message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].color = 1;    //�����쳣ʱ��ɫ��ɫ

//	//�п�����
//	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].graphic_name[0] = 2;
//	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].graphic_name[1] = 0;
//	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].graphic_name[2] = 5; 
//	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].operate_tpye = 2;
//	if(ore_lift2_flag == 0)
//		message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].color = 4;  //�п����׹ر�ʱ��ʾ��ɫ
//	else if(ore_lift2_flag == 1)
//		message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].color = 2;  //�п����״�ʱ��ʾ��ɫ
//	else
//		message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].color = 1;   //�쳣ʱ��ʾ��ɫ
//	
//	append_crc8_check_sum(&message_car_collimator_2.FrameHead.sof,sizeof(message_car_collimator_2.FrameHead));
//	append_crc16_check_sum(&message_car_collimator_2.FrameHead.sof,sizeof(message_car_collimator_2.FrameHead)
//	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + message_car_collimator_2.FrameHead.dataLenth);
//	HAL_UART_Transmit(&JUDGE_HUART,&message_car_collimator_2.FrameHead.sof,sizeof(message_car_collimator_2),0xffff);
//}


////��Ԯץ��X2����Ԯ����ץ�ֵ�����������߳�ʼ������
//void init_collimator_3(void)
//{
//	message_car_collimator_3.FrameHead.sof = 0xA5;
//	message_car_collimator_3.FrameHead.dataLenth = 81;
//	message_car_collimator_3.FrameHead.seq = (seqcount++) & 0xFF;
//	message_car_collimator_3.CmdId = 0x0301;                    
//	message_car_collimator_3.Interactive_header_data.data_cmd_id = 0x0103;
//	message_car_collimator_3.Interactive_header_data.sender_ID = robot_state.robot_id;                
//	message_car_collimator_3.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;
//	
//	//��Ԯץ�ֵ���1
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].graphic_name[0] = 3;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].graphic_name[1] = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].graphic_name[2] = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].operate_tpye = 1;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].graphic_tpye = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].layer = 1;
//	if(rescue_hand_flag == 0)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].color = 4;  //��Ԯץ�ֹر�ʱ��ʾΪ��ɫ
//	else if(rescue_hand_flag == 1)
//	  message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].color = 2;   //��Ԯץ�ִ�ʱ��ʾΪ��ɫ
//	else
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].color = 1;   //�쳣ʱ��ʾΪ��ɫ
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].width = 2;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].start_x = 120;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].start_y = 830;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].end_x = 120;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].end_y = 870;
//	
//	
//	
//	//��Ԯץ�ֵ���2������������1��ͬ
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[1].graphic_name[0] = 3;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[1].graphic_name[1] = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[1].graphic_name[2] = 1;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[1].operate_tpye = 1;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[1].graphic_tpye = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[1].layer = 1;
//	if(rescue_hand_flag == 0)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[1].color = 4;
//	else if(rescue_hand_flag == 1)
//	  message_car_collimator_3.Client_graphic_five.grapic_data_struct[1].color = 2;
//	else
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[1].color = 1;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[1].width = 2;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[1].start_x = 230;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[1].start_y = 830;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[1].end_x = 230;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[1].end_y = 870;
//	
//	
//	
//	//��Ԯ����־
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].graphic_name[0] = 3;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].graphic_name[1] = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].graphic_name[2] = 2;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].operate_tpye = 1;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].graphic_tpye = 2;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].layer = 1;
//	if(rescue_card_flag == 0)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].color = 4;  //��Ԯ��û�д�ʱ��ʾΪ��ɫ
//	else if(rescue_card_flag == 1)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].color = 2;  //��Ԯ����ʱ��ʾΪ��ɫ
//	else
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].color = 1;   //�쳣ʱ��ʾΪ��ɫ
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].width = 2;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].start_x = 175;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].start_y = 840;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].radius = 5;
//	
//	
//	//ץ�ֵ��λ��
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].graphic_name[0] = 3;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].graphic_name[1] = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].graphic_name[2] = 3;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].operate_tpye = 1;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].graphic_tpye = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].layer = 1;
//	if(rotate_motor_flag == 0)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].color = 4;  //ץ���ں������ʾΪ��ɫ
//	else if(rotate_motor_flag == 1)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].color = 2;  //ץ�ֵ�ǰ��ʱ��ʾΪ��ɫ
//	else
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].color = 1;   //�쳣ʱ��ʾΪ��ɫ	
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].width = 2;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].start_x = 280;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].start_y = 800;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].end_x = 280;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].end_y = 760;
//	
//	
//	
//	//ץ������λ��
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].graphic_name[0] = 3;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].graphic_name[1] = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].graphic_name[2] = 4;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].operate_tpye = 1;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].graphic_tpye = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].layer = 1;
//	if(ore_grab_flag == 0)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].color = 4;  //ץ������û�д�ʱ��ʾΪ��ɫ
//	else if(ore_grab_flag == 1)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].color = 2;   //ץ�����״�ʱ��ʾΪ��ɫ
//	else
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].color = 1;	
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].width = 2;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].start_x = 320;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].start_y = 800;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].end_x = 320;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].end_y = 760;

//	append_crc8_check_sum(&message_car_collimator_3.FrameHead.sof,sizeof(message_car_collimator_3.FrameHead));
//	append_crc16_check_sum(&message_car_collimator_3.FrameHead.sof,sizeof(message_car_collimator_3.FrameHead)
//	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + message_car_collimator_3.FrameHead.dataLenth);
//	HAL_UART_Transmit(&JUDGE_HUART,&message_car_collimator_3.FrameHead.sof,sizeof(message_car_collimator_3),0xffff);
//}


////��־λ����
//void send_collimator_3(void)
//{
//	message_car_collimator_3.FrameHead.sof = 0xA5;
//	message_car_collimator_3.FrameHead.dataLenth = 81;
//	message_car_collimator_3.FrameHead.seq = (seqcount++) & 0xFF;
//	message_car_collimator_3.CmdId = 0x0301;                    
//	message_car_collimator_3.Interactive_header_data.data_cmd_id = 0x0103;
//	message_car_collimator_3.Interactive_header_data.sender_ID = robot_state.robot_id;                
//	message_car_collimator_3.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;
//	
//	//��Ԯץ�ֵ���1
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].graphic_name[0] = 3;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].graphic_name[1] = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].graphic_name[2] = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].operate_tpye = 2;
//	if(rescue_hand_flag == 0)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].color = 4;  //��Ԯץ�ֹر�ʱ��ʾΪ��ɫ
//	else if(rescue_hand_flag == 1)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].color = 2;   //��Ԯץ�ִ�ʱ��ʾΪ��ɫ
//	else
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].color = 1;   //�쳣ʱ��ʾΪ��ɫ	}
//	
//	//��Ԯץ�ֵ���2������������1��ͬ
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[1].graphic_name[0] = 3;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[1].graphic_name[1] = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[1].graphic_name[2] = 1;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[1].operate_tpye = 2;
//	if(rescue_hand_flag == 0)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[1].color = 4;
//	else if(rescue_hand_flag == 1)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[1].color = 2;
//	else
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[1].color = 1;
//	
//	
//	//��Ԯ����־
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].graphic_name[0] = 3;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].graphic_name[1] = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].graphic_name[2] = 2;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].operate_tpye = 2;
//	if(rescue_card_flag == 0)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].color = 4;  //��Ԯ��û�д�ʱ��ʾΪ��ɫ
//	else if(rescue_card_flag == 1)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].color = 2;  //��Ԯ����ʱ��ʾΪ��ɫ
//	else
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].color = 1;   //�쳣ʱ��ʾΪ��ɫ
//	
//	
//	
//	//ץ�ֵ��λ��
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].graphic_name[0] = 3;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].graphic_name[1] = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].graphic_name[2] = 3;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].operate_tpye = 2;
//	if(rotate_motor_flag == 0)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].color = 4;  //ץ���ں������ʾΪ��ɫ
//	else if(rotate_motor_flag == 1)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].color = 2;  //ץ�ֵ�ǰ��ʱ��ʾΪ��ɫ
//	else
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].color = 1;   //�쳣ʱ��ʾΪ��ɫ
//	
//	
//	//ץ������
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].graphic_name[0] = 3;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].graphic_name[1] = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].graphic_name[2] = 4;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].operate_tpye = 2;
//	if(ore_grab_flag == 0)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].color = 4;  //ץ������û�д�ʱ��ʾΪ��ɫ
//	else if(ore_grab_flag == 1)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].color = 2;   //ץ�����״�ʱ��ʾΪ��ɫ
//	else
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].color = 1;  //�쳣ʱ��ʾΪ��ɫ
//	
//	append_crc8_check_sum(&message_car_collimator_3.FrameHead.sof,sizeof(message_car_collimator_3.FrameHead));
//	append_crc16_check_sum(&message_car_collimator_3.FrameHead.sof,sizeof(message_car_collimator_3.FrameHead)
//	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + message_car_collimator_3.FrameHead.dataLenth);
//	HAL_UART_Transmit(&JUDGE_HUART,&message_car_collimator_3.FrameHead.sof,sizeof(message_car_collimator_3),0xffff);
//}



//void init_arc(void)
//{
//	message_car_arc.FrameHead.sof = 0xA5;
//	message_car_arc.FrameHead.dataLenth = 36;
//	message_car_arc.FrameHead.seq = (seqcount++) & 0xFF;
//	message_car_arc.CmdId = 0x0301;                    
//	message_car_arc.Interactive_header_data.data_cmd_id = 0x0102;
//	message_car_arc.Interactive_header_data.sender_ID = robot_state.robot_id;                
//	message_car_arc.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;
//	
//	//ץ�ֵ������
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].graphic_name[0] = 4;
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].graphic_name[1] = 0;
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].graphic_name[2] = 0;
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].operate_tpye = 1;
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].graphic_tpye = 4;
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].layer = 1;
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].color = 4;  //ͼ�������ʾΪ��ɫ
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].start_angle = 135;
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].end_angle = 225;
//	if(ore_sucker_flag == 0)
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].color = 4;  //ͼ�������ʾΪ��ɫ
//	else if(ore_sucker_flag == 1)
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].color = 2;  //ͼ�������ʾΪ��ɫ
//	else
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].color = 1;//�����쳣ʱ��ʾΪ��ɫ
//	if(stretch_motor_flag == 2)//���̳���
//	{
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].start_angle = 300;
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].end_angle = 60;
//	}
//	else if(stretch_motor_flag == 1)//���̳�ǰ
//	{
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].start_angle = 30;
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].end_angle = 150;
//	}
//	else if(stretch_motor_flag == 0)//���̳���
//	{
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].start_angle = 120;
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].end_angle = 240;
//	}
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].width = 2;
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].start_x = 360;
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].start_y = 780;
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].end_x = 20;
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].end_y = 20;
//	
//	
//	//ͼ�����λ��
//	message_car_arc.Client_graphic_double.grapic_data_struct[1].graphic_name[0] = 4;
//	message_car_arc.Client_graphic_double.grapic_data_struct[1].graphic_name[1] = 0;
//	message_car_arc.Client_graphic_double.grapic_data_struct[1].graphic_name[2] = 1;
//	message_car_arc.Client_graphic_double.grapic_data_struct[1].operate_tpye = 1;
//	message_car_arc.Client_graphic_double.grapic_data_struct[1].graphic_tpye = 4;
//	message_car_arc.Client_graphic_double.grapic_data_struct[1].layer = 1;
//	message_car_arc.Client_graphic_double.grapic_data_struct[1].color = 2;  //ͼ�������ʾΪ��ɫ
//	message_car_arc.Client_graphic_double.grapic_data_struct[1].start_angle = 315;
//	message_car_arc.Client_graphic_double.grapic_data_struct[1].end_angle = 45;
//	if(gimbal_flag == 0)
//	{
//		message_car_arc.Client_graphic_double.grapic_data_struct[1].start_angle = 315;
//		message_car_arc.Client_graphic_double.grapic_data_struct[1].end_angle = 45;
//	}
//	else if(gimbal_flag == 1)
//	{
//		message_car_arc.Client_graphic_double.grapic_data_struct[1].start_angle = 225;
//		message_car_arc.Client_graphic_double.grapic_data_struct[1].end_angle = 315;
//	}
//	else
//	{
//		message_car_arc.Client_graphic_double.grapic_data_struct[1].start_angle = 135;
//		message_car_arc.Client_graphic_double.grapic_data_struct[1].end_angle = 225;
//	}
//	
//	message_car_arc.Client_graphic_double.grapic_data_struct[1].width = 2;
//	message_car_arc.Client_graphic_double.grapic_data_struct[1].start_x = 170;
//	message_car_arc.Client_graphic_double.grapic_data_struct[1].start_y = 740;
//	message_car_arc.Client_graphic_double.grapic_data_struct[1].end_x = 20;
//	message_car_arc.Client_graphic_double.grapic_data_struct[1].end_y = 20;
//	
//	
//	append_crc8_check_sum(&message_car_arc.FrameHead.sof,sizeof(message_car_arc.FrameHead));
//	append_crc16_check_sum(&message_car_arc.FrameHead.sof,sizeof(message_car_arc.FrameHead)
//	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + message_car_arc.FrameHead.dataLenth);
//	HAL_UART_Transmit(&JUDGE_HUART,&message_car_arc.FrameHead.sof,sizeof(message_car_arc),0xffff);
//}


////ץ�ֵ����ͼ�������־λ����
//void send_arc(void)
//{
//	message_car_arc.FrameHead.sof = 0xA5;
//	message_car_arc.FrameHead.dataLenth = 36;
//	message_car_arc.FrameHead.seq = (seqcount++) & 0xFF;
//	message_car_arc.CmdId = 0x0301;                    
//	message_car_arc.Interactive_header_data.data_cmd_id = 0x0102;
//	message_car_arc.Interactive_header_data.sender_ID = robot_state.robot_id;                
//	message_car_arc.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;
//	
//	//ץ�ֵ����־λ����
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].graphic_name[0] = 4;
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].graphic_name[1] = 0;
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].graphic_name[2] = 0;
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].operate_tpye = 2;
//	if(ore_sucker_flag == 0)
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].color = 4;  //ͼ�������ʾΪ��ɫ
//	else if(ore_sucker_flag == 1)
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].color = 2;  //ͼ�������ʾΪ��ɫ
//	else
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].color = 1;//�����쳣ʱ��ʾΪ��ɫ
//	if(stretch_motor_flag == 2)//���̳���
//	{
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].start_angle = 300;
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].end_angle = 60;
//	}
//	else if(stretch_motor_flag == 1)//���̳�ǰ
//	{
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].start_angle = 30;
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].end_angle = 150;
//	}
//	else if(stretch_motor_flag == 0)//���̳���
//	{
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].start_angle = 120;
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].end_angle = 240;
//	}
//	
//	
//	//ͼ�������־λ����
//	message_car_arc.Client_graphic_double.grapic_data_struct[1].graphic_name[0] = 4;
//	message_car_arc.Client_graphic_double.grapic_data_struct[1].graphic_name[1] = 0;
//	message_car_arc.Client_graphic_double.grapic_data_struct[1].graphic_name[2] = 1;
//	message_car_arc.Client_graphic_double.grapic_data_struct[1].operate_tpye = 2;
//	if(gimbal_flag == 0)
//	{
//		message_car_arc.Client_graphic_double.grapic_data_struct[1].start_angle = 315;
//		message_car_arc.Client_graphic_double.grapic_data_struct[1].end_angle = 45;
//	}
//	else if(gimbal_flag == 1)
//	{
//		message_car_arc.Client_graphic_double.grapic_data_struct[1].start_angle = 225;
//		message_car_arc.Client_graphic_double.grapic_data_struct[1].end_angle = 315;
//	}
//	else
//	{
//		message_car_arc.Client_graphic_double.grapic_data_struct[1].start_angle = 135;
//		message_car_arc.Client_graphic_double.grapic_data_struct[1].end_angle = 225;
//	}
//	
//	append_crc8_check_sum(&message_car_arc.FrameHead.sof,sizeof(message_car_arc.FrameHead));
//	append_crc16_check_sum(&message_car_arc.FrameHead.sof,sizeof(message_car_arc.FrameHead)
//	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + message_car_arc.FrameHead.dataLenth);
//	HAL_UART_Transmit(&JUDGE_HUART,&message_car_arc.FrameHead.sof,sizeof(message_car_arc),0xffff);
//}


////ɾ��ͼ��
//static void send_dele(uint8_t lay)
//{
//	message_dele.FrameHead.sof = 0xA5;
//	message_dele.FrameHead.dataLenth = 8;
//	message_dele.FrameHead.seq = (seqcount++) & 0xFF;
//	message_dele.CmdId = 0x0301;     
//	message_dele.Interactive_header_data.data_cmd_id = 0x0100;    
//	message_dele.Interactive_header_data.sender_ID = robot_state.robot_id;
//	message_dele.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;
//	message_dele.Client_Dele.operate_tpye = 1;
//	message_dele.Client_Dele.layer = lay;	
//	
//	append_crc8_check_sum(&message_dele.FrameHead.sof,sizeof(message_dele.FrameHead));
//	append_crc16_check_sum(&message_dele.FrameHead.sof,sizeof(message_dele.FrameHead)
//	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + message_dele.FrameHead.dataLenth);
//	HAL_UART_Transmit(&JUDGE_HUART,&message_dele.FrameHead.sof,sizeof(message_dele),0xffff);
//}


////�����ַ������ڲ���
//void send_character(void)
//{
//	message_character.FrameHead.sof = 0xA5;
//	message_character.FrameHead.dataLenth = 51;
//	message_character.FrameHead.seq = (seqcount++) & 0xFF;                 
//	message_character.Interactive_header_data.data_cmd_id = 0x0110;
//	message_character.Interactive_header_data.sender_ID = robot_state.robot_id;
//	message_character.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;
//	message_character.Client_character.grapic_data_struct.graphic_name[0] = 0;
//	message_character.Client_character.grapic_data_struct.graphic_name[1] = 0;
//	message_character.Client_character.grapic_data_struct.graphic_name[2] = 3; 
//	message_character.Client_character.grapic_data_struct.operate_tpye = 1;
//	message_character.Client_character.grapic_data_struct.graphic_tpye = 7;
//	message_character.Client_character.grapic_data_struct.layer = 0;
//	message_character.Client_character.grapic_data_struct.color = 0;
//	message_character.Client_character.grapic_data_struct.start_angle = 20;
//	message_character.Client_character.grapic_data_struct.end_angle = 1;
//	message_character.Client_character.grapic_data_struct.width = 5;
//	message_character.Client_character.grapic_data_struct.start_x = 400;
//	message_character.Client_character.grapic_data_struct.start_y = 600;
//	message_character.Client_character.grapic_data_struct.radius = 0;
//	message_character.Client_character.grapic_data_struct.end_x = 0;
//	message_character.Client_character.grapic_data_struct.end_y = 0;
//	message_character.Client_character.data[0] = 'a';
//	
//	append_crc8_check_sum(&message_character.FrameHead.sof,sizeof(message_character.FrameHead));
//	append_crc16_check_sum(&message_character.FrameHead.sof,sizeof(message_character.FrameHead)
//	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + message_character.FrameHead.dataLenth);
//	HAL_UART_Transmit(&JUDGE_HUART,&message_character.FrameHead.sof,sizeof(message_character),0xffff);
//}
////���͸����������ڲ���
//void renew_float(void)
//{
//	message_onefloat.FrameHead.sof = 0xA5;
//	message_onefloat.FrameHead.dataLenth = 21;
//	message_onefloat.FrameHead.seq = (seqcount++) & 0xFF;  
//	message_onefloat.CmdId = 0x0301;  
//	message_onefloat.Interactive_header_data.data_cmd_id = 0x0101;
//	message_onefloat.Interactive_header_data.sender_ID = robot_state.robot_id;
//	message_onefloat.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;
//	message_onefloat.Client_float_single.graphic_name[0] = 0;
//	message_onefloat.Client_float_single.graphic_name[1] = 0;
//	message_onefloat.Client_float_single.graphic_name[2] = 4;
//	message_onefloat.Client_float_single.operate_tpye = 1;
//	message_onefloat.Client_float_single.graphic_tpye = 5;
//	message_onefloat.Client_float_single.layer = 0;
//	message_onefloat.Client_float_single.color = 0;
//	message_onefloat.Client_float_single.start_angle = 50;
//	message_onefloat.Client_float_single.end_angle = 1;
//	message_onefloat.Client_float_single.width = 5;
//	message_onefloat.Client_float_single.start_x = 400;
//	message_onefloat.Client_float_single.start_y = 900; 
//	message_onefloat.Client_float_single.num = 2.0f;
//	
//	append_crc8_check_sum(&message_onefloat.FrameHead.sof,sizeof(message_onefloat.FrameHead));
//	append_crc16_check_sum(&message_onefloat.FrameHead.sof,sizeof(message_onefloat.FrameHead)
//	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + message_onefloat.FrameHead.dataLenth);
//	HAL_UART_Transmit(&JUDGE_HUART,&message_onefloat.FrameHead.sof,sizeof(message_onefloat),0xffff);
//}

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

//UI_message_t 	 	UI_message;				//UI图形结构体  //底盘模式 光电开关线X2  底盘轮廓 状态文字	3未使用	
//graphic_seven_t message_car_collimator_1; 	// 抬矿气缸、夹矿气缸状态线 								6未使用
//graphic_seven_t message_car_collimator_2;	//一级、二级抬升状态线									
//graphic_five_t message_car_collimator_3;	//车的救援抓手以及救援卡、抓手电机、气动状态线
//graphic_double_t message_car_arc; 			//车的抓手以及图传舵机状态图形


//UI_message_t 	 	UI_message_float;	//UI数字结构体
//character_t    		message_character;	//字符
//graphic_dele_t 		message_dele;		//删除图层
//float_single_t 		message_onefloat;	//浮点数


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


////UI任务
//void ui_task(void const *argu)
//{	
//	osDelay(1000);
//	//初始化
//	init_message(&UI_message,&UI_message_float);
//	init_collimator_1();
//	init_collimator_2();
//	init_collimator_3();
//	init_arc();

//	uint32_t UI_wake_time = osKernelSysTick();	
//	
//	while(1)
//	{
//		//读取光电开关标志位
//		opto_switch1_flag = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
//		opto_switch2_flag = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
//		//为防止UI界面出现故障，每隔5s进行一次清屏
//		clear_time++;
//		if(clear_time == 30)
//		{
//			send_dele(0);
//			send_dele(1);
//			//清屏后再进入初始化
//			init_message(&UI_message, &UI_message_float);
//			init_collimator_1();
//			init_collimator_2();
//			init_collimator_3();
//			init_arc();
//			osDelay(UI_TIME);
//			clear_time = 0;
//		}
//		
//		//更新UI界面内容
//		send_collimator_1();
//		send_collimator_2();
//		send_collimator_3();
//		send_arc();
//		send_message(&UI_message,&UI_message_float);
//		osDelay(UI_TIME);
//	}
//}
////底盘模式 光电开关线 控制模式 底盘轮廓 状态文字
//static void init_message(UI_message_t *UI_message, UI_message_t *UI_message_float)
//{
//	
//	UI_message->graphic_five.FrameHead.sof = 0xA5;		
//	UI_message->graphic_five.FrameHead.dataLenth = 81;
//	UI_message->graphic_five.FrameHead.seq = (seqcount++) & 0xFF;  									//帧头填充
//	UI_message->graphic_five.CmdId = 0x0301;                     									//交互命令码
//	UI_message->graphic_five.Interactive_header_data.data_cmd_id = 0x0103;   						//UI图形命令码
//	UI_message->graphic_five.Interactive_header_data.sender_ID = robot_state.robot_id;				//机器人发送ID
//	UI_message->graphic_five.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;	//机器人接收ID
//	
//	//底盘模式指示灯
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[0] = 0;			//图形ID，不要重复！！
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[1] = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[2] = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].operate_tpye = 1;			//行为代号
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_tpye = 2;			//图形类型代号
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].layer = 1;					//图层代号
//	if(chassis_behaviour_flag == 0)
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 4;
//	else if(chassis_behaviour_flag == 1)
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 2;
//	else if(chassis_behaviour_flag == 2)
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 3;
//	else
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 7;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_angle = 0;				//起始角度	
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_angle = 0;				//终止角度
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].width = 5;					//线宽
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_x = 1820;				//x起点坐标
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_y = 830;				//y起点坐标
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].radius = 15;					//圆半径
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_x = 0;					//x终点坐标
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_y = 0;					//y终点坐标
//	
//	
//	//光电开关1
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[0] = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[1] = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[2] = 1;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].operate_tpye = 1;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_tpye = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].layer = 1;
//	if(opto_switch1_flag == 0)
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 4;  //前方检测到有障碍时显示紫色
//	else if(opto_switch1_flag == 1)
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 2;  //前方检测没有障碍时显示绿色
//	else
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 1;   //异常时显示黑色
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].width = 2;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].start_x = 760;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].start_y = 540;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].end_x = 760;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].end_y = 270;
//	
//	
//	//光电开关2
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
//	//控制模式标志位
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].graphic_name[0] = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].graphic_name[1] = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].graphic_name[2] = 3;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].operate_tpye = 0;
//	
//	
//	//底盘轮廓
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
//	//底盘状态文字
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
//	//控制模式状态文字
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
//	//控制模式标志位 123为对应模式 0为异常
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

////更新底盘状态 光电开关 控制模式标志位及数据
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
//	//底盘状态标志位更新
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
//	//前方光电开关标志位更新
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[0] = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[1] = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[2] = 1;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].operate_tpye = 2;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_tpye = 0;
//	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].layer = 1;
//	if(opto_switch1_flag == 0)
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 4;  //前方检测到有障碍时显示紫色
//	else if(opto_switch1_flag == 1)
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 2;  //前方检测没有障碍时显示绿色
//	else
//		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 1;   //异常时显示黑色
//	
//	
//	
//	//后方光电开关志位更新
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
//	//控制模式标志位更新
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

////发送工程障碍块舵机状态线 抬矿、夹矿气缸的状态直线对其进行初始化
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
//	//线0到线3为工程障碍块舵机状态线，线4为夹矿气缸状态线，线5为抬矿气缸状态线 6未使用
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
//	//舵机第二靠下的线
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
//	//舵机抬起障碍块以后的线
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
//	//舵机收回以后的线
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
//	//抬矿气缸
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].graphic_name[0] = 1;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].graphic_name[1] = 0;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].graphic_name[2] = 4;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].operate_tpye = 1;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].graphic_tpye = 0;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].layer = 1;
////	if(ore_store2_flag == 0)
////		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].color = 4;  //抬矿气缸关闭时显示紫色
////	else if(ore_store2_flag == 1)
////		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].color = 2;   //抬矿气缸打开时显示绿色
////	else
////		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].color = 1;    //程序异常时显色黄色
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].width = 2;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].start_x = 220;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].start_y = 670;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].end_x = 220;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].end_y = 750;
//		
//	
//	//夹矿气缸
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].graphic_name[0] = 1;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].graphic_name[1] = 0;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].graphic_name[2] = 5; 
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].operate_tpye = 1;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].graphic_tpye = 0;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].layer = 1;
////	if(ore_store1_flag == 0)
////		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].color = 4;  //夹矿气缸关闭时显示紫色
////	else if(ore_store1_flag == 1)
////		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].color = 2;  //夹矿气缸打开时显示绿色
////	else
////		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].color = 1;   //异常时显示黄色	
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


////发送直线对车的夹矿、抬矿气缸状态线进行更新
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
//	//线0到线3为工程障碍块舵机状态线，线4为夹矿气缸状态线，线5为抬矿气缸状态线 6未使用
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
//	//舵机第二靠下的线
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
//	//舵机抬起障碍块以后的线
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
//	//舵机收回以后的线
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
//	//抬矿气缸
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].graphic_name[0] = 1;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].graphic_name[1] = 0;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].graphic_name[2] = 4;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].operate_tpye = 2;
////	if(ore_store2_flag == 0)
////		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].color = 4;  //抬矿气缸关闭时显示紫色
////	else if(ore_store2_flag == 1)
////		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].color = 2;   //抬矿气缸打开时显示绿色
////	else
////		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[4].color = 1;    //程序异常时显色黄色

////	
////	//夹矿气缸
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].graphic_name[0] = 1;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].graphic_name[1] = 0;
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].graphic_name[2] = 5; 
////	message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].operate_tpye = 2;
////	if(ore_store1_flag == 0)
////		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].color = 4;  //夹矿气缸关闭时显示紫色
////	else if(ore_store1_flag == 1)
////		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].color = 2;  //夹矿气缸打开时显示绿色
////	else
////		message_car_collimator_1.Client_graphic_seven.grapic_data_struct[5].color = 1;   //异常时显示黄色		

//	append_crc8_check_sum(&message_car_collimator_1.FrameHead.sof,sizeof(message_car_collimator_1.FrameHead));
//	append_crc16_check_sum(&message_car_collimator_1.FrameHead.sof,sizeof(message_car_collimator_1.FrameHead)
//	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + message_car_collimator_1.FrameHead.dataLenth);
//	HAL_UART_Transmit(&JUDGE_HUART,&message_car_collimator_1.FrameHead.sof,sizeof(message_car_collimator_1),0xffff);
//}

////		线0为一级抬升气缸状态线，线1为二级抬升气缸状态线
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

//	//一级抬升气缸线
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].graphic_name[0] = 2;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].graphic_name[1] = 0;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].graphic_name[2] = 0;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].operate_tpye = 1;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].graphic_tpye = 0;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].layer = 1;
////	if(ore_lift1_flag == 0)
////		message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].color = 4;  //一级抬升气缸关闭时显示紫色
////	else if(ore_lift1_flag == 1)
////		message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].color = 2;   //一级抬升气缸打开时显示绿色
////	else
////		message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].color = 1;    //程序异常时显色黄色
////	
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].width = 2;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].start_x = 260;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].start_y = 670;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].end_x = 360;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].end_y = 670;
////		
////	//二级抬升气缸线
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].graphic_name[0] = 2;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].graphic_name[1] = 0;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].graphic_name[2] = 1; 
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].operate_tpye = 1;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].graphic_tpye = 0;
////	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].layer = 1;
////	if(ore_store2_flag == 0)
////		message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].color = 4;  //二级抬升气缸关闭时显示紫色
////	else if(ore_store2_flag == 1)
////		message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].color = 2;  //二级抬升气缸打开时显示绿色
////	else
////		message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].color = 1;   //异常时显示黄色
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


////更新规则与初始化规则相同
////	线0为一级抬升气缸状态线，线1为二级抬升气缸状态线
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
//	//障碍块转到对应的位置对应的图形变为绿色，其余三条为紫色
//	//舵机最下边位置的线
//		
//	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].graphic_name[0] = 2;
//	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].graphic_name[1] = 0;
//	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].graphic_name[2] = 4;
//	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].operate_tpye = 2;
//	if(ore_lift1_flag == 0)
//		message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].color = 4;  //一级抬升气缸关闭时显示紫色
//	else if(ore_lift1_flag == 1)
//		message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].color = 2;   //一级抬升气缸打开时显示绿色
//	else
//		message_car_collimator_2.Client_graphic_seven.grapic_data_struct[0].color = 1;    //程序异常时显色黄色

//	//夹矿气缸
//	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].graphic_name[0] = 2;
//	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].graphic_name[1] = 0;
//	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].graphic_name[2] = 5; 
//	message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].operate_tpye = 2;
//	if(ore_lift2_flag == 0)
//		message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].color = 4;  //夹矿气缸关闭时显示紫色
//	else if(ore_lift2_flag == 1)
//		message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].color = 2;  //夹矿气缸打开时显示绿色
//	else
//		message_car_collimator_2.Client_graphic_seven.grapic_data_struct[1].color = 1;   //异常时显示黄色
//	
//	append_crc8_check_sum(&message_car_collimator_2.FrameHead.sof,sizeof(message_car_collimator_2.FrameHead));
//	append_crc16_check_sum(&message_car_collimator_2.FrameHead.sof,sizeof(message_car_collimator_2.FrameHead)
//	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + message_car_collimator_2.FrameHead.dataLenth);
//	HAL_UART_Transmit(&JUDGE_HUART,&message_car_collimator_2.FrameHead.sof,sizeof(message_car_collimator_2),0xffff);
//}


////救援抓手X2、救援卡、抓手电机、气动的线初始化函数
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
//	//救援抓手的线1
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].graphic_name[0] = 3;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].graphic_name[1] = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].graphic_name[2] = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].operate_tpye = 1;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].graphic_tpye = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].layer = 1;
//	if(rescue_hand_flag == 0)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].color = 4;  //救援抓手关闭时显示为紫色
//	else if(rescue_hand_flag == 1)
//	  message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].color = 2;   //救援抓手打开时显示为绿色
//	else
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].color = 1;   //异常时显示为黄色
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].width = 2;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].start_x = 120;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].start_y = 830;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].end_x = 120;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].end_y = 870;
//	
//	
//	
//	//救援抓手的线2，功能作用与1相同
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
//	//救援卡标志
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].graphic_name[0] = 3;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].graphic_name[1] = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].graphic_name[2] = 2;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].operate_tpye = 1;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].graphic_tpye = 2;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].layer = 1;
//	if(rescue_card_flag == 0)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].color = 4;  //救援卡没有打开时显示为紫色
//	else if(rescue_card_flag == 1)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].color = 2;  //救援卡打开时显示为绿色
//	else
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].color = 1;   //异常时显示为黄色
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].width = 2;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].start_x = 175;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].start_y = 840;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].radius = 5;
//	
//	
//	//抓手电机位置
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].graphic_name[0] = 3;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].graphic_name[1] = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].graphic_name[2] = 3;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].operate_tpye = 1;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].graphic_tpye = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].layer = 1;
//	if(rotate_motor_flag == 0)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].color = 4;  //抓手在后边是显示为紫色
//	else if(rotate_motor_flag == 1)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].color = 2;  //抓手到前边时显示为绿色
//	else
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].color = 1;   //异常时显示为黄色	
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].width = 2;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].start_x = 280;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].start_y = 800;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].end_x = 280;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].end_y = 760;
//	
//	
//	
//	//抓手气缸位置
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].graphic_name[0] = 3;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].graphic_name[1] = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].graphic_name[2] = 4;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].operate_tpye = 1;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].graphic_tpye = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].layer = 1;
//	if(ore_grab_flag == 0)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].color = 4;  //抓手气缸没有打开时显示为紫色
//	else if(ore_grab_flag == 1)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].color = 2;   //抓手气缸打开时显示为绿色
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


////标志位更新
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
//	//救援抓手的线1
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].graphic_name[0] = 3;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].graphic_name[1] = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].graphic_name[2] = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].operate_tpye = 2;
//	if(rescue_hand_flag == 0)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].color = 4;  //救援抓手关闭时显示为紫色
//	else if(rescue_hand_flag == 1)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].color = 2;   //救援抓手打开时显示为绿色
//	else
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[0].color = 1;   //异常时显示为黄色	}
//	
//	//救援抓手的线2，功能作用与1相同
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
//	//救援卡标志
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].graphic_name[0] = 3;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].graphic_name[1] = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].graphic_name[2] = 2;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].operate_tpye = 2;
//	if(rescue_card_flag == 0)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].color = 4;  //救援卡没有打开时显示为紫色
//	else if(rescue_card_flag == 1)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].color = 2;  //救援卡打开时显示为绿色
//	else
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[2].color = 1;   //异常时显示为黄色
//	
//	
//	
//	//抓手电机位置
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].graphic_name[0] = 3;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].graphic_name[1] = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].graphic_name[2] = 3;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].operate_tpye = 2;
//	if(rotate_motor_flag == 0)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].color = 4;  //抓手在后边是显示为紫色
//	else if(rotate_motor_flag == 1)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].color = 2;  //抓手到前边时显示为绿色
//	else
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[3].color = 1;   //异常时显示为黄色
//	
//	
//	//抓手气缸
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].graphic_name[0] = 3;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].graphic_name[1] = 0;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].graphic_name[2] = 4;
//	message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].operate_tpye = 2;
//	if(ore_grab_flag == 0)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].color = 4;  //抓手气缸没有打开时显示为紫色
//	else if(ore_grab_flag == 1)
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].color = 2;   //抓手气缸打开时显示为绿色
//	else
//		message_car_collimator_3.Client_graphic_five.grapic_data_struct[4].color = 1;  //异常时显示为黄色
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
//	//抓手电机方向
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].graphic_name[0] = 4;
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].graphic_name[1] = 0;
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].graphic_name[2] = 0;
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].operate_tpye = 1;
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].graphic_tpye = 4;
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].layer = 1;
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].color = 4;  //图传舵机显示为紫色
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].start_angle = 135;
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].end_angle = 225;
//	if(ore_sucker_flag == 0)
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].color = 4;  //图传舵机显示为紫色
//	else if(ore_sucker_flag == 1)
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].color = 2;  //图传舵机显示为绿色
//	else
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].color = 1;//程序异常时显示为黄色
//	if(stretch_motor_flag == 2)//吸盘朝上
//	{
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].start_angle = 300;
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].end_angle = 60;
//	}
//	else if(stretch_motor_flag == 1)//吸盘朝前
//	{
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].start_angle = 30;
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].end_angle = 150;
//	}
//	else if(stretch_motor_flag == 0)//吸盘朝下
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
//	//图传舵机位置
//	message_car_arc.Client_graphic_double.grapic_data_struct[1].graphic_name[0] = 4;
//	message_car_arc.Client_graphic_double.grapic_data_struct[1].graphic_name[1] = 0;
//	message_car_arc.Client_graphic_double.grapic_data_struct[1].graphic_name[2] = 1;
//	message_car_arc.Client_graphic_double.grapic_data_struct[1].operate_tpye = 1;
//	message_car_arc.Client_graphic_double.grapic_data_struct[1].graphic_tpye = 4;
//	message_car_arc.Client_graphic_double.grapic_data_struct[1].layer = 1;
//	message_car_arc.Client_graphic_double.grapic_data_struct[1].color = 2;  //图传舵机显示为绿色
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


////抓手电机，图传舵机标志位更新
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
//	//抓手电机标志位更新
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].graphic_name[0] = 4;
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].graphic_name[1] = 0;
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].graphic_name[2] = 0;
//	message_car_arc.Client_graphic_double.grapic_data_struct[0].operate_tpye = 2;
//	if(ore_sucker_flag == 0)
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].color = 4;  //图传舵机显示为紫色
//	else if(ore_sucker_flag == 1)
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].color = 2;  //图传舵机显示为绿色
//	else
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].color = 1;//程序异常时显示为黄色
//	if(stretch_motor_flag == 2)//吸盘朝上
//	{
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].start_angle = 300;
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].end_angle = 60;
//	}
//	else if(stretch_motor_flag == 1)//吸盘朝前
//	{
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].start_angle = 30;
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].end_angle = 150;
//	}
//	else if(stretch_motor_flag == 0)//吸盘朝下
//	{
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].start_angle = 120;
//		message_car_arc.Client_graphic_double.grapic_data_struct[0].end_angle = 240;
//	}
//	
//	
//	//图传舵机标志位更新
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


////删除图层
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


////发送字符，用于测试
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
////发送浮点数，用于测试
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

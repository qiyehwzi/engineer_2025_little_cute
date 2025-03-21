#include "arm_control_task.h"
#include "chassis_task.h"
#include "CAN_receive.h"
#include "keyboard.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "referee.h"
#include "bsp_dwt.h"
#include "math.h"
#include "gpio.h"

static void arm_control_init(all_key_t *arm_control_key_init, Robotic_6DOF_control_t *R_6D_ctrl);
static void arm_control_key_check(all_key_t *arm_control_key_check);
static void arm_feedback_update( arm_control_t *arm_control_position, Robotic_6DOF_control_t *R_6D_ctrl);
static void arm_control_set( arm_control_t *arm_control_set, all_key_t * arm_key);
static void arm_check_get_position(arm_control_t *check_position);
static void arm_control_loop(Robotic_6DOF_control_t *R_6D_ctrl,arm_control_t *arm_control_loop, all_key_t *arm_key); 

arm_control_t arm_control;
Robotic_6DOF_control_t R_6D_ctrl;
Ag_Catch_t Ag_Catch;

float max_joint_speed[6];
Joint6D_t Last_Joint6D;
uint8_t suker_key_flag;
int G_flag;
float32_t yaw_angle_set;

void arm_control_task(void const * argument)
{
		arm_control_init(&all_key,&R_6D_ctrl);
		Joint6D_Init(&R_6D_ctrl.Robotic_6D);
		
		uint32_t system_clock = osKernelSysTick();
		while(1)
		{
				arm_control.dt = DWT_GetDeltaT(&arm_control.DWT_Count);
				
				arm_control_key_check(&all_key);
				arm_feedback_update(&arm_control,&R_6D_ctrl);
				arm_control_set(&arm_control, &all_key);
				arm_control_loop(&R_6D_ctrl, &arm_control,&all_key);
				osDelay(2);
		}
}

void arm_control_init(all_key_t *arm_control_key_init, Robotic_6DOF_control_t *R_6D_ctrl)
{
		//机械臂动作初始化
	
//		//上面银矿
//		fp32 Ag1_temp[11][7] = {{9.1f,18.0f,1.57f,-1.57f,0.0f,-1.67f + 2.13f,0.0f},        //末端向下
//														{4.1f,18.0f,1.57f,-1.57f,0.0f,-1.67f + 2.13f,0.0f},        //抬起
//														{4.1f,44.0f,1.57f,-1.57f,0.0f,-1.67f + 2.13f,0.0f},        //伸出
//														{5.8f,44.0f,1.57f,-1.57f,0.0f,-1.67f + 2.13f,0.0f},        //放下吸矿
//														{1.5f,44.0f,1.57f,-1.57f,0.0f,-1.67f + 2.13f,0.0f},        //提起
//														{1.5f,40.77f,1.57f,-1.57f,0.0f,-1.67f + 2.13f,0.0f},       //后退
//														{1.5f,40.77f,1.38f,1.59f,0.0f,-1.67f + 2.13f,0.0f},        //放到第一个位置上方
//														{1.5f,40.77f,1.38f,1.59f,0.0f,-1.67f + 2.13f,1.45f+0.5f},  //转矿  
//														{8.5f,40.77f,1.38f,1.59f,0.0f,-1.67f + 2.13f,1.45f+0.5f},  //放矿
//														{4.5f,40.77f,1.38f,1.59f,0.0f,-1.67f + 2.13f,1.45f+0.5f},  //抬起
//														{9.1f,18.0f,1.57f,-1.57f,0.0f,-1.67f + 2.13f,0.0f} };      //复位
//																	
//		uint16_t flag_and_time_1[11][2] = {{0,50},{0,100},{0,100},{1,1000},{1,500},{1,100},{1,500},{1,300},{1,200},{0,100},{0,100}};
	
		//上面银矿点位左
			fp32 Ag1_temp[10][7] = {
														{7.8f + LIFT_MIGRATION ,5.10f,0.0f,0.0f,0.0f,2.13f,0.0f},
														{7.8f + LIFT_MIGRATION,45.0f,0.00f,1.57f,0.0f,3.70f,0.0f},
														{17.0f + LIFT_MIGRATION,45.0f,0.00f,1.57f,0.025f,3.70f,0.0f},
														{19.0f + LIFT_MIGRATION,45.0f,0.00f,1.57f,0.025f,3.70f,0.0f},
														{7.9f + LIFT_MIGRATION,45.5f,0.00f,1.57f,0.025f,3.70f,0.0f},
														{7.9f + LIFT_MIGRATION,42.5f,1.38f,1.59f,0.0f,3.70f,+0.3f},    
														{7.9f + LIFT_MIGRATION,37.8f,1.38f,1.59f,0.0f,3.70f,+0.3f},
														{7.9f + LIFT_MIGRATION,37.8f,1.38f,1.59f,0.0f,3.70f,+0.3f},
														{7.9f + LIFT_MIGRATION,19.0f,0.0f,1.59f,0.0f,2.13f,+0.3f}, 	
														{7.8f + LIFT_MIGRATION,5.10f,0.0f,0.0f,0.0f,2.13f,0.0f},
														};
		
		uint16_t flag_and_time_1[10][2] = {{0,50},{1,200},{1,1000},{1,500},{1,400},{1,300},{1,150},{0,1000},{0,200},{0,100}};
		Ag_Catch.Ag1 = 	Ag1_temp[3][0];
		memcpy(arm_control.arm_move_routine.Ag1,Ag1_temp, sizeof(Ag1_temp));	
		memcpy(arm_control.arm_move_routine.flag_and_time[0], flag_and_time_1,sizeof(flag_and_time_1));
		
		//上面银矿点位
//		fp32 Ag2_temp[12][7] = {{9.1f,16.0f,1.57f,-1.57f,0.0f,-1.67f + 2.13f,0.0f},			//复位姿态、末端朝下
//														{4.1f,16.0f,1.57f,-1.57f,0.0f,-1.67f + 2.13f,0.0f},			//抬起
//														{4.1f,20.0f,0.57f,-1.26f,0.0f,-1.67f + 2.13f,0.0f},        //收臂、伸出
//														{4.1f,28.5f,0.57f,-1.26f,0.025f,0.426f			,0.0f},     //移到矿上方
//														{5.8f,28.5f,0.57f,-1.26f,0.025f,0.426f			,0.0f},     //抓矿
//														{1.5f,30.5f,0.57f,-1.26f,0.025f,0.426f			,0.0f},     //提起
//														{1.5f,20.0f,0.0f,0.0f,0.0f,-1.67f + 2.13f,0.0f},        //收臂
//														{1.5f,18.0f,1.7f,-1.53f,0.0f,-1.67f + 2.13f, 0.0f},     //放到第二个位置上方
//														{1.5f,18.0f,1.7f,-1.53f,0.0f,-1.67f + 2.13f, 0.7f},     //转矿
//														{8.5f,18.0f,1.83f,-1.75f,0.07f,-1.67f + 2.13f,0.7f},    //放矿
//														{8.5f,16.0f,1.83f,-1.75f,0.07f,-1.67f + 2.13f,0.7f},    //夹矿
//														{9.1f,18.0f,1.57f,-1.57f,0.0f, + 2.13f,0.0f} };   //复位                                                       
//														
//													
//		
//		uint16_t flag_and_time_2[12][2] = {{0,50},{0,100},{0,100},{0,500},{1,500},{1,300},{1,100},{1,500},{1,100},{1,200},{1,1000},{0,200}};
		
		//上面银矿点位中
		fp32 Ag2_temp[11][7] = {
														{7.8f + LIFT_MIGRATION,5.10f,0.0f,0.0f,0.0f,2.13f,0.0f}, 			
														{7.8f + LIFT_MIGRATION,5.10f,0.0f,0.0f,0.0f,3.8f,0.0f},  			
														{17.0f + LIFT_MIGRATION,20.0f,0.0f,0.0f,0.0f,3.8f,0.0f},				
														{19.0f + LIFT_MIGRATION,20.0f,0.0f,0.0f,0.0f,3.8f,0.0f},				
														{9.4f + LIFT_MIGRATION,20.0f,0.0f,0.0f,0.0f,3.8f,0.0f},				
														{9.4f + LIFT_MIGRATION,20.0f,0.0f,1.57f,0.0f,3.8f,0.0f},	     
														{9.4f + LIFT_MIGRATION,20.0f,0.0f,1.57f,0.0f,3.8f,0.0f},			
														{9.4f + LIFT_MIGRATION,12.0f,0.0f,1.57f,0.0f,3.8f,0.0f},			
														{9.4f + LIFT_MIGRATION,12.0f,0.0f,1.57f,0.0f,3.8f,0.0f},			
														{9.4f + LIFT_MIGRATION,12.0f,0.0f,1.57f,0.0f,3.8f,0.0f},			
														{7.8f + LIFT_MIGRATION,5.10f,0.0f,0.0f,0.0f,2.13f,0.0f},};			
		
		uint16_t flag_and_time_2[11][2] = {{0,50},{1,200},{1,900},{1,200},{1,200},{1,400},{1,500},{1,200},{0,1000},{0,200},{0,500}};
		
		memcpy(arm_control.arm_move_routine.Ag2,Ag2_temp,sizeof(Ag2_temp));
		memcpy(arm_control.arm_move_routine.flag_and_time[1], flag_and_time_2,sizeof(flag_and_time_2));
		Ag_Catch.Ag2 = 	Ag2_temp[3][0];									
		//上面银矿点位
//		fp32 Ag3_temp[10][7] = {{9.1f,18.0f,1.57f,-1.57f,0.0f,-1.67f + 2.13f,0.0f},  
//														{4.1f,30.0f,0.5f,-1.52f,0.025f,-1.67f + 2.13f,0.0f},
//														{4.1f,45.2f,0.0f,-1.62f,0.025f,-1.67f + 2.13f,0.0f},
//														{5.8f,45.2f,0.0f,-1.62f,0.025f,-1.67f + 2.13f,0.0f},
//														{1.5f,45.2f,0.0f,-1.62f,0.025f,-1.67f + 2.13f,0.0f},
//														{1.5f,30.2f,1.57f,-1.62f,0.025f,-1.67f + 2.13f,0.0f},
//														{1.5f,18.0f,1.57f,-1.57f,0.0f,1.67f + 2.13f,0.0f},
//														{9.1f,18.0f,1.57f,-1.57f,0.0f,1.67f + 2.13f,0.0f},

//		};	
//		
//		uint16_t flag_and_time_3[10][2] = {{0,50},{0,100},{0,100},{1,1000},{1,400},{1,300},{1,2000},{1,4000}};
		
		//侧面银矿点位
		fp32 Ag3_temp[6][7] = {	{5.7f + LIFT_MIGRATION,5.10f,0.0f,0.0f,0.0f,2.13f,0.0f}, 			
														{5.7f + LIFT_MIGRATION,5.10f,0.0f,0.0f,0.0f,3.70f,0.0f},  			
														{5.7f + LIFT_MIGRATION,20.0f,0.0f,0.0f,0.0f,3.70f,0.0f},				
														{7.6f + LIFT_MIGRATION,20.0f,0.0f,0.0f,0.0f,3.70f,0.0f},				
														{3.9f + LIFT_MIGRATION,20.0f,0.0f,0.0f,0.0f,3.70f,0.0f},				
														{3.9f + LIFT_MIGRATION,5.10f,0.0f,0.0f,0.0f,2.13f,0.0f}};
		
		uint16_t flag_and_time_3[6][2] = {{0,50},{1,200},{1,200},{1,1300},{1,800},{1,200}};
		
		
		memcpy(arm_control.arm_move_routine.Ag3,Ag3_temp,sizeof(Ag3_temp));
		memcpy(arm_control.arm_move_routine.flag_and_time[2], flag_and_time_3,sizeof(flag_and_time_3));
		Ag_Catch.Ag3 = 	Ag3_temp[3][0];													
		fp32 exchange1_temp[5][7] = {{9.4f + LIFT_MIGRATION,32.0f,0.0f,0.0f,0.0f,2.13f,0.0f},
																{14.71f + LIFT_MIGRATION,32.0f,0.0f,1.68f,-1.735f,0.44f,0.0f},
																{14.71f + LIFT_MIGRATION,24.87f,0.0f,1.68f,-1.735f,0.44f,0.0f},
																{9.4f + LIFT_MIGRATION,37.37f,0.0f,1.68f,-1.735f,0.44f,0.0f},
																{9.4f + LIFT_MIGRATION,37.37f,0.0f,0.0f,-1.735f,2.13f,0.0f}
																};
		
		uint16_t flag_and_time_4[5][2] = {{1,50},{1,50},{1,200},{1,100},{1,500}};
		
		memcpy(arm_control.arm_move_routine.exchange1,exchange1_temp,140);
		memcpy(arm_control.arm_move_routine.flag_and_time[3], flag_and_time_4,40);
		
		fp32 exchange2_temp[5][7] = {{9.4f + LIFT_MIGRATION,32.0f,0.0f,0.0f,0.0f,2.13f,0.0f},
																{14.71f + LIFT_MIGRATION,32.0f,0.0f,1.68f,-1.735f,0.44f,0.0f},
																{14.71f + LIFT_MIGRATION,3.6f,0.0f,1.68f,-1.735f,0.44f,0.0f},
																{10.5f + LIFT_MIGRATION,37.37f,0.0f,1.68f,-1.735f,0.44f,0.0f},
																{10.5f + LIFT_MIGRATION,32.0f,0.0f,0.0f,-1.735f,2.13f,0.0f},};
		
		uint16_t flag_and_time_5[5][2] = {{1,50},{1,50},{1,200},{1,100},{1,500}};
		
		memcpy(arm_control.arm_move_routine.exchange2,exchange2_temp,140);
		memcpy(arm_control.arm_move_routine.flag_and_time[4], flag_and_time_5,40);

		fp32 Au1_temp[10][7] = {{15.7f + LIFT_MIGRATION, 30.0f, 0.0f,  0.0f, 0.0f, 2.13f, 0.0f},
														{15.7f + LIFT_MIGRATION, 30.0f, 0.0f,  0.0f, 0.0f, 2.13f - 0.10f, 0.0f},//0.52是抓到矿以后补偿锥齿轮虚位
														{14.0f + LIFT_MIGRATION, 30.0f, 0.0f,  0.0f, 0.0f, 2.13f - 0.10f, 0.0f},
														{	4.5f + LIFT_MIGRATION, 30.0f, 0.0f,  0.0f, 0.0f, 2.13f - 0.10f, 0.0f},
														{ 4.5f + LIFT_MIGRATION,30.2f,1.57f,-1.62f,0.025f, 1.57f + 2.13f, 0.0f},
														{ 4.5f + LIFT_MIGRATION,40.97f,1.38f,1.59f,0.0f,1.57f + 2.13f,0.0f},
														{ 7.1f + LIFT_MIGRATION,40.97f,1.38f,1.59f,0.0f,1.57f + 2.13f,0.1f},
														{ 9.1f + LIFT_MIGRATION,40.97f,1.38f,1.59f,0.0f,1.57f + 2.13f,0.1f},
														{ 4.5f + LIFT_MIGRATION,40.97f,1.38f,1.59f,0.0f,1.57f + 2.13f,0.1f},
														{ 15.0f + LIFT_MIGRATION,18.0f,0.0f,0.0f,0.0f,2.13f + 0.26f,0.1f},};	
		uint16_t flag_and_time_6[10][2] = {{0,500},{1,500},{1,3000},{1,500},{1,500},{1,300},{1,150},{0,400},{0,100},{0,100}};
			
		memcpy(arm_control.arm_move_routine.Au1,Au1_temp,280);	  
		memcpy(arm_control.arm_move_routine.flag_and_time[5], flag_and_time_6,80);
		
		fp32 Au2_temp[10][7] = {{15.9f + LIFT_MIGRATION, 30.0f, 0.0f,  0.0f, 0.0f, 2.13f, 0.0f},               //伸出
														{15.9f + LIFT_MIGRATION, 30.0f, 0.0f,  0.0f, 0.0f, 2.13f - 0.10f, 0.0f},       //开吸盘 加补偿
														{14.2f + LIFT_MIGRATION, 30.0f, 0.0f,  0.0f, 0.0f, 2.13f - 0.10f, 0.0f},       //抬起 ，动底盘抽出矿石
														{	4.5f + LIFT_MIGRATION, 30.0f, 0.0f,  0.0f, 0.0f, 2.13f - 0.20f, 0.0f},       //举矿
														{ 4.5f + LIFT_MIGRATION, 30.0f, 1.7f, -1.53f,0.0f,1.67f + 2.13f, 0.0f},//伸出后调整矿石姿态，准备放矿
														{ 8.5f + LIFT_MIGRATION, 30.0f, 1.7f, -1.53f,0.0f,1.67f + 2.13f, -0.1f},//移回横移
														{ 8.5f + LIFT_MIGRATION, 16.7f,1.83f,-1.75f,0.07f,1.67f + 2.13f, -0.1f},//放矿
														{ 8.5f + LIFT_MIGRATION, 16.7f,1.83f,-1.75f,0.07f,1.67f + 2.13f, -0.1f},//复位  抬起
														{ 4.5f + LIFT_MIGRATION, 16.7f,0.0f,0.0f,0.0f,2.13f + 0.26f,-0.1f},							//复位  关节复位						
														{ 15.0f + LIFT_MIGRATION, 18.0f,0.0f,0.0f,0.0f,2.13f + 0.26f,0.0f},};          //复位  放下
		
		uint16_t flag_and_time_7[10][2] = {{0,1000},{1,100},{1,3000},{1,100},{1,500},{1,200},{1,200},{0,500},{0,200},{0,1000}};
		memcpy(arm_control.arm_move_routine.Au2,Au2_temp,280);	
		memcpy(arm_control.arm_move_routine.flag_and_time[6], flag_and_time_7,80);
		
		fp32 Au3_temp[3][7] = {{15.9f + LIFT_MIGRATION, 30.0f, 0.0f,  0.0f, 0.0f, 2.13f, 0.0f},               //伸出
														{15.9f + LIFT_MIGRATION, 30.0f, 0.0f,  0.0f, 0.0f, 2.13f - 0.00f, 0.0f},       //开吸盘 加补偿
														{13.6f + LIFT_MIGRATION, 30.0f, 0.0f,  0.0f, 0.0f, 2.13f - 0.00f, 0.0f},      
													};
		
		uint16_t flag_and_time_8[3][2] = {{0,1000},{1,500},{1,2500}};
		
		memcpy(arm_control.arm_move_routine.Au3,Au3_temp,84);	
		memcpy(arm_control.arm_move_routine.flag_and_time[7], flag_and_time_8,24);
		
		arm_control.arm_move_flag = NORMAL_POSITION;
		arm_control.arm_position_flag = 0;
		arm_control.arm_get_position_flag = 0;
		arm_control.motor_1_position = 0.0f;
		arm_control.motor_2_position = 0.0f;
		arm_control.motor_3_position = 0.0f;
		arm_control.motor_4_position = 0.0f;
		arm_control.motor_5_position = 0.0f;
		arm_control.motor_6_position = 0.0f;
		arm_control.motor_7_position = 0.0f;
		
		//抬升微调
		arm_control.lift_adjustion = 0.0f;
		
		key_init(&arm_control_key_init->capture_key, Z);
		key_init(&arm_control_key_init->exchange_key,X);
		key_init(&arm_control_key_init->suker_key,R);
		key_init(&arm_control_key_init->self_control_rotate_key,CTRL);
		
		key_init(&arm_control_key_init->adjust_key,CTRL);
		key_init(&arm_control_key_init->lift_up_key,W);
		key_init(&arm_control_key_init->lift_down_key,S);
		key_init(&arm_control_key_init->yaw_plus_key,Q);
		key_init(&arm_control_key_init->yaw_minus_key,E);		
		key_init(&arm_control_key_init->lift_down_key,S);
		key_init(&arm_control_key_init->lift_adjust_clear_key,C);
		
		//各关节角度限制
		R_6D_ctrl->Joint[0].angleLimitMax =  PI/2;
		R_6D_ctrl->Joint[0].angleLimitMin = -PI/3;
		
		R_6D_ctrl->Joint[1].angleLimitMax =  2.0f*PI/3.0f;
		R_6D_ctrl->Joint[1].angleLimitMin = -2.0f*PI/3.0f;
		
		R_6D_ctrl->Joint[2].angleLimitMax =  PI;
		R_6D_ctrl->Joint[2].angleLimitMin =  0;
		
		R_6D_ctrl->Joint[3].angleLimitMax =  (PI/2.0f + 0.10f*PI);//100-110度;
		R_6D_ctrl->Joint[3].angleLimitMin = -(PI/2.0f + 0.10f*PI);//-100-110度;
		
		R_6D_ctrl->Joint[4].angleLimitMax =  PI;
		R_6D_ctrl->Joint[4].angleLimitMin = -PI;
		
		for(int i = 0; i < 5; i++)
		{
				R_6D_ctrl->Joint_Final[i] = 0.0f;
		}
//		uint8_t routine_length[9] = {12, 12, 10, 6, 7, 10, 10 ,3};
		uint8_t routine_length[9] = {10, 12, 10, 6, 6, 10, 10 ,3};
		
		for(int i = 0; i < 9; i++)
		{
				arm_control.routine_length[i] = routine_length[i];
		}
		TD_init(&R_6D_ctrl->Pose6D_IK_Z_TD, 50.0f, 2.0f, 0.002f, R_6D_ctrl->Pose6D_IK.Z);
		
		//机械臂复位位置
		fp32 repositon_position[7] = {  10.1f + LIFT_MIGRATION,18.0f,1.57f,-1.57f,-1.57f,2.13f,0.0f};		
		fp32 three_ore_position[7] = {  6.1f + LIFT_MIGRATION,18.0f,1.57f,-1.57f,-1.57f,2.13f,0.0f};	
		fp32 pre_Au_reposition[7] = {	8.1f + LIFT_MIGRATION,18.0f,0.0f,0.0f,0.0f,2.13f,0.0f};
		fp32 Au_reposition[7] = {  15.0f + LIFT_MIGRATION,30.0f,0.0f,0.0f,0.0f,2.13f,0.0f};
//		fp32 Ag_reposition[7] = {  9.1f,18.0f,1.57f,-1.57f,0.0f,1.92f,0.0f};
		fp32 Ag_reposition[7] = {  9.4f + LIFT_MIGRATION,5.10f,0.0f,0.0f,0.0f,2.13f,0.0f};		
		memcpy(arm_control.repostion_position,repositon_position,28);
		memcpy(arm_control.three_ore_position,three_ore_position,28);
		memcpy(arm_control.pre_Au_reposition,pre_Au_reposition,28);
		memcpy(arm_control.Au_reposition,Au_reposition,28);
		memcpy(arm_control.Ag_reposition,Ag_reposition,28);
}

void arm_control_key_check(all_key_t *arm_cnotrol_key_check)
{
		if(chassis.chassis_mode == ONE_KEY_MODE)
		{
				key_itself_press_num(&(arm_cnotrol_key_check->capture_key), 2);
				key_itself_press_num(&(arm_cnotrol_key_check->exchange_key),2);
				key_init(&arm_cnotrol_key_check->self_control_rotate_key,CTRL);
		}		
		else if(chassis.chassis_mode == SELF_CONTROL_MODE)
		{
				key_itself_press_num(&(arm_cnotrol_key_check->self_control_rotate_key),2);		
		}
		key_itself_press_num(&(arm_cnotrol_key_check->suker_key),2);
		//抬升微调
		key_itself_press_num(&(arm_cnotrol_key_check->adjust_key),2);
		key_itself_press_num(&(arm_cnotrol_key_check->lift_adjust_clear_key),2);
		key_itself_press_num(&(arm_cnotrol_key_check->lift_up_key),2);
		key_itself_press_num(&(arm_cnotrol_key_check->lift_down_key),2);
		//末端yaw轴
		key_itself_press_num(&(arm_cnotrol_key_check->yaw_plus_key),2);
		key_itself_press_num(&(arm_cnotrol_key_check->yaw_minus_key),2);
		
}

void arm_feedback_update( arm_control_t *arm_control_position, Robotic_6DOF_control_t *R_6D_ctrl)
{
	 if(Quaterniont_Mode)
	 {
				R_6D_ctrl->Pose6D_IK.X = arm_pose.x;
				R_6D_ctrl->Pose6D_IK.Y = arm_pose.y;
				R_6D_ctrl->Pose6D_IK.Z = arm_pose.z;
				R_6D_ctrl->Pose6D_IK.Q[0] = arm_pose.q[0];
				R_6D_ctrl->Pose6D_IK.Q[1] = arm_pose.q[1];
				R_6D_ctrl->Pose6D_IK.Q[2] = arm_pose.q[2];
				R_6D_ctrl->Pose6D_IK.Q[3] = arm_pose.q[3];	 
	 }
	 if(chassis.chassis_mode == SELF_CONTROL_MODE && chassis.last_chassis_mode!=SELF_CONTROL_MODE)
	 {
				yaw_angle_set = arm_message.target_position[5];
	 }
}

void arm_control_set(arm_control_t *arm_control_set, all_key_t *arm_key)
{
		
		if(arm_key->suker_key.itself.mode != arm_key->suker_key.itself.last_mode)
		{
				suker_key_flag = 1 - suker_key_flag;
		}	
		//按C清零微调量
		if(arm_key->lift_adjust_clear_key.itself.mode != arm_key->lift_adjust_clear_key.itself.last_mode)
		{
			arm_control_set->lift_adjustion = 0.0f;
			arm_control_set->arm_move_routine.Ag1[3][0] = Ag_Catch.Ag1 + arm_control_set->lift_adjustion;
			arm_control_set->arm_move_routine.Ag2[3][0] = Ag_Catch.Ag2 + arm_control_set->lift_adjustion;
			arm_control_set->arm_move_routine.Ag3[3][0] = Ag_Catch.Ag3 + arm_control_set->lift_adjustion;
		}
		//按住ctrl 按一次w/s 微调0.1f
		if(arm_key->adjust_key.itself.flag)
		{
			if(arm_key->lift_up_key.itself.mode != arm_key->lift_up_key.itself.last_mode)
			{
				arm_control_set->lift_adjustion -= 0.1f;
			}
			if(arm_key->lift_down_key.itself.mode != arm_key->lift_down_key.itself.last_mode)
			{
				arm_control_set->lift_adjustion += 0.1f;
			}
			arm_control_set->arm_move_routine.Ag1[3][0] = Ag_Catch.Ag1 + arm_control_set->lift_adjustion;
			arm_control_set->arm_move_routine.Ag2[3][0] = Ag_Catch.Ag2 + arm_control_set->lift_adjustion;
			arm_control_set->arm_move_routine.Ag3[3][0] = Ag_Catch.Ag3 + arm_control_set->lift_adjustion;		
		}
		if(chassis.chassis_mode != ONE_KEY_MODE)
		{
				return;
		}
		
		if(picture_rotate_flag == 0)
		{
				arm_control_set->arm_move_flag = NORMAL_POSITION;
		}
		
		arm_check_get_position(arm_control_set);	
		//一键抓矿
		if(arm_key->capture_key.itself.mode != arm_key->capture_key.itself.last_mode)
		{
				arm_control_set->arm_position_flag = 0;
				if(arm_control_set->arm_move_flag == NORMAL_POSITION)
				{
						if(picture_rotate_flag == 1)
						{
								if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1) == 1)
								{
										arm_control_set->arm_move_flag = Au3;
								}
								else if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0) == 1)
								{
										arm_control_set->arm_move_flag = Au2;
								}
								else
								{
										arm_control_set->arm_move_flag = Au1;
								}
						}
						else if(picture_rotate_flag == 2)
						{
								if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1) == 1)
								{
										arm_control_set->arm_move_flag = Ag3;
								}
								else if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0) == 1)
								{
										arm_control_set->arm_move_flag = Ag2;
								}
								else
								{
										arm_control_set->arm_move_flag = Ag1;
								}
						}
						else
						{
								if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1) == 1)
								{
										arm_control_set->arm_move_flag = Ag3;
								}
								else if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0) == 1)
								{
										arm_control_set->arm_move_flag = Ag2;
								}
								else
								{
										arm_control_set->arm_move_flag = Ag1;
								}
						}
				}
				else
				{
						arm_control_set->arm_move_flag = NORMAL_POSITION;
				}
		}
	
		//一键兑矿
		if(arm_key->exchange_key.itself.mode != arm_key->exchange_key.itself.last_mode)
		{
				arm_control_set->arm_position_flag = 0;
				if(arm_control_set->arm_move_flag == NORMAL_POSITION)
				{
						if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1) == 1)
						{
								arm_control_set->arm_move_flag = EXCHANGE1;
						}
						else
						{
								arm_control_set->arm_move_flag = EXCHANGE2;
						}	
				}
				else
				{
						arm_control_set->arm_move_flag = NORMAL_POSITION;
				}
		}
		
		if(arm_control_set->arm_move_flag != NORMAL_POSITION)
		{
				arm_check_get_position(arm_control_set);
				if(arm_control_set->arm_get_position_flag > arm_control_set->arm_move_routine.flag_and_time[arm_control_set->arm_move_flag][arm_control_set->arm_position_flag][1])
				{
						arm_control_set->arm_position_flag ++;
						arm_control_set->arm_get_position_flag = 0;
						if (arm_control_set->arm_move_routine.flag_and_time[arm_control_set->arm_move_flag][arm_control_set->arm_position_flag][0] == 0)
						{
								suker_key_flag = 0;
						}
						else if(arm_control_set->arm_move_routine.flag_and_time[arm_control_set->arm_move_flag][arm_control_set->arm_position_flag][0] == 1)
						{
								suker_key_flag = 1;
						}
				}
				if(arm_control_set->arm_position_flag == arm_control_set->routine_length[arm_control_set->arm_move_flag]-1 
					&& arm_control_set->arm_get_position_flag >= arm_control_set->arm_move_routine.flag_and_time[arm_control_set->arm_move_flag][arm_control_set->arm_position_flag][1])
				{
						//连续一键
						if(arm_control_set->arm_move_flag == Ag1)
						{
								if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0) == 1)
								{
										arm_control_set->arm_move_flag = Ag2;
										arm_control_set->arm_position_flag = 0;
								}
								else
								{
										arm_control_set->arm_move_flag = NORMAL_POSITION;
								}
						}
						else if(arm_control_set->arm_move_flag == Ag2)
						{
								if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1) == 1)
								{
										arm_control_set->arm_move_flag = NORMAL_POSITION;
										arm_control_set->arm_position_flag = 0;
								}
								else
								{
										arm_control_set->arm_move_flag = NORMAL_POSITION;
								}
						}
//						else if(arm_control_set->arm_move_flag == Ag3)
//						{
//								arm_control_set->arm_move_flag = NORMAL_POSITION;
//								arm_control_set->arm_position_flag = 0;
//						}
						else if(arm_control_set->arm_move_flag == EXCHANGE1)
						{
								arm_control_set->arm_position_flag = 0;
								arm_control_set->arm_move_flag = NORMAL_POSITION;
						}
						else if(arm_control_set->arm_move_flag == EXCHANGE2)
						{
								arm_control_set->arm_move_flag = NORMAL_POSITION;
								arm_control_set->arm_position_flag = 0;
						}
						else if(arm_control_set->arm_move_flag == Au1)
						{
								if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0) == 1)
								{
										arm_control_set->arm_move_flag = Au2;
										arm_control_set->arm_position_flag = 0;
								}
								else
								{
										arm_control_set->arm_move_flag = NORMAL_POSITION;
								}
						}
						else if(arm_control_set->arm_move_flag == Au2)
						{
								if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1) == 1)
								{
										arm_control_set->arm_move_flag = Au3;
										arm_control_set->arm_position_flag = 0;
								}
								else
								{
										arm_control_set->arm_move_flag = NORMAL_POSITION;
								}
						}
						else if(arm_control_set->arm_move_flag == Au3)
						{
								arm_control_set->arm_move_flag = NORMAL_POSITION;
								arm_control_set->arm_position_flag = 0;
						}		
				}
		}
		else
		{
				arm_control_set->arm_get_position_flag = 0;
		}	
}


float AbsMaxOf6(Joint6D_t _joints)
{
    float max = -1;
    for (uint8_t i = 0; i < 5; i++)
    {
        if (fabs(_joints.theta[i]) > max)
        {
					  max = fabs(_joints.theta[i]);
        }
    }

    return max;
}
//select solution decoupled
Joint6D_t deltaJoints;
bool xyz_valid[4];
bool valid[4];
//float go_delta_min = 100.0f;
void MoveL(Robotic_6DOF_control_t *R_6D_ctrl, arm_control_t *arm_control_set, all_key_t *arm_key)
{
		bool SolveIK_Success;
		//机械臂逆解算
		SolveIK_Success = SolveIK(&R_6D_ctrl->Robotic_6D, &R_6D_ctrl->Pose6D_IK,
															&R_6D_ctrl->output_solvers_IK, &Last_Joint6D, Quaterniont_Mode);

		if(!SolveIK_Success)
		{
				return;
		}		
		else
		{
				R_6D_ctrl->xyz_ValidCnt = 0;
				R_6D_ctrl->ValidCnt = 0;			
				for (int i = 0; i < 4; i++)
				{
						xyz_valid[i] = true;
						valid[i] = true;
							
						for (int j = 0; j < 2; j++)
						{
								if (R_6D_ctrl->output_solvers_IK.theta[i][j] > R_6D_ctrl->Joint[j].angleLimitMax ||
										R_6D_ctrl->output_solvers_IK.theta[i][j] < R_6D_ctrl->Joint[j].angleLimitMin ||
										R_6D_ctrl->output_solvers_IK.high_position > 17.3f||
										R_6D_ctrl->output_solvers_IK.high_position < -1.0f)
								{
										xyz_valid[i] = false;
								}
								
						}							
						for (int j = 0; j < 5; j++)
						{
								if (R_6D_ctrl->output_solvers_IK.theta[i][j] > R_6D_ctrl->Joint[j].angleLimitMax ||
										R_6D_ctrl->output_solvers_IK.theta[i][j] < R_6D_ctrl->Joint[j].angleLimitMin ||
										R_6D_ctrl->output_solvers_IK.high_position > 17.3f||
										R_6D_ctrl->output_solvers_IK.high_position < -1.0f)
								{
										valid[i] = false;
								}
								
						}	
							if (xyz_valid[i]) R_6D_ctrl->xyz_ValidCnt++;
							if (valid[i]) R_6D_ctrl->ValidCnt++;
				}                                                                                                 

				if (R_6D_ctrl->xyz_ValidCnt && (!R_6D_ctrl->ValidCnt))
				{
						float go_delta_min = 100.0f;
						float go_max_position = 0.0f;
						uint8_t indexConfig = 0;
						for (int i = 0; i < 4; i++)
						{

								if (xyz_valid[i])
								{
										if(fabs(R_6D_ctrl->Pose6D_IK.Y* motor_to_real * sc_to_arm )<200)
										{
												if(fabs(R_6D_ctrl->output_solvers_IK.theta[i][0] - Last_Joint6D.theta[0]) < go_delta_min)
												{
														go_delta_min = fabs(R_6D_ctrl->output_solvers_IK.theta[i][0] - Last_Joint6D.theta[0]);
														indexConfig = i;
												}
										}
										else
										{
												if(R_6D_ctrl->Pose6D_IK.Y < 0)
												{
														if (R_6D_ctrl->output_solvers_IK.theta[i][0] >= go_max_position)
														{
																go_max_position = R_6D_ctrl->output_solvers_IK.theta[i][0]; 
																indexConfig = i;
														}
												}
												else
												{
														if (R_6D_ctrl->output_solvers_IK.theta[i][0] <= go_max_position)
														{
																go_max_position = R_6D_ctrl->output_solvers_IK.theta[i][0]; 
																indexConfig = i;
														}									
												}
										}
								}
						}
						R_6D_ctrl->Joint_Final[0] 	= 	R_6D_ctrl->output_solvers_IK.theta[indexConfig][0];
						R_6D_ctrl->Joint_Final[1]   =   R_6D_ctrl->output_solvers_IK.theta[indexConfig][1];
						R_6D_ctrl->high 	=		-R_6D_ctrl->output_solvers_IK.high_position;			
						for (int j = 0; j < 2; j++)
						{
								Last_Joint6D.theta[j] = R_6D_ctrl->Joint_Final[j];
						}					
				}
				else if(R_6D_ctrl->ValidCnt)
				{		
					  float go_delta_min = 100.0f;
						float go_max_position = 0.0f;
						uint8_t indexConfig = 0;
						for (int i = 0; i < 4; i++)
						{			
								if (valid[i])
								{
										if(fabs(R_6D_ctrl->Pose6D_IK.Y* motor_to_real * sc_to_arm)<200)
										{
											if(fabs(R_6D_ctrl->output_solvers_IK.theta[i][0] - Last_Joint6D.theta[0]) < go_delta_min)
											{
												go_delta_min = fabs(R_6D_ctrl->output_solvers_IK.theta[i][0] - Last_Joint6D.theta[0]);
												indexConfig = i;
											}
										}
										else
										{
												if(R_6D_ctrl->Pose6D_IK.Y < 0)
												{
														if (R_6D_ctrl->output_solvers_IK.theta[i][0] >= go_max_position)
														{
																go_max_position = R_6D_ctrl->output_solvers_IK.theta[i][0]; 
																indexConfig = i;
														}
												}
												else
												{
														if (R_6D_ctrl->output_solvers_IK.theta[i][0] <= go_max_position)
														{
																go_max_position = R_6D_ctrl->output_solvers_IK.theta[i][0]; 
																indexConfig = i;
														}									
												}
										}
								}
						}
						//3508向上为负
						R_6D_ctrl->Joint_Final[0] 	= 	R_6D_ctrl->output_solvers_IK.theta[indexConfig][0];
						R_6D_ctrl->Joint_Final[1]   =   R_6D_ctrl->output_solvers_IK.theta[indexConfig][1];
						R_6D_ctrl->Joint_Final[2]   =   R_6D_ctrl->output_solvers_IK.theta[indexConfig][2];
						R_6D_ctrl->Joint_Final[3]   =   -R_6D_ctrl->output_solvers_IK.theta[indexConfig][3];
						R_6D_ctrl->Joint_Final[4]   =   R_6D_ctrl->output_solvers_IK.theta[indexConfig][4];
						R_6D_ctrl->high 						=		-R_6D_ctrl->output_solvers_IK.high_position;			
						for (int j = 0; j < 5; j++)
						{
								Last_Joint6D.theta[j] = R_6D_ctrl->Joint_Final[j];
						}
			}
			if(R_6D_ctrl->xyz_ValidCnt)
			{						
						//motor2 first
						if(fabs(40.0f - arm_message.target_position[0]) > 0.3f)
						{
								arm_control_set->motor_1_position       =  	 14.9f;
								arm_control_set->motor_2_position       =    40.0f;
								arm_control_set->motor_3_position       =    0.0f;
								arm_control_set->motor_4_position			  =	   0.0f;
								arm_control_set->motor_5_position 	  	=		 -1.57f;
								arm_control_set->motor_6_position 			=	   2.13f;
//								if(arm_key->self_control_rotate_key.itself.mode == 0)
//								{
										arm_control_set->motor_7_position 	=		 0.0f;
//								}
//								else
//								{
//										arm_control_set->motor_7_position   =    PI;								
//								}								
						}
						else
						{
							//按键改末端yaw角度
							if(all_key.adjust_key.itself.flag == 1)
							{
								if(all_key.yaw_plus_key.itself.mode != all_key.yaw_plus_key.itself.last_mode)
									yaw_angle_set += 0.1f;
								if(all_key.yaw_minus_key.itself.mode != all_key.yaw_minus_key.itself.last_mode)
									yaw_angle_set -= 0.1f;
							}
								arm_control_set->motor_1_position       =  	 R_6D_ctrl->high + 14.9f;
								arm_control_set->motor_2_position       =    40.0f;
								arm_control_set->motor_3_position       =    R_6D_ctrl->Joint_Final[0] ;
								arm_control_set->motor_4_position			  =	   R_6D_ctrl->Joint_Final[1] ;
								arm_control_set->motor_5_position 	  	=		 R_6D_ctrl->Joint_Final[2] - 1.57f;
								arm_control_set->motor_6_position 			=	   -R_6D_ctrl->Joint_Final[3] + 2.58f - 0.56f;//2006需要复位导致的零点变化
//								if(arm_key->self_control_rotate_key.itself.mode == 0)
//								{
										arm_control_set->motor_7_position 	=		 /*yaw_angle_set;*/R_6D_ctrl->Joint_Final[4];
//								}
//								else
//								{
//										arm_control_set->motor_7_position   =  	 rad_format(yaw_angle_set + PI);//rad_format(R_6D_ctrl->Joint_Final[4] + PI);								
//								}							
						}
				}
		}
		return;
}

void arm_control_loop(Robotic_6DOF_control_t *R_6D_ctrl,arm_control_t *arm_control_loop, all_key_t *arm_key)
{
	
		if(suker_key_flag == 1)
		{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
		}
		else
		{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
		}	
	
		if(chassis.chassis_mode == ONE_KEY_MODE)
		{
				for(int i = 0; i < 7; i++)
				{
						if(arm_control_loop->arm_move_flag == Ag1)
						{
							arm_control_loop->one_key_position[i] = arm_control_loop->arm_move_routine.Ag1[arm_control_loop->arm_position_flag][i];
						}
						else if(arm_control_loop->arm_move_flag == Ag2)
						{
							arm_control_loop->one_key_position[i] = arm_control_loop->arm_move_routine.Ag2[arm_control_loop->arm_position_flag][i];
						}
						else if(arm_control_loop->arm_move_flag == Ag3)
						{
							arm_control_loop->one_key_position[i] = arm_control_loop->arm_move_routine.Ag3[arm_control_loop->arm_position_flag][i];
						}
						else if(arm_control_loop->arm_move_flag == EXCHANGE1)
						{
							arm_control_loop->one_key_position[i] = arm_control_loop->arm_move_routine.exchange1[arm_control_loop->arm_position_flag][i];
						}
						else if(arm_control_loop->arm_move_flag == EXCHANGE2)
						{
							arm_control_loop->one_key_position[i] = arm_control_loop->arm_move_routine.exchange2[arm_control_loop->arm_position_flag][i];
						}
						else if(arm_control_loop->arm_move_flag == Au1)
						{
							arm_control_loop->one_key_position[i] = arm_control_loop->arm_move_routine.Au1[arm_control_loop->arm_position_flag][i];
						}
						else if(arm_control_loop->arm_move_flag == Au2)
						{
							arm_control_loop->one_key_position[i] = arm_control_loop->arm_move_routine.Au2[arm_control_loop->arm_position_flag][i];
						}
						else if(arm_control_loop->arm_move_flag == Au3)
						{
							arm_control_loop->one_key_position[i] = arm_control_loop->arm_move_routine.Au3[arm_control_loop->arm_position_flag][i];
						}
				}
			
				if(arm_control_loop->arm_move_flag == NORMAL_POSITION)
				{
						if(picture_rotate_flag == 0)
						{
								if(suker_key_flag ==1 && HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1) == 1)
								{
										//复位
										G_flag = 0;
										G_reset_flag = 3000;
										arm_control_loop->one_key_position[0] = arm_control_loop->three_ore_position[0];
										arm_control_loop->one_key_position[1] = arm_control_loop->three_ore_position[1];
										arm_control_loop->one_key_position[2] = arm_control_loop->three_ore_position[2];
										arm_control_loop->one_key_position[3] = arm_control_loop->three_ore_position[3];
										arm_control_loop->one_key_position[4] = arm_control_loop->three_ore_position[4];
										arm_control_loop->one_key_position[5] = arm_control_loop->three_ore_position[5];
										arm_control_loop->one_key_position[6] = arm_control_loop->three_ore_position[6];											
								}
								else
								{
										//复位
										G_flag = 0;
										G_reset_flag = 3000;
										arm_control_loop->one_key_position[0] = arm_control_loop->repostion_position[0];
										arm_control_loop->one_key_position[1] = arm_control_loop->repostion_position[1];
										arm_control_loop->one_key_position[2] = arm_control_loop->repostion_position[2];
										arm_control_loop->one_key_position[3] = arm_control_loop->repostion_position[3];
										arm_control_loop->one_key_position[4] = arm_control_loop->repostion_position[4];
										arm_control_loop->one_key_position[5] = arm_control_loop->repostion_position[5];
										arm_control_loop->one_key_position[6] = arm_control_loop->repostion_position[6];										
								}
			
						}
						else if(picture_rotate_flag == 1)
						{
								if(G_flag <= 500 || G_reset_flag <= 2500)
								{
										G_flag++;
										G_reset_flag ++;
										arm_control_loop->one_key_position[0] = arm_control_loop->pre_Au_reposition[0];
										arm_control_loop->one_key_position[1] = arm_control_loop->pre_Au_reposition[1];
										arm_control_loop->one_key_position[2] = arm_control_loop->pre_Au_reposition[2];
										arm_control_loop->one_key_position[3] = arm_control_loop->pre_Au_reposition[3];
										arm_control_loop->one_key_position[4] = arm_control_loop->pre_Au_reposition[4];
										arm_control_loop->one_key_position[5] = arm_control_loop->pre_Au_reposition[5];
										arm_control_loop->one_key_position[6] = arm_control_loop->pre_Au_reposition[6];
								}
								else
								{
										arm_control_loop->one_key_position[0] = arm_control_loop->Au_reposition[0];
										arm_control_loop->one_key_position[1] = arm_control_loop->Au_reposition[1];
										arm_control_loop->one_key_position[2] = arm_control_loop->Au_reposition[2];
										arm_control_loop->one_key_position[3] = arm_control_loop->Au_reposition[3];
										arm_control_loop->one_key_position[4] = arm_control_loop->Au_reposition[4];
										arm_control_loop->one_key_position[5] = arm_control_loop->Au_reposition[5];
										arm_control_loop->one_key_position[6] = arm_control_loop->Au_reposition[6];
								}
						}
						else if(picture_rotate_flag == 2)
						{
								arm_control_loop->one_key_position[0] = arm_control_loop->Ag_reposition[0];
								arm_control_loop->one_key_position[1] = arm_control_loop->Ag_reposition[1];
								arm_control_loop->one_key_position[2] = arm_control_loop->Ag_reposition[2];
								arm_control_loop->one_key_position[3] = arm_control_loop->Ag_reposition[3];
								arm_control_loop->one_key_position[4] = arm_control_loop->Ag_reposition[4];
								arm_control_loop->one_key_position[5] = arm_control_loop->Ag_reposition[5];
								arm_control_loop->one_key_position[6] = arm_control_loop->Ag_reposition[6];
						}
				}
				arm_control_loop->motor_1_position = arm_control_loop->one_key_position[0];
				arm_control_loop->motor_2_position = arm_control_loop->one_key_position[1];
				arm_control_loop->motor_3_position = arm_control_loop->one_key_position[2];
				arm_control_loop->motor_4_position = arm_control_loop->one_key_position[3];
				arm_control_loop->motor_5_position = arm_control_loop->one_key_position[4];
				arm_control_loop->motor_6_position = arm_control_loop->one_key_position[5];
				arm_control_loop->motor_7_position = arm_control_loop->one_key_position[6];
		}
		else if(chassis.chassis_mode == SELF_CONTROL_MODE)
		{
				if(chassis.last_chassis_mode != SELF_CONTROL_MODE)
				{
						R_6D_ctrl->Pose6D_IK_Z_TD.x = 0.0f;
				}
				TD_calc(&R_6D_ctrl->Pose6D_IK_Z_TD,R_6D_ctrl->Pose6D_IK.Z);
				R_6D_ctrl->Pose6D_IK.Z = R_6D_ctrl->Pose6D_IK_Z_TD.x;
				MoveL(R_6D_ctrl,arm_control_loop,arm_key);
		}
}

void arm_check_get_position(arm_control_t *check_position)
{
		if(check_position->motor_1_position - chassis.motor_lift.position < 0.2f &&
			 check_position->motor_1_position - chassis.motor_lift.position > -0.2f &&
		   check_position->motor_2_position - (arm_message.target_position[0]) < 0.2f &&
			 check_position->motor_2_position - (arm_message.target_position[0]) > -0.2f &&
		   check_position->motor_3_position - (arm_message.target_position[1]) < 0.1f &&
			 check_position->motor_3_position - (arm_message.target_position[1]) > -0.1f &&
		   check_position->motor_4_position - (arm_message.target_position[2]) < 0.1f &&
			 check_position->motor_4_position - (arm_message.target_position[2]) > -0.1f &&
		   check_position->motor_5_position - (arm_message.target_position[3]) < 0.2f &&
			 check_position->motor_5_position - (arm_message.target_position[3]) > -0.2f &&
		   check_position->motor_6_position - (arm_message.target_position[4]) < 0.15f &&
			 check_position->motor_6_position - (arm_message.target_position[4]) > -0.15f &&
		   check_position->motor_7_position - (arm_message.target_position[5]) < 0.15f &&
			 check_position->motor_7_position - (arm_message.target_position[5]) > -0.15f )
		{
				check_position->arm_get_position_flag++;
		}
		else
		{
				check_position->arm_get_position_flag = 0;
		}
}

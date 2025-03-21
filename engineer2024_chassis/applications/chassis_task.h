/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_t.c/h
  * @brief      chassis_t control task,
  *             底盘控制任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *  V1.1.0     Nov-11-2019     RM              1. add chassis_t power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "controller.h"
#include "user_lib.h"
#include "keyboard.h"
#include "referee.h"
#include "TD.h"

//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 1
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 0
//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 2
//选择底盘状态 开关通道号
#define RC_S_RIGHT 0
#define RC_S_LEFT 1
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN -0.005f
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN -0.004f
//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN    -0.000009f
#define CHASSIS_WZ_MOUSE_SEN -0.00008f
//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.008f
//底盘速度设定值滤波器
#define CHASSIS_ACCEL_X_NUM 0.18f
#define CHASSIS_ACCEL_Y_NUM 0.18f
//摇杆死区
#define CHASSIS_RC_DEADLINE 10
#define MOUSE_DEADBAND 	0
//m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
//单个底盘电机最大速度
#define MAX_WHEEL_SPEED 					 		3.5f
#define SHIFT_NORMAL_MAX_CHASSIS_SPEED_X 		2.5f
#define SHIFT_NORMAL_MAX_CHASSIS_SPEED_Y 		1.8f
#define NORMAL_MAX_CHASSIS_SPEED_X 		1.5f
#define NORMAL_MAX_CHASSIS_SPEED_Y 		1.3f
#define SHIFT_SLOW_CHASSIS_SPEED_X 		1.0f
#define SHIFT_SLOW_CHASSIS_SPEED_Y 		1.0f
#define SLOW_CHASSIS_SPEED_X 					0.8f
#define SLOW_CHASSIS_SPEED_Y 			 		0.3f
#define SELF_CONTROL_SPEED_X 					0.7f
#define SELF_CONTROL_SPEED_Y 			 		0.7f
//麦轮解算参数
#define CHASSIS_WZ_SET_SCALE 		   0.0f
#define MOTOR_DISTANCE_TO_CENTER   0.307f
//麦轮反解算参数
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f
//底盘电机速度环PID
#define CHASSIS_M3508_SPEED_PID_KP 				32000.0f
#define CHASSIS_M3508_SPEED_PID_KI        1000.0f
#define CHASSIS_M3508_SPEED_PID_KD        1.0f
#define CHASSIS_M3508_SPEED_PID_MAX_OUT   16383.0f
#define CHASSIS_M3508_SPEED_PID_MAX_IOUT  3000.0f
//底盘角速度环PID
#define CHASSIS_YAW_PID_KP       16.5f
#define CHASSIS_YAW_PID_KI       0.5f
#define CHASSIS_YAW_PID_KD 			 0.28f
#define CHASSIS_YAW_PID_MAX_OUT  5.8f
#define CHASSIA_YAW_PID_MAX_IOUT 0.3f
//夹矿电机
#define CLAMP_POSITION_PID_KP      30.0f
#define CLAMP_POSITION_PID_KI      0.0f
#define CLAMP_POSITION_PID_KD      0.05f
#define CLAMP_POSITION_PID_MAX_OUT 20.0f
#define CLAMP_POSITION_PID_MAX_KD  0.0f
#define CLAMP_SPEED_PID_KP       	 1500.0f
#define CLAMP_SPEED_PID_KI         0.0f
#define CLAMP_SPEED_PID_KD         0.0f
#define CLAMP_SPEED_PID_MAX_OUT    10000.0f
#define CLAMP_SPEED_PID_MAX_KD     0.0f
//图传抬升
#define PICTURE_POSITION_PID_KP       5.0f
#define PICTURE_POSITION_PID_KI       0.0f
#define PICTURE_POSITION_PID_KD       0.1f
#define PICTURE_POSITION_PID_MAX_OUT  25.0f
#define PICTURE_POSITION_PID_MAX_IOUT 5.0f
#define PICTURE_SPEED_PID_KP 					700.0f
#define PICTURE_SPEED_PID_KI 					0.0f
#define PICTURE_SPEED_PID_KD 					0.03f
#define PICTURE_SPEED_PID_MAX_OUT  		10000.0f
#define PICTURE_SPEED_PID_MAX_IOUT 		0.0f
//机械臂抬升
#define LIFT_POSITION_PID_KP           200.0f
#define LIFT_POSITION_PID_KI           5.0f
#define LIFT_POSITION_PID_KD           0.01f
#define LIFT_POSITION_PID_MAX_OUT      28.0f
#define LIFT_POSITION_PID_MAX_IOUT     5.0f
#define LIFT_SPEED_PID_KP              900.0f
#define LIFT_SPEED_PID_KI              0.0f
#define LIFT_SPEED_PID_KD              0.01f
#define LIFT_SPEED_SPEED_PID_MAX_OUT   16383.0f
#define LIFT_SPEED_SPEED_PID_MAX_IOUT  3000.0f
//编码器参考值
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8192
//减速比
#define REDUCTION_RATIO_3508				187.0f/3591.0f
#define REDUCTION_RATIO_2006				1.0f/36.0f
//电机单位转化参数
#define MOTOR_ECD_TO_ANGLE_2006     0.000021305288720633f
#define MOTOR_ECD_TO_ANGLE_3508     0.00003994074176199f
#define RPM_TO_RAD_S							  0.104719755119659774f


typedef enum
{
	ZERO_POWER_MODE,
	ONE_KEY_MODE,
	SELF_CONTROL_MODE,
}mode_e;

typedef struct
{
  const motor_measure_t *motor_measure;
  fp32 		speed;
  fp32 		speed_set;
  int16_t give_current;
} chassis_motor_t;

typedef struct
{
	const motor_measure_t *motor_measure;
	fp32 		speed;
	fp32 		speed_set;	
	fp32 		position;
	fp32 		position_set;
	fp32 		offset_ecd;
	int32_t round_cnt;
	int16_t give_current;
}position_motor_t;


typedef struct
{
  const RC_ctrl_t *chassis_RC;                //底盘使用的遥控器指针
  chassis_motor_t motor_chassis[4];           //底盘3508电机数据
	position_motor_t motor_clamp;								//夹矿2006电机数据
	position_motor_t motor_picture;			  			//图传2006电机数据
	position_motor_t motor_lift;			      		//抬升3508电机数据
		      
  PID_t chassis_motor_speed_pid[4];             
	PID_t clamp_motor_position_pid;
	PID_t clamp_motor_speed_pid;
	PID_t picture_motor_position_pid;
	PID_t picture_motor_speed_pid;
	PID_t lift_motor_position_pid;
	PID_t lift_motor_speed_pid;
	PID_t chassis_yaw_pid; 
 	
	TD_t lift_3508_TD;
	TD_t chassis_yaw_TD;
	
  first_order_filter_type_t chassis_cmd_slow_set_vx;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值
  first_order_filter_type_t chassis_cmd_slow_set_vy;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值
	
  fp32 vx;                          //chassis_t vertical speed, positive means forward,unit m/s. 底盘速度 前进方向 前为正，单位 m/s
  fp32 vy;                          //chassis_t horizontal speed, positive means letf,unit m/s.底盘速度 左右方向 左为正  单位 m/s
  fp32 wz;                          //chassis_t rotation speed, positive means counterclockwise,unit rad/s.底盘旋转角速度，逆时针为正 单位 rad/s
  fp32 vx_set;                      //chassis_t set vertical speed,positive means forward,unit m/s.底盘设定速度 前进方向 前为正，单位 m/s
  fp32 vy_set;                      //chassis_t set horizontal speed,positive means left,unit m/s.底盘设定速度 左右方向 左为正，单位 m/s
  fp32 wz_set;                      //chassis_t set rotation speed,positive means counterclockwise,unit rad/s.底盘设定旋转角速度，逆时针为正 单位 rad/s
	
	fp32 yaw_angle;
	fp32 yaw_angle_set;
	
	mode_e  chassis_mode;//模式
	mode_e  last_chassis_mode;	
	
	float dt;
	uint32_t  DWT_Count;	
} chassis_t;


extern void chassis_task(void const *pvParameters);
extern chassis_t chassis;
extern uint8_t picture_rotate_flag;
extern uint32_t G_reset_flag;
#endif

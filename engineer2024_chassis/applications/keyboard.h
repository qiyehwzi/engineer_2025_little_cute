#ifndef KEY_H
#define KEY_H
#include "struct_typedef.h"

typedef enum
{
		W,
		S,
		A,
		D,
		SHIFT,
		CTRL,
		Q,
		E,
		R,
		F,
		G,
		Z,
		X,
		C,
		V,
		B,
		PR,
		PL,

}key_e;

typedef struct
{
	int8_t mode;//��һ�μ�1
	int8_t time;
	int8_t flag;//����/�ɿ�״ָ̬ʾ
	int8_t last_mode;
}key_mode_t;

typedef struct
{
	key_e content;
	key_mode_t itself;
}key_t;

typedef struct
{
	key_t suker_key;	
	key_t capture_key;
	key_t exchange_key;
	key_t self_control_rotate_key;
	key_t rotate_key_G;
	key_t rotate_key_V;
	key_t picture_rotate_key;
	key_t arm_restart_key1;
	key_t arm_restart_key2;	
	key_t adjust_key;
	key_t lift_up_key;
	key_t lift_down_key;
	key_t lift_adjust_clear_key;
	key_t yaw_plus_key;
	key_t yaw_minus_key;
}all_key_t;

extern all_key_t all_key;

extern void key_init(key_t *key_init, key_e KEY);
extern void key_itself_press_num(key_t *press_num, int8_t num);  
extern int8_t key_press(key_t *press);     
#endif

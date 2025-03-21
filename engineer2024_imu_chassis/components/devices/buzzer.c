#include "buzzer.h"
#include "tim.h"
extern TIM_HandleTypeDef htim4;

void buzzer_int(void) {
    HAL_TIM_Base_Start(&htim4);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}
//ARR��װ��ֵΪ65535+1
void buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim4, psc);//�޸�Ԥ��Ƶ
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);//�޸�CCR���Ƚ�ֵ��
}
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}


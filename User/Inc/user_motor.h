/*
 * hal_motor.h
 *
 *  Created on: 2023年7月31日
 *      Author: Yeqing.Xiao
 */

#ifndef INC_USER_MOTOR_H_
#define INC_USER_MOTOR_H_

#include "main.h"

#define Motor1_TIM htim3
#if 1
/*新板*/
#define Motor1_IN1 TIM_CHANNEL_2
#define Motor1_IN2 TIM_CHANNEL_3
#else
/*旧板*/
#define Motor1_IN1 TIM_CHANNEL_4
#define Motor1_IN2 TIM_CHANNEL_3

#endif

#define Motor2_TIM htim1
#if 1
/*新板*/
#define Motor2_IN1 TIM_CHANNEL_2
#define Motor2_IN2 TIM_CHANNEL_1

#else
/*旧板*/
#define Motor2_IN1 TIM_CHANNEL_1
#define Motor2_IN2 TIM_CHANNEL_2

#endif

/*
电机驱动芯片工作模式
IN1      	IN2      	Mode
H (PWM)     L           Forward
L (PWM)     L           Fast Decay(快速放电/滑行)
L           H (PWM)     Reverse
L           L (PWM)     Fast Decay
H           L (PWM)     Forward
H           H (PWM)     Slow Decay(慢速放电/刹车)
L(PWM)      H           Reverse
H(PWM)      H           Slow Decay
*/

typedef struct{
	uint16_t adj_current[2];
	uint16_t adj_ramp_up_time;
	uint8_t adj_motor_speed;
	uint8_t adj_select_motor;
	uint8_t adj_direction_cw;
	uint8_t adj_direction_ccw;
	uint8_t adj_fast_braking;
	uint8_t adj_error[2];
	uint8_t adj_fast_braking_flag;
}adj_motor_control_t;

typedef struct{
	TIM_HandleTypeDef* htim;
	uint16_t Motor_IN1;
	uint16_t Motor_IN2;
}adj_motor_channel_t;

extern void adj_motor_handler(void);
extern void user_adj_motor_init(void);

#endif /* INC_USER_MOTOR_H_ */

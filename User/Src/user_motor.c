/*
 * hal_motor.c
 *
 *  Created on: 2023年7月31日
 *      Author: Yeqing.Xiao
 */

#include "tim.h"
#include "user_motor.h"
#include "user_can.h"
#include "database.h"

#define MOTOR_PWM_PERIOD (1000UL)

const adj_motor_channel_t adj_motor_channel[2] = {
	{
		&Motor1_TIM,
		Motor1_IN1,
		Motor1_IN2,
	},
	{
		&Motor2_TIM,
		Motor2_IN1,
		Motor2_IN2,
	}};

extern uint32_t AllPtrToData[IO_MaxNumber + IO_SubjoinNumber];
extern uint16_t node_guarding_count;
volatile adj_motor_control_t adj_motor_control = {0};

void user_adj_motor_init(void)
{
	//	uint8_t len = sizeof(adj_motor_control);
	//	uint8_t* pointer = (uint8_t*)&adj_motor_control;
	//	for(uint8_t i = 0; i < len; i++)
	//	{
	//		pointer[i] = 0;
	//	}
	__HAL_TIM_SET_COMPARE(&Motor1_TIM, Motor1_IN1, 0); // 占空比设置为0
	__HAL_TIM_SET_COMPARE(&Motor1_TIM, Motor1_IN2, 0); // 占空比设置为0
	HAL_TIM_PWM_Start(&Motor1_TIM, Motor1_IN1);
	HAL_TIM_PWM_Start(&Motor1_TIM, Motor1_IN2);

	__HAL_TIM_SET_COMPARE(&Motor2_TIM, Motor2_IN1, 0); // 占空比设置为0
	__HAL_TIM_SET_COMPARE(&Motor2_TIM, Motor2_IN2, 0); // 占空比设置为0
	HAL_TIMEx_PWMN_Start(&Motor2_TIM, Motor2_IN1);
	HAL_TIMEx_PWMN_Start(&Motor2_TIM, Motor2_IN2);
}

/*
 * 正转
 */
void adj_motor_cw(const adj_motor_channel_t *motor, uint16_t speed)
{
	speed = MOTOR_PWM_PERIOD * speed / 100;
	__HAL_TIM_SET_COMPARE(motor->htim, motor->Motor_IN1, speed);
	__HAL_TIM_SET_COMPARE(motor->htim, motor->Motor_IN2, 0); // 占空比设置为0
	HAL_TIM_PWM_Start(motor->htim, motor->Motor_IN1);
	HAL_TIM_PWM_Start(motor->htim, motor->Motor_IN2);
}

/*
 * 反转
 */
void adj_motor_ccw(const adj_motor_channel_t *motor, uint16_t speed)
{
	speed = MOTOR_PWM_PERIOD * speed / 100;
	__HAL_TIM_SET_COMPARE(motor->htim, motor->Motor_IN1, 0); // 占空比设置为0
	__HAL_TIM_SET_COMPARE(motor->htim, motor->Motor_IN2, speed);
	HAL_TIM_PWM_Start(motor->htim, motor->Motor_IN1);
	HAL_TIM_PWM_Start(motor->htim, motor->Motor_IN2);
}

/*
 * 停止
 */
void adj_motor_stop(const adj_motor_channel_t *motor, uint16_t speed)
{
	if (__HAL_TIM_GET_COMPARE(motor->htim, motor->Motor_IN1) == 0)
	{
		__HAL_TIM_SET_COMPARE(motor->htim, motor->Motor_IN1, 0);	 // 占空比设置为0
		__HAL_TIM_SET_COMPARE(motor->htim, motor->Motor_IN2, speed); // 占空比设置为0
	}
	else
	{
		__HAL_TIM_SET_COMPARE(motor->htim, motor->Motor_IN1, speed); // 占空比设置为0
		__HAL_TIM_SET_COMPARE(motor->htim, motor->Motor_IN2, 0);	 // 占空比设置为0
	}
	HAL_TIM_PWM_Start(motor->htim, motor->Motor_IN1);
	HAL_TIM_PWM_Start(motor->htim, motor->Motor_IN2);
}

void adj_motor_handler(void)
{
	static uint8_t motor1_speed[2] = {0, 0};
	if (adj_motor_control.adj_select_motor > 1)
	{
		//		adj_motor_stop(&adj_motor_channel[0], 0);
		//		adj_motor_stop(&adj_motor_channel[1], 0);
		return;
	}
	if (adj_motor_control.adj_fast_braking_flag == 1)
	{
		adj_motor_control.adj_fast_braking_flag = 0;
		motor1_speed[0] = 0;
		motor1_speed[1] = 0;
		adj_motor_stop(&adj_motor_channel[0], motor1_speed[0]);
		adj_motor_stop(&adj_motor_channel[1], motor1_speed[1]);
	}

	if (node_guarding_count > node_guarding_count_2s)
	{
		adj_motor_control.adj_direction_cw = 0;
		adj_motor_control.adj_direction_ccw = 0;
		motor1_speed[0] = 0;
		motor1_speed[1] = 0;
	}
	else
	{
		if (AllPtrToData[IO_doutDcMotorCW] == 0)
		{
			adj_motor_control.adj_direction_cw = 0;
		}
		else
		{
			adj_motor_control.adj_direction_cw = 1;
		}

		if (AllPtrToData[IO_doutDcMotorCCW] == 0)
		{
			adj_motor_control.adj_direction_ccw = 0;
		}
		else
		{
			adj_motor_control.adj_direction_ccw = 1;
		}
	}

	if (adj_motor_control.adj_direction_cw) // 正转
	{
		updata_data(IO_dinDcMotorIsMoving, 1);
		if (adj_motor_control.adj_fast_braking)
		{
			motor1_speed[adj_motor_control.adj_select_motor] = adj_motor_control.adj_motor_speed;
		}
		if (motor1_speed[adj_motor_control.adj_select_motor] < adj_motor_control.adj_motor_speed)
		{
			motor1_speed[adj_motor_control.adj_select_motor]++;
		}
		adj_motor_cw(&adj_motor_channel[adj_motor_control.adj_select_motor], motor1_speed[adj_motor_control.adj_select_motor]);
	}
	else if (adj_motor_control.adj_direction_ccw) // 反转
	{
		updata_data(IO_dinDcMotorIsMoving, 1);
		if (adj_motor_control.adj_fast_braking)
		{
			motor1_speed[adj_motor_control.adj_select_motor] = adj_motor_control.adj_motor_speed;
		}
		if (motor1_speed[adj_motor_control.adj_select_motor] < adj_motor_control.adj_motor_speed)
		{
			motor1_speed[adj_motor_control.adj_select_motor]++;
		}
		adj_motor_ccw(&adj_motor_channel[adj_motor_control.adj_select_motor], motor1_speed[adj_motor_control.adj_select_motor]);
	}
	else
	{
		updata_data(IO_dinDcMotorIsMoving, 0);
		// motor1_speed[0] = 0;
		// motor1_speed[1] = 0;
		if (motor1_speed[0])
		{
			motor1_speed[0]--;
		}
		if (motor1_speed[1])
		{
			motor1_speed[1]--;
		}
		adj_motor_stop(&adj_motor_channel[0], motor1_speed[0]);
		adj_motor_stop(&adj_motor_channel[1], motor1_speed[1]);
	}
}

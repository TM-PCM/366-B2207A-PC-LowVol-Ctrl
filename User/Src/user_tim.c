/*
 * user_tim.c
 *
 *  Created on: Aug 2, 2023
 *      Author: Yeqing.Xiao
 */

#include "tim.h"
#include "user_tim.h"

#define inport_htime htim2

#define inport1_htime htim2
#define inport1_chaneel TIM_CHANNEL_2

#define inport2_htime htim2
#define inport2_chaneel TIM_CHANNEL_3

extern TIM_HandleTypeDef htim2;

tim_capture_t distance_inport1_pwm = {0};//channel 4
tim_capture_t distance_inport2_pwm = {0};//channel 3

/*
 * 开启定时器中断
 */
void user_tim_init(void)
{
	HAL_TIM_Base_Start_IT(&inport_htime);				// 开启定时器2溢出中断
	HAL_TIM_IC_Start_IT(&inport1_htime, inport1_chaneel); // 开启输入捕获中断
	HAL_TIM_IC_Start_IT(&inport2_htime, inport2_chaneel); // 开启输入捕获中断
}

void user_distance_IRQHandler(void)
{
	if (inport_htime.Instance->SR & 0x00000001) // TIM_FLAG_UPDATE 定时器溢出中断
	{
		distance_inport2_pwm.round++;
		distance_inport1_pwm.round++;
	}
	if (inport2_htime.Instance->SR & (1 << 3)) // TIM_FLAG_CC3 定时器输入捕获3
	{
		inport2_htime.Instance->CCER ^= 0x200; // 改变触发方式
		if (distance_inport2_pwm.flag == 0)
		{
			distance_inport2_pwm.lastCaptureVal = inport2_htime.Instance->CCR3;
			distance_inport2_pwm.flag = 1;
		}
		else
		{
			distance_inport2_pwm.curCaptureVal = inport2_htime.Instance->CCR3;
			if (inport2_htime.Instance->CCER & 0x200) // 现在是下降沿触发，所以改变触发方式前为上升沿，差值则为低电平时间
			{
				distance_inport2_pwm.tCaptureLow = distance_inport2_pwm.allCurCapture - distance_inport2_pwm.lastCaptureVal;
				distance_inport2_pwm.lastCaptureVal = distance_inport2_pwm.curCaptureVal;
				distance_inport2_pwm.round = 0;
			}
			else
			{
				distance_inport2_pwm.tCaptureHigh = distance_inport2_pwm.allCurCapture - distance_inport2_pwm.lastCaptureVal;
				distance_inport2_pwm.lastCaptureVal = distance_inport2_pwm.curCaptureVal;
				distance_inport2_pwm.round = 0;
			}
		}
	}

	if (inport1_htime.Instance->SR & (1 << 2)) // TIM_FLAG_CC4 定时器输入捕获2
	{
		inport1_htime.Instance->CCER ^= 0x20; // 改变触发方式
		if (distance_inport1_pwm.flag == 0)
		{
			distance_inport1_pwm.lastCaptureVal = inport1_htime.Instance->CCR2;
			distance_inport1_pwm.flag = 1;
		}
		else
		{
			distance_inport1_pwm.curCaptureVal = inport1_htime.Instance->CCR2;
			if (inport1_htime.Instance->CCER & 0x20) // 现在是下降沿触发，所以改变触发方式前为上升沿，差值则为低电平时间
			{
				distance_inport1_pwm.tCaptureLow = distance_inport1_pwm.allCurCapture - distance_inport1_pwm.lastCaptureVal;
				distance_inport1_pwm.lastCaptureVal = distance_inport1_pwm.curCaptureVal;
				distance_inport1_pwm.round = 0;
			}
			else
			{
				distance_inport1_pwm.tCaptureHigh = distance_inport1_pwm.allCurCapture - distance_inport1_pwm.lastCaptureVal;
				distance_inport1_pwm.lastCaptureVal = distance_inport1_pwm.curCaptureVal;
				distance_inport1_pwm.round = 0;
			}
		}
	}
	inport_htime.Instance->SR = 0; // 清除所有中断标记
}

#if 0 // 官方回调函数， 处理的东西太多，导致100KHZ的波形，捕获时间就不准确，故不采用官方回调函数
/*
 * 捕获中断回调函数
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(TIM4 == htim->Instance)
	{
		if(HAL_TIM_ACTIVE_CHANNEL_3 == htim->Channel)
		{
			htim->Instance->CCER ^= 0x200;//改变触发方式
			if(distance_inport2_pwm.flag == 0)
			{
				distance_inport2_pwm.lastCaptureVal = htim->Instance->CCR3;
				distance_inport2_pwm.flag = 1;
			}
			else
			{
				distance_inport2_pwm.curCaptureVal = htim->Instance->CCR3;
				if(htim->Instance->CCER & 0x200)// TIM_CCER_CC3P 上升沿
				{
					distance_inport2_pwm.tCaptureLow = distance_inport2_pwm.allCurCapture - distance_inport2_pwm.lastCaptureVal;
					distance_inport2_pwm.lastCaptureVal = distance_inport2_pwm.curCaptureVal;
					distance_inport2_pwm.round = 0;
				}
				else//下降沿
				{
					distance_inport2_pwm.tCaptureHigh= distance_inport2_pwm.allCurCapture - distance_inport2_pwm.lastCaptureVal;
					distance_inport2_pwm.lastCaptureVal = distance_inport2_pwm.curCaptureVal;
					distance_inport2_pwm.round = 0;
				}
			}
		}
		if(HAL_TIM_ACTIVE_CHANNEL_4 == htim->Channel)
		{
			htim->Instance->CCER ^= 0x2000;//改变触发方式
			if(distance_inport1_pwm.flag == 0)
			{
				distance_inport1_pwm.lastCaptureVal = htim->Instance->CCR4;
				distance_inport1_pwm.flag = 1;
			}
			else
			{
				distance_inport1_pwm.curCaptureVal = htim->Instance->CCR4;
				if(htim->Instance->CCER & 0x2000)// TIM_CCER_CC4P 上升沿
				{
					distance_inport1_pwm.tCaptureLow = distance_inport1_pwm.allCurCapture - distance_inport1_pwm.lastCaptureVal;
					distance_inport1_pwm.lastCaptureVal = distance_inport1_pwm.curCaptureVal;
					distance_inport1_pwm.round = 0;
				}
				else//下降沿
				{
					distance_inport1_pwm.tCaptureHigh= distance_inport1_pwm.allCurCapture - distance_inport1_pwm.lastCaptureVal;
					distance_inport1_pwm.lastCaptureVal = distance_inport1_pwm.curCaptureVal;
					distance_inport1_pwm.round = 0;
				}
			}
		}
	}
}
#endif

#if 0 // main.c中已有定义
//定时器溢出回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(TIM4 == htim->Instance)
    {
		distance_inport2_pwm.round++;
		distance_inport1_pwm.round++;
    }
}
#endif

/*
 * user_adc.c
 *
 *  Created on: Aug 3, 2023
 *      Author: Yeqing.Xiao
 */

#include "adc.h"
#include "user_adc.h"
#include "cmsis_os.h"
#include "user_motor.h"

extern volatile adj_motor_control_t adj_motor_control;
extern DMA_HandleTypeDef hdma_adc1;
extern osSemaphoreId_t ADC_IRQ_BinarySemHandle;

static uint16_t adc_buf[ADC_MAX_CHANNEL];
static uint16_t adc_value[ADC_MAX_CHANNEL]; // adc模拟值
static uint32_t adc_sum[ADC_MAX_CHANNEL];

void USER_ADC_ConvCpltCallback(void)
{
	if (hdma_adc1.DmaBaseAddress->ISR & 0x2) // 传输完成中断
	{
		hdma_adc1.DmaBaseAddress->IFCR |= 0x2; // 清除中断标记位

		osSemaphoreRelease(ADC_IRQ_BinarySemHandle); // 释放信号量
	}
	else
	{
		hdma_adc1.DmaBaseAddress->IFCR = 0xffffffff;
	}
}

/*
 * 开始adc采集
 */
void adc_start(void)
{
	hdma_adc1.Instance->CCR &= (~DMA_CCR_EN);				  // 禁用通道
	hdma_adc1.Instance->CNDTR = ADC_MAX_CHANNEL;			  // 设置DMA传输数量
	hdma_adc1.Instance->CPAR = (uint32_t)&hadc1.Instance->DR; // 设置外设地址
	hdma_adc1.Instance->CMAR = (uint32_t)adc_buf;			  // 设置存储器地址
	hdma_adc1.Instance->CCR |= DMA_CCR_EN;					  // 使能通道

	hadc1.Instance->SR = 0; // 清除所有状态
	//	hadc1.Instance->CR2 |= ADC_CR2_ADON;//开始转换(重复写1会启动转换)
	hadc1.Instance->CR2 |= ADC_CR2_SWSTART; // 开始规则组通道转换，需要先开启规则通道的外部触发转换(hadc1.Instance->CR2 =| ADC_CR2_EXTTRIG)
}

/*
 * adc 初始化
 */
void user_adc_init(void)
{
	HAL_ADCEx_Calibration_Start(&hadc1);	 // 开始adc校准
	hdma_adc1.Instance->CCR &= (~DMA_IT_HT); // 禁止半传输中断
	hdma_adc1.Instance->CCR |= DMA_IT_TE;	 // 开启传输错误中断
	hdma_adc1.Instance->CCR |= DMA_IT_TC;	 // 开启传输完成中断

	hadc1.Instance->CR2 |= (ADC_CR2_ADON | ADC_CR2_DMA | ADC_CR2_EXTTRIG); // 使能ADC和开启DMA传输、开启规则通道的外部触发转换

	for (int i = 0; i < ADC_MAX_CHANNEL; i++)
	{
		adc_value[i] = 0;
		adc_sum[i] = 0;
	}

	//	adc_start();
}

/**
 * @brief       获取内部温度传感器温度值
 * @param       无
 * @retval      温度值(扩大了100倍,单位:℃.)
 */
int32_t get_interior_temperature(void)
{
	double temperature;
	int32_t result;

	temperature = (double)adc_value[2] * 3.3 / 4096;  // 转化为电压值
	temperature = (1.43 - temperature) / 0.0043 + 25; // 计算温度
	result = (temperature * 100);					  // 扩大100倍.
	return result;
}

void adc_buff_handler(void)
{
	if (osOK == osSemaphoreAcquire(ADC_IRQ_BinarySemHandle, portMAX_DELAY))
	{
		for (int i = 0; i < ADC_MAX_CHANNEL; i++)
		{
			adc_sum[i] += adc_buf[i];
			adc_sum[i] -= adc_value[i];
			adc_value[i] = adc_sum[i] >> 3;
		}

		adj_motor_control.adj_current[0] = (uint32_t)adc_value[0];// * 33000UL / 4095UL;
		adj_motor_control.adj_current[1] = (uint32_t)adc_value[1];// * 33000UL / 4095UL;
		osDelay(6); // 延时一段时间后再启动转换
		adc_start();
		//		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);//验证用
		//		get_interior_temperature();
	}
}

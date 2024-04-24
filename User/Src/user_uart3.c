/*
 * user_uart3.c
 *
 *  Created on: Jul 29, 2023
 *      Author: xiaoyeqing
 */

#include "usart.h"
#include "user_uart3.h"
#include "cmsis_os.h"
#include "user_can.h"
#include <string.h>
#include "user_motor.h"
#include "database.h"
#include "main.h"

extern volatile adj_motor_control_t adj_motor_control;
extern volatile osSemaphoreId_t uart3RxBinarySemHandle;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern uint16_t node_guarding_count;
extern node_message_t node_message_value;

volatile uart_rx_data_ty rxData = {0};
volatile bldc_motor_control_answer_t bldc_control_answer_data;
volatile bldc_motor_control_issued_t bldc_control_data;

#if USE_NEW_AGREEMENT
static const uint8_t bldc_io_table[] = {
	/*
	 * 只读指令
	 */
	IO_ainBldcPhaseCurrent,
	IO_ainCPU2Temperature,
	IO_ainBldcPhaseVoltage,
	IO_ainBldcSpeed, // BLDC实际转速(霍尔读取到的转速)
	IO_ainBldcError,
	IO_ainPumpBldcSpeed,
	IO_dinBldcIsMoving,

	/*
	 * 只写指令
	 */
	IO_doutSelectBldc,
	IO_doutBldcCW,
	IO_doutBldcCCW,
	IO_doutBldcFastBreaking,
	IO_doutPumpBldcCW,
	IO_doutPumpBldcCCW,
	IO_doutPumpBldcFastBreaking,
	IO_pwmBldc, // BLDC目标转速
	IO_pwmPumpBldc,

	IO_SubjoinBldcRampUPTime,	  // 研磨BLDC加速时间
	IO_SubjoinPumpBldcRampUPTime, // 水泵BLDC加速时间

	/*
	 * 可读可写指令
	 */
	IO_cntGrinderMotorTurns,
	IO_cntPumpMotorTurns,
};
static uint8_t bldc_io_table_number = sizeof(bldc_io_table);
#endif

#if 0
volatile database_data_t database_data;
volatile uint8_t syncCounter[ID_BLDC_max_number];

const database_item_t DataItemTable[] = {
	{ID_BLDC_select_motor,		sizeof(database_data.select_motor),			&database_data.select_motor,		&syncCounter[ID_BLDC_select_motor]},
	{ID_BLDC_direction,			sizeof(database_data.direction),			&database_data.direction,			&syncCounter[ID_BLDC_direction]},
	{ID_BLDC_ramp_up_time,		sizeof(database_data.ramp_up_time),			&database_data.ramp_up_time,		&syncCounter[ID_BLDC_ramp_up_time]},
	{ID_BLDC_motor_use,			sizeof(database_data.motor_use),			&database_data.motor_use,			&syncCounter[ID_BLDC_motor_use]},
	{ID_BLDC_set_speed,			sizeof(database_data.set_speed_val),		&database_data.set_speed_val,		&syncCounter[ID_BLDC_set_speed]},
	{ID_BLDC_stop,				sizeof(database_data.stop_val),				&database_data.stop_val,			&syncCounter[ID_BLDC_stop]},
	{ID_BLDC_phase_current,		sizeof(database_data.phase_current),		&database_data.phase_current,		&syncCounter[ID_BLDC_phase_current]},
	{ID_BLDC_phase_voltage,		sizeof(database_data.phase_voltage),		&database_data.phase_voltage,		&syncCounter[ID_BLDC_phase_voltage]},
	{ID_BLDC_fault,				sizeof(database_data.fault_val),			&database_data.fault_val,			&syncCounter[ID_BLDC_fault]},
	{ID_BLDC_read_speed,		sizeof(database_data.read_speed_val),		&database_data.read_speed_val,		&syncCounter[ID_BLDC_read_speed]},
	{ID_BLDC_rotation_counter,	sizeof(database_data.rotation_counter), 	&database_data.rotation_counter,	&syncCounter[ID_BLDC_rotation_counter]},
	{ID_BLDC_temperature,		sizeof(database_data.temperature),			&database_data.temperature,			&syncCounter[ID_BLDC_temperature]},
};
const uint8_t numberOfDataTable = sizeof(DataItemTable) / sizeof(DataItemTable[0]);

/*
 * 串口空闲中断回调
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	uint8_t len, cnt = huart->RxXferSize - huart->RxXferCount;
	rxData.Lenght += cnt;
	rxData.Tail += cnt;
	if(rxData.Tail >= UART3_RX_LEN)
	{
		rxData.Tail = 0;
	}

	if(rxData.Tail > rxData.Lenght)//(UART3_RX_LEN - rxData.Lenght) > (UART3_RX_LEN - rxData.Tail)
	{
		len = UART3_RX_LEN - rxData.Tail;
	}
	else
	{
		len = UART3_RX_LEN - rxData.Lenght;
	}

	HAL_UARTEx_ReceiveToIdle_IT(&huart3, (uint8_t*)&rxData.Buffer[rxData.Tail], len);
	osSemaphoreRelease(uart3RxBinarySemHandle);// 释放信号量
}

/*
 * 传输完成中断
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
#if 0
	if(rxData.Lenght > 0)
	{
		uint8_t len = UART3_RX_LEN - rxData.Head;
		if(len >= rxData.Lenght)
		{
			len = rxData.Lenght;
		}
		rxData.Lenght = rxData.Lenght - len;
		HAL_UART_Transmit_DMA(&huart3, (uint8_t*)&rxData.Buffer[rxData.Head], len);
		rxData.Head += len;
		if(rxData.Head >= UART3_RX_LEN)
		{
			rxData.Head = 0;
		}
	}
#endif
}
#endif

#if USE_NEW_AGREEMENT

extern uint8_t AllUartSyncCounter[IO_MaxNumber + IO_SubjoinNumber];
extern uint32_t AllPtrToData[IO_MaxNumber + IO_SubjoinNumber]; // IO数据

static const uint8_t PEC_LOOKUP_TABLE[256] =
	{0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D, 0x70,
	 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D, 0xE0, 0xE7,
	 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD, 0x90, 0x97, 0x9E,
	 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD, 0xC7, 0xC0, 0xC9, 0xCE,
	 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA, 0xB7, 0xB0, 0xB9, 0xBE, 0xAB,
	 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A, 0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C,
	 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A, 0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45,
	 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A, 0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
	 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4, 0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1,
	 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4, 0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56,
	 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44, 0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F,
	 0x28, 0x3D, 0x3A, 0x33, 0x34, 0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F,
	 0x6A, 0x6D, 0x64, 0x63, 0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A,
	 0x1D, 0x14, 0x13, 0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D,
	 0x84, 0x83, 0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4,
	 0xF3};

//=================================================================
/**
 *@brief     CRC Calculage function
 *@details
 *@param     uint8_t* Address:The address of the databuf.
 *@param     uint16_t Length: The length of the databuf.
 *@return    uint8_t  CRC: CRC Value
 */
uint8_t CRC_Calculate_t(uint8_t const *Address, uint16_t Length)
{
	uint8_t udata = 0;
	uint8_t remainder = 0;
	uint16_t currentPosition = 0;

	for (currentPosition = 0; currentPosition < Length;)
	{
		udata = Address[currentPosition] ^ (remainder);
#if 1
		remainder = PEC_LOOKUP_TABLE[udata] ^ 0;
#else
		remainder = PEC_LOOKUP_TABLE[udata] ^ (remainder << 8);
#endif
		++currentPosition;
	}
	return (remainder);
}

#else

#define POLYNOM 0xA001
uint16_t crc16_MODBUS(uint8_t *ptr, uint16_t len)
{
	uint8_t i;
	uint16_t crc = 0xffff;
	if (len == 0)
	{
		len = 1;
	}
	while (len--)
	{
		crc ^= *ptr;
		for (i = 0; i < 8; i++)
		{
			if (crc & 0x01)
			{
				crc >>= 1;
				crc ^= POLYNOM;
			}
			else
			{
				crc >>= 1;
			}
		}
		ptr++;
	}
	return (crc);
}

/*
转义待发送的数据,并发送
待发送数据中不包含帧头和帧尾
*/
void uart_send_escape_character(const unsigned char *buff, unsigned short len)
{
	if (len > 0)
	{
		static uint8_t tx_data[30];
		uint8_t tx_len = 0, i;
		tx_data[tx_len++] = FRAME_HEADER; // 存入帧头
		for (i = 0; i < len; i++)
		{
			if (ESCAPE_CHARACTER == buff[i])
			{
				tx_data[tx_len++] = 0x54;
				tx_data[tx_len++] = 0x01;
			}
			else if (FRAME_HEADER == buff[i])
			{
				tx_data[tx_len++] = 0x54;
				tx_data[tx_len++] = 0x02;
			}
			else
			{
				tx_data[tx_len++] = buff[i];
			}
		}
		tx_data[tx_len++] = FRAME_HEADER; // 存入帧尾
		uart3_dma_transmit_buff(tx_data, tx_len);
	}
}

/*
逆转义接收到的一帧数据
传入的数据中可包含帧头和帧尾
返回值：转义后的长度
*/
unsigned short uart_receive_escape_character(unsigned char *buff, unsigned short len)
{
	uint8_t rx_len = 0, i;
	if (len > 0)
	{
		for (i = 0; i < len; i++)
		{
			if (ESCAPE_CHARACTER == buff[i])
			{
				i++;
				//				if(i < len)
				{
					if (buff[i] == 0x01)
					{
						buff[rx_len++] = ESCAPE_CHARACTER;
					}
					else
					{
						buff[rx_len++] = FRAME_HEADER;
					}
				}
				//				else//数据出错了
				//				{
				//					buff[rx_len++] = buff[i - 1];
				//				}
			}
			else
			{
				buff[rx_len++] = buff[i];
			}
		}
	}
	return rx_len;
}
#endif

/*
 * 串口3 DMA发送数据
 */
void uart3_dma_transmit_buff(uint8_t *buff, uint16_t len)
{
#if 0
	if(hdma_usart3_tx.Instance->CNDTR > 0)
	{
		return;
	}
#endif
	hdma_usart3_tx.Instance->CCR &= (~DMA_CCR_EN);					// 禁用通道
	hdma_usart3_tx.Instance->CNDTR = len;							// 设置DMA传输数量
	hdma_usart3_tx.Instance->CPAR = (uint32_t)&huart3.Instance->DR; // 设置外设地址
	hdma_usart3_tx.Instance->CMAR = (uint32_t)buff;					// 设置存储器地址
	hdma_usart3_tx.Instance->CCR |= DMA_CCR_EN;						// 使能通道
}

/*
 * 串口3 DMA接收数据
 */
void uart3_dma_receive_data(void)
{
	uint8_t len, cnt = huart3.RxXferSize - hdma_usart3_rx.Instance->CNDTR;
	rxData.Lenght += cnt;
	rxData.Tail += cnt;
	if (rxData.Tail >= UART3_RX_LEN)
	{
		rxData.Tail = 0;
	}

	if (rxData.Tail > rxData.Lenght) //(UART3_RX_LEN - rxData.Lenght) > (UART3_RX_LEN - rxData.Tail)
	{
		len = UART3_RX_LEN - rxData.Tail;
	}
	else
	{
		len = UART3_RX_LEN - rxData.Lenght;
	}

	hdma_usart3_rx.Instance->CCR &= (~DMA_CCR_EN);							 // 禁用通道
	huart3.RxXferSize = len;												 // 记录传输数据
	hdma_usart3_rx.Instance->CNDTR = len;									 // 设置DMA传输数量
	hdma_usart3_rx.Instance->CPAR = (uint32_t)&huart3.Instance->DR;			 // 设置外设地址
	hdma_usart3_rx.Instance->CMAR = (uint32_t)(&rxData.Buffer[rxData.Tail]); // 设置存储器地址
	hdma_usart3_rx.Instance->CCR |= DMA_CCR_EN;								 // 使能通道
}

/*
 * 串口3中断处理
 */
void user_uart3_IRQHandler(void)
{
	if (huart3.Instance->SR & USART_SR_IDLE) // 空闲中断
	{
		uint32_t temp;
		temp = huart3.Instance->SR; // 清除空闲中断标记
		temp = huart3.Instance->DR; // 清除空闲中断标记

		uart3_dma_receive_data();
		osSemaphoreRelease(uart3RxBinarySemHandle); // 释放信号量
		UNUSED(temp);
	}
}

/*
 * DMA接收中断处理 串口3
 */
void user_uart3_RX_DMA_IRQHandler(void)
{
	if (hdma_usart3_rx.DmaBaseAddress->ISR & 0x200) // 传输完成中断
	{
		hdma_usart3_rx.DmaBaseAddress->IFCR |= 0x200; // 清除中断标记位

		uart3_dma_receive_data();
		//		osSemaphoreRelease(uart3RxBinarySemHandle);// 释放信号量
	}
	else
	{
		hdma_usart3_rx.DmaBaseAddress->IFCR = 0xffffffff;
	}
}

/*
 * 串口3 DMA发送中断处理(未调用)
 */
void user_uart3_TX_DMA_IRQHandler(void)
{
	if (hdma_usart3_tx.DmaBaseAddress->ISR & 0x20) // 传输完成中断
	{
		hdma_usart3_tx.DmaBaseAddress->IFCR |= 0x20; // 清除中断标记位

#if 0 // 验证用
		if(rxData.Lenght > 0)
		{
			uint8_t len = UART3_RX_LEN - rxData.Head;
			if(len >= rxData.Lenght)
			{
				len = rxData.Lenght;
			}
			rxData.Lenght = rxData.Lenght - len;

			uart3_dma_transmit_buff((uint8_t*)&rxData.Buffer[rxData.Head], len);
			rxData.Head += len;
			if(rxData.Head >= UART3_RX_LEN)
			{
				rxData.Head = 0;
			}
		}
#endif
	}
	else
	{
		hdma_usart3_rx.DmaBaseAddress->IFCR = 0xffffffff;
	}
}

/*
 * 串口初始化
 */
void user_uart3_init(void)
{
	hdma_usart3_rx.Instance->CCR &= (~DMA_IT_HT);  // 禁止半传输中断
	hdma_usart3_rx.Instance->CCR |= DMA_IT_TE;	   // 开启传输错误中断
	hdma_usart3_rx.Instance->CCR |= DMA_IT_TC;	   // 开启传输完成中断
	hdma_usart3_rx.Instance->CCR &= (~DMA_CCR_EN); // 禁用通道

	hdma_usart3_tx.Instance->CCR &= (~DMA_IT_HT); // 禁止半传输中断
	hdma_usart3_tx.Instance->CCR &= (~DMA_IT_TE); // 禁止传输错误中断
	//	hdma_usart3_tx.Instance->CCR |= DMA_IT_TC;//启用传输完成中断
	hdma_usart3_tx.Instance->CCR &= (~DMA_IT_TC);  // 禁止传输完成中断
	hdma_usart3_tx.Instance->CCR &= (~DMA_CCR_EN); // 禁用通道

	huart3.Instance->CR1 |= USART_CR1_IDLEIE; // 使能空闲中断
	huart3.Instance->CR1 |= USART_CR1_RE;	  // 使能
	huart3.Instance->CR1 |= USART_CR1_TE;	  // 使能

	huart3.Instance->CR1 &= (~USART_CR1_RXNEIE); // 禁止接收中断
	huart3.Instance->CR1 &= (~USART_CR1_TCIE);	 // 禁止发送中断
	huart3.Instance->CR3 |= USART_CR3_DMAR;		 // 使能DMA接收
	huart3.Instance->CR3 |= USART_CR3_DMAT;		 // 使能DMA发送

	rxData.Tail = 0;
	rxData.Head = 0;
	rxData.Lenght = 0;

	huart3.RxXferSize = UART3_RX_LEN;								// 记录传输数据
	hdma_usart3_rx.Instance->CNDTR = UART3_RX_LEN;					// 设置DMA传输数量
	hdma_usart3_rx.Instance->CPAR = (uint32_t)&huart3.Instance->DR; // 设置外设地址
	hdma_usart3_rx.Instance->CMAR = (uint32_t)(&rxData.Buffer[0]);	// 设置存储器地址
	hdma_usart3_rx.Instance->CCR |= DMA_CCR_EN;						// 使能通道
}

/*
 * 帧消息处理
 */
static uint8_t message_done = 0;
void message_handler(uint8_t *message, uint8_t len)
{
#if USE_NEW_AGREEMENT

	uint8_t temp;
	uint8_t headCounter = 2;
	uint32_t io_data;
	temp = CRC_Calculate_t(message, len - 1);
	if (message[len - 1] == temp)
	{
		while (message[headCounter] < IO_MaxNumber + IO_SubjoinNumber) // message[headCounter] != 0xff &&
		{
			temp = message[headCounter + 1] & 0x7f; // 数据长度
			if (headCounter + 2 + temp < len && (temp % 4) == 0 && (message[headCounter] + temp / 4) <= IO_MaxNumber + IO_SubjoinNumber)
			{
				if (AllUartSyncCounter[message[headCounter]] & 0x80) // 读数据
				{
					if (AllUartSyncCounter[message[headCounter]] == message[headCounter + 1]) // 指令相同
					{
						uint8_t *ptrMessage = &message[headCounter + 2];
						uint8_t data_len = 0;
						AllUartSyncCounter[message[headCounter]] = 0;
						while (data_len < temp)
						{
							io_data = ptrMessage[0] + (ptrMessage[1] << 8) + (ptrMessage[2] << 16) + (ptrMessage[3] << 24);
							ptrMessage += 4;

							updata_data(message[headCounter], io_data); // 更新数据
							message[headCounter]++;						// 连续写数据时IO需要加1
							data_len += 4;
						}
					}
				}
				else if (AllUartSyncCounter[message[headCounter]] == temp) // 写数据
				{
					if (memcmp(&AllPtrToData[message[headCounter]], &message[headCounter + 2], temp) == 0) // 数据相同
					{
						AllUartSyncCounter[message[headCounter]] = 0;
					}
				}
				headCounter += (2 + temp);
				if (headCounter >= len)
				{
					break;
				}
			}
			else
			{
				break;
			}
		}
		message_done = 1;
	}

#else

#define message_len_index 1
#define message_cmd_index 2
#define message_data_index 3	// 数据索引
#define STATE_REPORT_CMD_LEN 18 // 控制应答数据长度
	uint16_t crc16;
	len = uart_receive_escape_character(message, len);
	if (len == message[message_len_index] && len > 5)
	{
		crc16 = (message[len - 2] << 8) + message[len - 3];
		if (crc16 == crc16_MODBUS(&message[message_len_index], len - 4))
		{
			switch (message[message_cmd_index])
			{
			case STATE_REPORT_CMD:
			{
#if 0
				if(message[message_len_index] != STATE_REPORT_CMD_LEN)//长度不符合退出
				{
					break;
				}
#endif
				memcpy((uint8_t *)bldc_control_answer_data.buff, &message[message_data_index], sizeof(bldc_control_answer_data));
			}
			break;

			default:
				break;
			}
		}
	}
#endif
}

/*
 * 接收数据处理
 */
void uart3_rx_handler(void)
{
	//	if(osOK == osSemaphoreWait(uart3RxBinarySemHandle, 10))// 如果接收到信号量 osSemaphoreAcquire
	if (osOK == osSemaphoreAcquire(uart3RxBinarySemHandle, portMAX_DELAY)) // 如果接收到信号量
	{
#if USE_NEW_AGREEMENT

#define message_len (80)
#define Frame_Header (0xeb90)
#define Frame_Header1 (0xeb)
#define Frame_Header2 (0x90)
		static uint8_t message[message_len] = {0};
		static uint8_t message_index = 0;
		static uint32_t frame_data = 0;
		while (rxData.Lenght)
		{
			if (rxData.Head >= UART3_RX_LEN)
			{
				rxData.Head = 0;
			}
			frame_data <<= 8;
			frame_data |= rxData.Buffer[rxData.Head];
			if (message_index == 0)
			{
				if ((frame_data & 0xffff) == Frame_Header)
				{
					message[0] = Frame_Header1;
					message[1] = Frame_Header2;
					message_index = 2;
				}
			}
			else
			{
				message[message_index++] = rxData.Buffer[rxData.Head];
				if ((frame_data & 0xffffff00) == 0xffff0100)
				{
					message_handler(message, message_index); // 处理消息
					message_index = 0;
				}
				if (message_index >= message_len) // 超过最大长度,重新判定桢头
				{
					message_index = 0;
				}
			}
			rxData.Head++;
			rxData.Lenght--;
		}

#else

#if !TEST_CAN_USART
#define message_len 99 // 实际长度为30(29+1)
		static uint8_t message[message_len + 1] = {0}, len = 0;
		while (rxData.Lenght)
		{
			if (rxData.Head >= UART3_RX_LEN)
			{
				rxData.Head = 0;
			}
			if (rxData.Buffer[rxData.Head] == FRAME_HEADER)
			{
				if (message[0] == FRAME_HEADER && len > 5) // 有一帧完整数据
				{
					message[len++] = rxData.Buffer[rxData.Head];
					message_handler(message, len); // 处理消息
					message[0] = 0;				   // 重新查寻帧头
					len = 0;					   // 重新查寻帧头
					rxData.Head++;
					rxData.Lenght--;
					continue;
				}
				else
				{
					len = 0;
				}
			}
			message[len] = rxData.Buffer[rxData.Head];
			if (len < message_len)
			{
				len++;
			}
			rxData.Head++;
			rxData.Lenght--;
		}
#else				   // 测试
		if (rxData.Lenght > 0)
		{
			uint8_t len = UART3_RX_LEN - rxData.Head;
			if (len >= rxData.Lenght)
			{
				len = rxData.Lenght;
			}
			rxData.Lenght = rxData.Lenght - len;

			can_data_t test;
			test.can_id = 0x7ff;
			test.id_combine_10 = 1;
			if (len > 8)
			{
				test.can_dlc = 8;
			}
			else
			{
				test.can_dlc = len;
			}
			memcpy(test.message, (uint8_t *)&rxData.Buffer[rxData.Head], test.can_dlc);
			CAN1_Send(&test);

			rxData.Head += len;
			if (rxData.Head >= UART3_RX_LEN)
			{
				rxData.Head = 0;
			}
		}
#endif

#endif
	}
}

/*
 * 上电进入自检
 */
void selftest_control(void)
{
#define SELFTEST_TIME_1S (33)
#define SELFTEST_TIME_3S (100)
#define SELFTEST_TIME_10S (350)
#define SELFTEST_TIME_20S (SELFTEST_TIME_10S * 2)
#define SELFTEST_TIME_30S (SELFTEST_TIME_10S * 3)
#define SELFTEST_TIME_40S (SELFTEST_TIME_10S * 4)
	static uint8_t startup_flag = 1, selftest_flag = 0;
	static uint16_t time_cnt = 0;
	if (startup_flag == 0)
	{
		return;
	}
	if (startup_flag == 1)
	{
		if (HAL_GPIO_ReadPin(AUTO_TEST_GPIO_Port, AUTO_TEST_Pin) == GPIO_PIN_RESET)
		{
			time_cnt++;
			if (time_cnt >= SELFTEST_TIME_3S)
			{
				time_cnt = 0;
				selftest_flag = 1;
				startup_flag = 2;
			}
		}
		else
		{
			startup_flag = 0;
		}
	}
	if (selftest_flag)
	{
		bldc_control_data.target_speed = 1000;
		if (time_cnt < SELFTEST_TIME_10S)
		{
			bldc_control_data.select_motor = 0;
			bldc_control_data.motor_state = 1;
			bldc_control_data.motor_dir = 0;

			if (time_cnt < SELFTEST_TIME_10S - SELFTEST_TIME_3S)
			{
				adj_motor_control.adj_direction_cw = 1;
				adj_motor_control.adj_direction_ccw = 0;
				adj_motor_control.adj_select_motor = 0;
				adj_motor_control.adj_motor_speed = 90;
			}
			else
			{
				adj_motor_control.adj_direction_cw = 0;
				adj_motor_control.adj_direction_ccw = 0;
				adj_motor_control.adj_select_motor = 0;
				adj_motor_control.adj_motor_speed = 90;
			}
		}
		else if (time_cnt < SELFTEST_TIME_20S)
		{
			bldc_control_data.select_motor = 0;
			bldc_control_data.motor_state = 1;
			bldc_control_data.motor_dir = 1;
			if (time_cnt < SELFTEST_TIME_20S - SELFTEST_TIME_3S)
			{
				adj_motor_control.adj_direction_cw = 0;
				adj_motor_control.adj_direction_ccw = 1;
				adj_motor_control.adj_select_motor = 0;
				adj_motor_control.adj_motor_speed = 90;
			}
			else
			{
				adj_motor_control.adj_direction_cw = 0;
				adj_motor_control.adj_direction_ccw = 0;
				adj_motor_control.adj_select_motor = 0;
				adj_motor_control.adj_motor_speed = 90;
			}
		}
		else if (time_cnt < SELFTEST_TIME_30S)
		{
			bldc_control_data.select_motor = 1;
			bldc_control_data.motor_state = 1;
			bldc_control_data.motor_dir = 0;
			if (time_cnt < SELFTEST_TIME_30S - SELFTEST_TIME_3S)
			{
				adj_motor_control.adj_direction_cw = 1;
				adj_motor_control.adj_direction_ccw = 0;
				adj_motor_control.adj_select_motor = 1;
				adj_motor_control.adj_motor_speed = 90;
			}
			else
			{
				adj_motor_control.adj_direction_cw = 0;
				adj_motor_control.adj_direction_ccw = 0;
				adj_motor_control.adj_select_motor = 1;
				adj_motor_control.adj_motor_speed = 90;
			}
		}
		else if (time_cnt < SELFTEST_TIME_40S)
		{
			bldc_control_data.select_motor = 1;
			bldc_control_data.motor_state = 1;
			bldc_control_data.motor_dir = 1;
			if (time_cnt < SELFTEST_TIME_40S - SELFTEST_TIME_3S)
			{
				adj_motor_control.adj_direction_cw = 0;
				adj_motor_control.adj_direction_ccw = 1;
				adj_motor_control.adj_select_motor = 1;
				adj_motor_control.adj_motor_speed = 90;
			}
			else
			{
				adj_motor_control.adj_direction_cw = 0;
				adj_motor_control.adj_direction_ccw = 0;
				adj_motor_control.adj_select_motor = 1;
				adj_motor_control.adj_motor_speed = 90;
			}
		}
		time_cnt++;
		if (time_cnt >= SELFTEST_TIME_40S)
		{
			time_cnt = 0;
		}
	}
}

/*
 * 发送数据处理
 */
void uart3_tx_handler(void)
{
#if USE_NEW_AGREEMENT

#define uart3_tx_handler_count_time_100ms (100)

#define message_tx_len (90)
#define Frame_Header (0xeb90)
#define Frame_Header1 (0xeb)
#define Frame_Header2 (0x90)

	static uint8_t mesage_tx[message_tx_len];
	uint8_t index = 2, i;
	static uint8_t time_count = 0;

	time_count++;
//	if (time_count >= uart3_tx_handler_count_time_100ms)
	if (time_count >= uart3_tx_handler_count_time_100ms || message_done)
	{
		message_done = 0;
		time_count = 0;
		if (node_guarding_count > node_guarding_count_30s)
		{
			if (node_message_value.main_state != 0)
			{
				node_message_value.main_state = 0;
				reset_all_data();
			}
		}
		else if (node_guarding_count > node_guarding_count_2s)
		{
			mesage_tx[index++] = IO_doutBldcCW;
			mesage_tx[index++] = 0x08;
			mesage_tx[index++] = 0;
			mesage_tx[index++] = 0;
			mesage_tx[index++] = 0;
			mesage_tx[index++] = 0;
			mesage_tx[index++] = 0;
			mesage_tx[index++] = 0;
			mesage_tx[index++] = 0;
			mesage_tx[index++] = 0;
		}
		else
		{
			uart_read_io_data(IO_ainBldcPhaseCurrent);
			uart_read_io_buff(IO_ainCPU2Temperature, 3);
			uart_read_io_data(IO_ainBldcError);
			uart_read_io_data(IO_ainPumpBldcSpeed);
			uart_read_io_data(IO_dinBldcIsMoving);
			//		uart_read_io_data(IO_cntGrinderMotorTurns);
			uart_read_io_data(IO_cntGrinderMotorTurns);//可读可写的不要连续读
			uart_read_io_data(IO_cntPumpMotorTurns);//可读可写的不要连续读

			for (i = 0; i < bldc_io_table_number; i++)
			{
				if (AllUartSyncCounter[bldc_io_table[i]]) // && (AllUartSyncCounter[bldc_io_table[i]] % 4) == 0
				{
					//				mesage_tx[index++] = bldc_io_table[i];
					//				mesage_tx[index++] = AllUartSyncCounter[bldc_io_table[i]];
					if ((AllUartSyncCounter[bldc_io_table[i]] & 0x80) == 0) // 写数据
					{
						if (index + AllUartSyncCounter[bldc_io_table[i]] > (message_tx_len - 6))
						{
							break;
						}
						mesage_tx[index++] = bldc_io_table[i];
						mesage_tx[index++] = AllUartSyncCounter[bldc_io_table[i]];
						memcpy(&mesage_tx[index], &AllPtrToData[bldc_io_table[i]], AllUartSyncCounter[bldc_io_table[i]]);
						index += AllUartSyncCounter[bldc_io_table[i]];
					}
					else
					{
						if (index > (message_tx_len - 6))
						{
							break;
						}
						mesage_tx[index++] = bldc_io_table[i];
						mesage_tx[index++] = AllUartSyncCounter[bldc_io_table[i]];
					}
				}
			}
		}
		mesage_tx[0] = Frame_Header1;
		mesage_tx[1] = Frame_Header2;
		mesage_tx[index++] = 0xff;
		mesage_tx[index++] = 0xff;
		mesage_tx[index++] = 1;
		mesage_tx[index] = CRC_Calculate_t(mesage_tx, index);
		index++;
		uart3_dma_transmit_buff(mesage_tx, index);
	}
#else

#define DATA_LEN_INDEX 0  // 指令长度索引
#define CRC16_DATA_LEN 10 // 校验数据长度
	bldc_control_data.buff[0] = (uint32_t)&bldc_control_data.CRC16 - (uint32_t)&bldc_control_data.command + 5;
	bldc_control_data.buff[1] = STATE_ISSUED_CMD;
	selftest_control();
#if 0
	bldc_control_data.target_speed = 1000;
	bldc_control_data.motor_state = 1;
	bldc_control_data.select_motor = 1;
#endif
	bldc_control_data.CRC16 = crc16_MODBUS((uint8_t *)&bldc_control_data.buff[DATA_LEN_INDEX], CRC16_DATA_LEN);
	uart_send_escape_character((uint8_t *)&bldc_control_data.buff[0], 12);

#endif
}

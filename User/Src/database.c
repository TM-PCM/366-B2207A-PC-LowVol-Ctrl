/*
 * database.c
 *
 *  Created on: Feb 19, 2024
 *      Author: Yeqing.Xiao
 */

#include <string.h>
#include "can.h"
#include "user_can.h"
#include "cmsis_os.h"
#include "queue.h"
#include "semphr.h"
#include "user_uart3.h"
#include "user_adc.h"
#include "user_motor.h"
#include "user_tim.h"
#include "database.h"

extern osMutexId_t myMutexSendQueueHandle;
extern volatile adj_motor_control_t adj_motor_control;
extern tim_capture_t distance_inport2_pwm;
extern tim_capture_t distance_inport1_pwm;
extern CAN_HandleTypeDef hcan;
extern osMessageQueueId_t canTransferDataHandle;
extern uint16_t node_guarding_count;
volatile can_data_t can_transfer_message = {0};

#if 0

static uint8_t AllSendOnChangeFlag[IO_MaxNumber] = {0};
static uint16_t AllSendOnTimeTargetTime[IO_MaxNumber] = {0};
static uint16_t AllSendOnTimeCount[IO_MaxNumber] = {0};
static uint32_t AllPtrToData[IO_MaxNumber] = {0};

database_item_t DataItemTable[] = {
		{IO_ainBldcPhaseCurrent, &AllSendOnChangeFlag[0], &AllSendOnTimeTargetTime[0], &AllSendOnTimeCount[0], &AllPtrToData[0]},
};

#endif

uint8_t AllUartSyncCounter[IO_MaxNumber + IO_SubjoinNumber] = {0};					  // 串口发送计数
uint16_t AllSendTargetTime[SendTypeMaxNumber][IO_MaxNumber + IO_SubjoinNumber] = {0}; // 变化上报延迟时间和循环发送间隔时间
uint16_t AllSendTimeCount[IO_MaxNumber + IO_SubjoinNumber] = {0};					  // 时间计数
uint32_t AllPtrToData[IO_MaxNumber + IO_SubjoinNumber] = {0};						  // IO数据
send_attribute_t AllSendState[IO_MaxNumber + IO_SubjoinNumber] = {0};				  // 变化延迟发送和循环发送状态
send_queue_t AllSendQueue = {0};													  // 循环发送和变化后延迟发送队列

/*
 * 复位所有数据
 */
void reset_all_data(void)
{
	uint8_t i;
	AllSendQueue.lenght = 0;
	for (i = 0; i < IO_MaxNumber + IO_SubjoinNumber; i++)
	{
		AllUartSyncCounter[i] = 0;
		AllSendTargetTime[0][i] = 0;
		AllSendTargetTime[1][i] = 0;
		AllSendTimeCount[i] = 0;
		AllPtrToData[i] = 0;
		AllSendState[i].all_data = 0;
	}
}

/*
 * 串口读取指定IO的数据
 */
void uart_read_io_data(database_IO_index_t io)
{
	if (io >= IO_MaxNumber + IO_SubjoinNumber)
	{
		return;
	}
	if (AllUartSyncCounter[io] == 0)
	{
		AllUartSyncCounter[io] = 0x84;
	}
}

/*
 * 串口从指定IO读取任意个数据
 * ！！！可读可写的不要连续读
 */
void uart_read_io_buff(database_IO_index_t io, uint8_t number)
{
	if ((io + number) > IO_MaxNumber + IO_SubjoinNumber || number == 0 || number >= 30)
	{
		return;
	}
	if (AllUartSyncCounter[io] == 0)
	{
		AllUartSyncCounter[io] = 0x80 + number * 4;
	}
}

/*
 * 串口写入指定IO数据
 */
void uart_write_io_data(database_IO_index_t io)
{
	if (io >= IO_MaxNumber + IO_SubjoinNumber)
	{
		return;
	}
	AllUartSyncCounter[io] = 0x04;
}

/*
 * 串口从指定IO写入任意个数据
 */
void uart_write_io_buff(database_IO_index_t io, uint8_t number)
{
	if ((io + number) > IO_MaxNumber + IO_SubjoinNumber || number == 0 || number >= 30)
	{
		return;
	}
	AllUartSyncCounter[io] = number * 4;
}

/*
 * 存入队列
 */
void put_send_queue(database_IO_index_t io)
{
	xSemaphoreTake(myMutexSendQueueHandle, portMAX_DELAY); // 获取互斥量 MuxSem,没获取到则一直等待
	if (AllSendState[io].transfer_state == ENABLE || AllSendQueue.lenght >= IO_MaxNumber + IO_SubjoinNumber)
	{
		return;
	}
	AllSendQueue.io_queue[AllSendQueue.lenght] = io;
	AllSendQueue.lenght++;
	AllSendState[io].transfer_state = ENABLE;
	xSemaphoreGive(myMutexSendQueueHandle); // 给出互斥量
}

/*
 * index: 要删除的队列位置
 * 删除队列数据，并将后面数据前移
 */
void delete_queue(uint8_t index)
{
	xSemaphoreTake(myMutexSendQueueHandle, portMAX_DELAY); // 获取互斥量 MuxSem,没获取到则一直等待
	if (AllSendQueue.lenght > index)
	{
		AllSendState[AllSendQueue.io_queue[index]].all_data = 0;
		memcpy(&AllSendQueue.io_queue[index], &AllSendQueue.io_queue[index + 1], AllSendQueue.lenght - index - 1);
		AllSendQueue.lenght--;
	}
	xSemaphoreGive(myMutexSendQueueHandle); // 给出互斥量
}

/*
 * io: 要删除的io队列
 * 删除队列数据，并将后面数据前移
 */
void delete_queue_io(database_IO_index_t io)
{
	uint8_t i;
	for (i = 0; i < AllSendQueue.lenght; i++)
	{
	}
}

/*
 * 存入指定IO的数据
 */
void put_io_data(database_IO_index_t io, uint32_t data)
{
	if (io >= IO_MaxNumber + IO_SubjoinNumber)
	{
		return;
	}
	AllPtrToData[io] = data;
}

/*
 * 更新指定IO的数据，若开启变化上报，则向主机上报数据
 */
void updata_data(database_IO_index_t io, uint32_t data)
{
	if (io >= IO_MaxNumber + IO_SubjoinNumber)
	{
		return;
	}
	if (data != AllPtrToData[io])
	{
		AllPtrToData[io] = data;

		/*
		 * 如开启变化上报，则存入CAN总线发送队列
		 */
		if (AllSendState[io].transfer_type & TransferTypeOnChange)
		{
			if (AllSendState[io].transfer_state == ENABLE) // 在延时发送队列中
			{
				if (AllSendTargetTime[SendOnChangeIndex][io] == 0) //
				{
					AllSendTimeCount[io] = 0; // 重新计时，并上报一次
				}
				else
				{
					int32_t time = AllSendTargetTime[SendOnTimeIndex][io] - AllSendTimeCount[io];
					if (time > AllSendTargetTime[SendOnChangeIndex][io]) // 变化延迟时间小于循环间隔剩余时间
					{
						AllSendTimeCount[io] = AllSendTargetTime[SendOnTimeIndex][io] - AllSendTargetTime[SendOnChangeIndex][io];
					}
					AllSendState[io].on_change_delay_flag = 1;
					return;
				}
			}
			else
			{
				if (AllSendTargetTime[SendOnChangeIndex][io] != 0) // 只开启了变化后延迟上报
				{
					AllSendState[io].on_change_delay_flag = 1;
					AllSendTimeCount[io] = 0; // 重新计时
					put_send_queue(io);		  // 存入延时发送队列
					return;
				}
			}

			/*
			 * 存入CAN总线消息队列
			 */
			if (can_transfer_message.can_id != 0)
			{
				can_transfer_message.can_dlc = 8;
				can_transfer_message.id_combine_0_3 = 7;
				can_transfer_message.id_combine_10 = 1;
				//				can_transfer_message.can_IDE = CAN_ID_STD;
				//				can_transfer_message.can_RTR = CAN_RTR_DATA;

				can_transfer_message.can_message0_1 = 0;
				can_transfer_message.can_message2_3 = io;
				can_transfer_message.can_message4_7 = data;

				BaseType_t xHigherPriorityTaskWoken;
				xQueueSendFromISR(canTransferDataHandle, (void *)can_transfer_message.all_data, &xHigherPriorityTaskWoken); // 将数据存入队列
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);																// 如果需要的话进行一次任务切换
			}
		}
	}
}

/*
 * 获取CAN队列数据，并发送
 */
void CAN_transfer_queue_handler(void)
{
	static can_data_t temp_data;
	if (xQueueReceive(canTransferDataHandle, temp_data.all_data, portMAX_DELAY)) // 请求消息
	{
		while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) // 无可用邮箱，等待
		{
			osDelay(2);
		}
		CAN1_Send(&temp_data);
	}
}

/*
 * 查询定时发送和延时发送队列
 */
void basedata_queue_handler(void)
{
#define BASE_TIME (5) // 步进时间
	/*
	 * 更新本地数据到数据库
	 */
	updata_data(IO_ainGrinderDistanceSensor1LowTime, distance_inport2_pwm.tCaptureLow);
	updata_data(IO_ainGrinderDistanceSensor1HighTime, distance_inport2_pwm.tCaptureHigh);
	updata_data(IO_ainGrinderDistanceSensor2LowTime, distance_inport1_pwm.tCaptureLow);
	updata_data(IO_ainGrinderDistanceSensor2HighTime, distance_inport1_pwm.tCaptureHigh);
	updata_data(IO_ainDistanceSensor1State, distance_inport2_pwm.error_code);
	updata_data(IO_ainDistanceSensor2State, distance_inport1_pwm.error_code);
	updata_data(IO_ainCPU1Temperature, get_interior_temperature());

	if (AllPtrToData[IO_doutSelectDcMotor] == 1)
	{
		updata_data(IO_ainDcMotorError, adj_motor_control.adj_error[0]);
		updata_data(IO_ainDcMotorCurrent, adj_motor_control.adj_current[0]);

//		updata_data(IO_ainDcMotorCurrent, 0xff);
	}
	else
	{
		updata_data(IO_ainDcMotorError, adj_motor_control.adj_error[1]);
		updata_data(IO_ainDcMotorCurrent, adj_motor_control.adj_current[1]);

//		updata_data(IO_ainDcMotorCurrent, 0xee);
	}

	if (AllSendQueue.lenght)
	{
		uint8_t i = 0;
		for (; i < AllSendQueue.lenght;)
		{
			if (AllSendState[AllSendQueue.io_queue[i]].transfer_type == 0)
			{
				delete_queue(i); // 删除当前队列数据
			}
			else
			{
				AllSendTimeCount[AllSendQueue.io_queue[i]] += BASE_TIME;
				if (AllSendState[AllSendQueue.io_queue[i]].transfer_type & TransferTypeOnTime) // 启用定时发送
				{
					if (AllSendTimeCount[AllSendQueue.io_queue[i]] >= AllSendTargetTime[SendOnTimeIndex][AllSendQueue.io_queue[i]])
					{
						AllSendState[AllSendQueue.io_queue[i]].on_change_delay_flag = 0;
						AllSendTimeCount[AllSendQueue.io_queue[i]] = 0;

						can_transfer_message.can_message0_1 = 18;
						can_transfer_message.can_message2_3 = AllSendQueue.io_queue[i];
						can_transfer_message.can_message4_7 = AllPtrToData[AllSendQueue.io_queue[i]];

						BaseType_t xHigherPriorityTaskWoken;
						xQueueSendFromISR(canTransferDataHandle, (void *)can_transfer_message.all_data, &xHigherPriorityTaskWoken); // 将数据存入队列
						portYIELD_FROM_ISR(xHigherPriorityTaskWoken);																// 如果需要的话进行一次任务切换
					}
				}
				if (AllSendState[AllSendQueue.io_queue[i]].on_change_delay_flag) // 变化上报延迟
				{
					if (AllSendTimeCount[AllSendQueue.io_queue[i]] >= AllSendTargetTime[SendOnChangeIndex][AllSendQueue.io_queue[i]])
					{
						AllSendState[AllSendQueue.io_queue[i]].on_change_delay_flag = 0;
						AllSendTimeCount[AllSendQueue.io_queue[i]] = 0;

						can_transfer_message.can_message0_1 = 14;
						can_transfer_message.can_message2_3 = AllSendQueue.io_queue[i];
						can_transfer_message.can_message4_7 = AllPtrToData[AllSendQueue.io_queue[i]];

						BaseType_t xHigherPriorityTaskWoken;
						xQueueSendFromISR(canTransferDataHandle, (void *)can_transfer_message.all_data, &xHigherPriorityTaskWoken); // 将数据存入队列
						portYIELD_FROM_ISR(xHigherPriorityTaskWoken);																// 如果需要的话进行一次任务切换
					}
				}
				if (AllSendState[AllSendQueue.io_queue[i]].on_change_delay_flag == 0 &&
					(AllSendState[AllSendQueue.io_queue[i]].transfer_type & TransferTypeOnTime) == 0)
				{
					delete_queue(i); // 删除当前队列数据
				}
				else
				{
					i++;
				}
			}
		}
	}
}

/*
 * 判断指令是否为自动上报或变化上报消息
 */
void judge_is_reported_message(can_data_t *can)
{
	can_transfer_message.can_dlc = 8;
	can_transfer_message.can_id = can->can_id;
	can_transfer_message.id_combine_10 = 1;

	if (can->can_message0_1 == 0) // 变化上报
	{
		if (can->can_message4_5 == 0x8002)
		{
			AllSendTargetTime[SendOnChangeIndex][can->can_message2_3] = 0;
			AllSendState[can->can_message2_3].transfer_type |= TransferTypeOnChange;
			AllSendState[can->can_message2_3].on_change_delay_flag = 0;
		}
		else if (can->can_message4_5 == 0x02)
		{
			AllSendTargetTime[SendOnChangeIndex][can->can_message2_3] = 0;
			AllSendState[can->can_message2_3].transfer_type &= TransferTypeOnTime; // 清除值变化上报
			AllSendState[can->can_message2_3].on_change_delay_flag = 0;
		}
	}
	else if (can->can_message0_1 == 14) // 变化延时上报
	{
		if (can->can_message4_5 == 0)
		{
			AllSendTargetTime[SendOnChangeIndex][can->can_message2_3] = 0;
			AllSendState[can->can_message2_3].transfer_type &= TransferTypeOnTime; // 清除值变化上报
			AllSendState[can->can_message2_3].on_change_delay_flag = 0;
		}
		else
		{
			AllSendState[can->can_message2_3].on_change_delay_flag = 1;
			AllSendState[can->can_message2_3].transfer_type |= TransferTypeOnChange;
			AllSendTargetTime[SendOnChangeIndex][can->can_message2_3] = can->can_message4_5;
		}
	}
	else if (can->can_message0_1 == 18) // 定时上报
	{
		if (can->can_message4_5 == 0)
		{
			AllSendState[can->can_message2_3].transfer_type &= TransferTypeOnChange; // 清除定时上报
			AllSendTargetTime[SendOnTimeIndex][can->can_message2_3] = 0;
		}
		else
		{
			AllSendState[can->can_message2_3].transfer_type |= TransferTypeOnTime;
			AllSendTargetTime[SendOnTimeIndex][can->can_message2_3] = can->can_message4_5;
			put_send_queue(can->can_message2_3); // 添加上报队列
		}
	}
}

/*
 * user_can.c
 *
 *  Created on: Aug 1, 2023
 *      Author: Yeqing.Xiao
 */

#include "can.h"
#include "user_can.h"
#include "cmsis_os.h"
#include "queue.h"
#include "user_uart3.h"
#include "user_adc.h"
#include "user_motor.h"
#include "user_tim.h"
#include "database.h"
#include "main.h"
#include <string.h>

extern CAN_HandleTypeDef hcan;
extern osMessageQueueId_t canReceiveDataHandle;
extern osMessageQueueId_t canTransferDataHandle;
extern volatile bldc_motor_control_answer_t bldc_control_answer_data;
extern volatile bldc_motor_control_issued_t bldc_control_data;
extern volatile adj_motor_control_t adj_motor_control;
extern tim_capture_t distance_inport2_pwm;
extern tim_capture_t distance_inport1_pwm;
extern uint32_t AllPtrToData[IO_MaxNumber + IO_SubjoinNumber];
extern uint8_t AllUartSyncCounter[IO_MaxNumber + IO_SubjoinNumber];

static CAN_TxHeaderTypeDef TxMessage; // CAN发送的消息的消息头
static CAN_RxHeaderTypeDef RxMessage; // CAN接收的消息的消息头

static can_data_t RX_DATA; // CAN接收数据

node_message_t node_message_value;
uint16_t node_guarding_count = 0; // 节点保护计数

/*
 *
 */
void user_gpio_init(void)
{
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CAN_MODE2_GPIO_Port, CAN_MODE2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CAN_MODE1_GPIO_Port, CAN_MODE1_Pin, GPIO_PIN_SET);
}

/*
 * 用户数据初始化
 */
void user_data_init(void)
{
	node_message_value.main_state = 0;
	node_message_value.data1 = 0;
	node_message_value.important = 0;
	node_message_value.reserved = 0;
	node_message_value.SW_H = 1;
	node_message_value.SW_M = 1;
	node_message_value.SW_L = 0;
	node_message_value.HW = 0x14;
}

/*
 * 初始化CAN总线过滤器
 */
void user_can_init(void)
{
	CAN_FilterTypeDef sFilterConfig;
	can_16bit_filter_t id;
	can_16bit_filter_t mask;

	id.filter_EXID = 0;
	id.filter_IDE = 0; // 0表示标准ID，1表示扩展ID
	id.filter_RTR = 0; // 0表示数据帧，1表示远程帧
	id.filter_STID_0_3 = 0;
//	id.filter_STID_4_9 = 0;
	id.filter_STID_4_7 = IOP_k_KTYPE_MOT;
	id.filter_STID_8 = HAL_GPIO_ReadPin(CAN_ID_1_GPIO_Port, CAN_ID_1_Pin);
	id.filter_STID_9 = HAL_GPIO_ReadPin(CAN_ID_2_GPIO_Port, CAN_ID_2_Pin);
#if TEST_CAN_USART
	id.filter_STID_10 = 1; // 0表示主机到节点，1表示节点到主机
#else
	id.filter_STID_10 = 0; // 0表示主机到节点，1表示节点到主机
#endif

	mask.filter_EXID = 0;
	mask.filter_IDE = 1; // 只接收标准ID
	mask.filter_RTR = 1; // 只接收数据帧
	mask.filter_STID_0_3 = 0;
//	mask.filter_STID_4_9 = 0;
	mask.filter_STID_4_7 = 0xf;
	mask.filter_STID_8 = 1;
	mask.filter_STID_9 = 1;
	mask.filter_STID_10 = 1; // Only the message sent by the master is received. 只接收主机发送的信息

	sFilterConfig.FilterBank = 0;					   // CAN过滤器编号，范围0-13
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // CAN过滤器模式，掩码模式或列表模式
	sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT; // CAN过滤器尺度，16位或32位
	sFilterConfig.FilterIdHigh = id.val;			   // FXR1的16位过滤器的ID
	sFilterConfig.FilterIdLow = id.val;				   // FXR2的16位过滤器的ID
	sFilterConfig.FilterMaskIdHigh = mask.val;		   // FXR1的16位过滤器的屏蔽
	sFilterConfig.FilterMaskIdLow = mask.val;		   // FXR2的16位过滤器的屏蔽
	sFilterConfig.FilterFIFOAssignment = 0;			   // 报文通过过滤器的匹配后，存储到哪个FIFO
	sFilterConfig.FilterActivation = ENABLE;		   // 激活过滤器
	sFilterConfig.SlaveStartFilterBank = 0;

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_CAN_Start(&hcan) != HAL_OK)
	{
		Error_Handler();
	}

	/* 3. Enable CAN RX Interrupt */
	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		Error_Handler();
	}

	//	RxMessage.DLC = 8;
	//	RxMessage.IDE = CAN_ID_STD;
	//	RxMessage.RTR = CAN_RTR_DATA;	// 接收数据
	//	RxMessage.FilterMatchIndex = 0;	// 使用0号过滤器
	//	RxMessage.Timestamp = 0;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *canHandle)
{
	if (canHandle->Instance == CAN1)
	{
		can_data_t can_data;
		if (HAL_OK == HAL_CAN_GetRxMessage(canHandle, CAN_RX_FIFO0, &RxMessage, can_data.message))
		{
			//        	if(canReceiveDataHandle != NULL)//确保队列句柄已生成
			{
				can_data.can_dlc = RxMessage.DLC;
				can_data.can_id = RxMessage.StdId;
				can_data.can_IDE = RxMessage.IDE;
				can_data.can_RTR = RxMessage.RTR;

				BaseType_t xHigherPriorityTaskWoken;
				xQueueSendFromISR(canReceiveDataHandle, can_data.all_data, &xHigherPriorityTaskWoken); // 将数据存入队列
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);										   // 如果需要的话进行一次任务切换
			}
		}
	}
}

void CAN1_Send(can_data_t *can)
{
	// HAL_CAN_GetTxMailboxesFreeLevel() 此函数可获悉剩余可用发送邮箱数量，为0时即表示无可用邮箱
	uint32_t TxMailbox;
	TxMessage.IDE = CAN_ID_STD;				// 设置ID类型
	TxMessage.StdId = can->can_id;			// 设置ID号
	TxMessage.RTR = CAN_RTR_DATA;			// 设置传送数据帧
	TxMessage.DLC = can->can_dlc;			// 设置数据长度
	TxMessage.TransmitGlobalTime = DISABLE; // 不发送时间戳
	if (HAL_CAN_AddTxMessage(&hcan, &TxMessage, can->message, &TxMailbox) != HAL_OK)
	{
		//		Error_Handler();
	}
}

/*
 * 将CAN消息存入队列发送
 */
void CAN1_message_deposit_queue(can_data_t *message)
{
	//	message->can_dlc = 8;
	BaseType_t xHigherPriorityTaskWoken;
	xQueueSendFromISR(canTransferDataHandle, (void *)message->all_data, &xHigherPriorityTaskWoken); // 将数据存入队列
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);													// 如果需要的话进行一次任务切换
}

static void can_idC_message_handler(can_data_t *can)
{
	//	switch(can->can_message0_3)
	//	{
	//	case 0x01:
	//	{
	//
	//	}
	//	break;
	//
	//	default:
	//		break;
	//	}
}

static void can_idA_message_handler(can_data_t *can)
{
	//	switch(can->can_message0_3)
	//	{
	//	case 0x01:
	//	{
	//
	//	}
	//	break;
	//
	//	default:
	//		break;
	//	}
}

static void can_id1_message_handler(can_data_t *can)
{
	if (can->can_dlc == 1)
	{
		if (can->message[0] == 1) // init node
		{
			node_message_value.main_state = 1;
			node_guarding_count = 0;
			reset_all_data();
		}
		else if (can->message[0] == 4) // start node
		{
			node_guarding_count = 0;
			node_message_value.main_state = 4;
		}
		else if (can->message[0] == 50) // start node
		{
			node_guarding_count = 0;
			node_message_value.main_state = 50;
		}
	}
}

static void can_id0_message_handler(can_data_t *can)
{
	can->id_combine_10 = 1;
	can->can_dlc = 8;

	memcpy(can->message, node_message_value.all_buff, 8);
	CAN1_message_deposit_queue(can);
	node_guarding_count = 0;
}

static void can_id7_message_handler(can_data_t *can)
{
#if USE_NEW_AGREEMENT

	can->id_combine_10 = 1;
	switch (can->can_message2_3)
	{
	case IO_ainBldcPhaseCurrent:
	{
		if (can->can_message0_1 == 0x01 && can->can_dlc == 4) // 读取相电流值
		{
			can->can_message4_7 = AllPtrToData[IO_ainBldcPhaseCurrent];
			can->can_dlc = 8;
			CAN1_message_deposit_queue(can);
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_ainCPU1Temperature:
	{
		if (can->can_message0_1 == 0x01 && can->can_dlc == 4) // 读取值
		{
			can->can_message4_7 = AllPtrToData[IO_ainCPU1Temperature];
			can->can_dlc = 8;
			CAN1_message_deposit_queue(can);
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_ainCPU2Temperature:
	{
		if (can->can_message0_1 == 0x01 && can->can_dlc == 4) // 读取值
		{
			can->can_message4_7 = AllPtrToData[IO_ainCPU2Temperature];
			can->can_dlc = 8;
			CAN1_message_deposit_queue(can);
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_ainBldcPhaseVoltage:
	{
		if (can->can_message0_1 == 0x01 && can->can_dlc == 4) // 读取值
		{
			can->can_message4_7 = AllPtrToData[IO_ainBldcPhaseVoltage];
			can->can_dlc = 8;
			CAN1_message_deposit_queue(can);
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_ainBldcSpeed:
	{
		if (can->can_message0_1 == 0x01 && can->can_dlc == 4) // 读取值
		{
			can->can_message4_7 = AllPtrToData[IO_ainBldcSpeed];
			can->can_dlc = 8;
			CAN1_message_deposit_queue(can);
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_ainGrinderDistanceSensor1LowTime:
	{
		if (can->can_message0_1 == 0x01 && can->can_dlc == 4) // 读取值
		{
			can->can_message4_7 = AllPtrToData[IO_ainGrinderDistanceSensor1LowTime];
			can->can_dlc = 8;
			CAN1_message_deposit_queue(can);
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_ainGrinderDistanceSensor1HighTime:
	{
		if (can->can_message0_1 == 0x01 && can->can_dlc == 4) // 读取值
		{
			can->can_message4_7 = AllPtrToData[IO_ainGrinderDistanceSensor1HighTime];
			can->can_dlc = 8;
			CAN1_message_deposit_queue(can);
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_ainBldcError:
	{
		if (can->can_message0_1 == 0x01 && can->can_dlc == 4) // 读取值
		{
			can->can_message4_7 = AllPtrToData[IO_ainBldcError];
			can->can_dlc = 8;
			CAN1_message_deposit_queue(can);
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_ainDcMotorError:
	{
		if (can->can_message0_1 == 0x01 && can->can_dlc == 4) // 读取值
		{
			can->can_message4_7 = AllPtrToData[IO_ainDcMotorError];
			can->can_dlc = 8;
			CAN1_message_deposit_queue(can);
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_ainDcMotorCurrent:
	{
		if (can->can_message0_1 == 0x01 && can->can_dlc == 4) // 读取值
		{
			can->can_message4_7 = AllPtrToData[IO_ainDcMotorCurrent];
			can->can_dlc = 8;
			CAN1_message_deposit_queue(can);
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_ainDistanceSensor1State:
	{
		if (can->can_message0_1 == 0x01 && can->can_dlc == 4) // 读取值
		{
			can->can_message4_7 = AllPtrToData[IO_ainDistanceSensor1State];
			can->can_dlc = 8;
			CAN1_message_deposit_queue(can);
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_ainDistanceSensor2State:
	{
		if (can->can_message0_1 == 0x01 && can->can_dlc == 4) // 读取值
		{
			can->can_message4_7 = AllPtrToData[IO_ainDistanceSensor2State];
			can->can_dlc = 8;
			CAN1_message_deposit_queue(can);
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_ainGrinderDistanceSensor2LowTime:
	{
		if (can->can_message0_1 == 0x01 && can->can_dlc == 4) // 读取值
		{
			can->can_message4_7 = AllPtrToData[IO_ainGrinderDistanceSensor2LowTime];
			can->can_dlc = 8;
			CAN1_message_deposit_queue(can);
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_ainGrinderDistanceSensor2HighTime:
	{
		if (can->can_message0_1 == 0x01 && can->can_dlc == 4) // 读取值
		{
			can->can_message4_7 = AllPtrToData[IO_ainGrinderDistanceSensor2HighTime];
			can->can_dlc = 8;
			CAN1_message_deposit_queue(can);
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_ainPumpBldcSpeed:
	{
		if (can->can_message0_1 == 0x01 && can->can_dlc == 4) // 读取值
		{
			can->can_message4_7 = AllPtrToData[IO_ainPumpBldcSpeed];
			can->can_dlc = 8;
			CAN1_message_deposit_queue(can);
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_dinDistanceSensor1:
	{
		if (can->can_message0_1 == 0x01 && can->can_dlc == 4) // 读取值
		{
			can->can_message4_7 = AllPtrToData[IO_dinDistanceSensor1];
			can->can_dlc = 8;
			CAN1_message_deposit_queue(can);
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_dinDistanceSensor2:
	{
		if (can->can_message0_1 == 0x01 && can->can_dlc == 4) // 读取值
		{
			can->can_message4_7 = AllPtrToData[IO_dinDistanceSensor2];
			can->can_dlc = 8;
			CAN1_message_deposit_queue(can);
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_dinBldcIsMoving:
	{
		if (can->can_message0_1 == 0x01 && can->can_dlc == 4) // 读取值
		{
			can->can_message4_7 = AllPtrToData[IO_dinBldcIsMoving];
			can->can_dlc = 8;
			CAN1_message_deposit_queue(can);
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_dinDcMotorIsMoving:
	{
		if (can->can_message0_1 == 0x01 && can->can_dlc == 4) // 读取值
		{
			can->can_message4_7 = AllPtrToData[IO_dinDcMotorIsMoving];
			can->can_dlc = 8;
			CAN1_message_deposit_queue(can);
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_doutSelectBldc:
	{
		if (can->can_message0_1 == 0x01)
		{
			if (can->can_dlc == 4) // 读取值
			{
				can->can_message4_7 = AllPtrToData[IO_doutSelectBldc];
				can->can_dlc = 8;
				CAN1_message_deposit_queue(can);
				break;
			}
			else if (can->can_dlc == 6) // 设置值
			{
				updata_data(IO_doutSelectBldc, can->can_message4_5);
			}
			else if (can->can_dlc == 8) // 设置值
			{
				updata_data(IO_doutSelectBldc, can->can_message4_7);
			}
			uart_write_io_data(IO_doutSelectBldc);
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_doutBldcCW:
	{
		if (can->can_message0_1 == 0x01)
		{
			if (can->can_dlc == 4) // 读取值
			{
				can->can_message4_7 = AllPtrToData[IO_doutBldcCW];
				can->can_dlc = 8;
				CAN1_message_deposit_queue(can);
				break;
			}
			else if (can->can_dlc == 6) // 设置值
			{
				updata_data(IO_doutBldcCW, can->can_message4_5);
			}
			else if (can->can_dlc == 8) // 设置值
			{
				updata_data(IO_doutBldcCW, can->can_message4_7);
			}

			if (AllPtrToData[IO_doutBldcCCW] != 0)
			{
				updata_data(IO_doutBldcCCW, 0);
				uart_write_io_buff(IO_doutBldcCW, 2);
				break;
			}
			uart_write_io_data(IO_doutBldcCW);
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_doutBldcCCW:
	{
		if (can->can_message0_1 == 0x01)
		{
			if (can->can_dlc == 4) // 读取值
			{
				can->can_message4_7 = AllPtrToData[IO_doutBldcCCW];
				can->can_dlc = 8;
				CAN1_message_deposit_queue(can);
				break;
			}
			else if (can->can_dlc == 6) // 设置值
			{
				updata_data(IO_doutBldcCCW, can->can_message4_5);
			}
			else if (can->can_dlc == 8) // 设置值
			{
				updata_data(IO_doutBldcCCW, can->can_message4_7);
			}
			if (AllPtrToData[IO_doutBldcCW] != 0)
			{
				updata_data(IO_doutBldcCW, 0);
				uart_write_io_buff(IO_doutBldcCW, 2);
				break;
			}
			uart_write_io_data(IO_doutBldcCCW);
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_doutBldcFastBreaking:
	{
		if (can->can_message0_1 == 0x01)
		{
			if (can->can_dlc == 4) // 读取值
			{
				can->can_message4_7 = AllPtrToData[IO_doutBldcFastBreaking];
				can->can_dlc = 8;
				CAN1_message_deposit_queue(can);
				break;
			}
			else if (can->can_dlc == 6) // 设置值
			{
				updata_data(IO_doutBldcFastBreaking, can->can_message4_5);
			}
			else if (can->can_dlc == 8) // 设置值
			{
				updata_data(IO_doutBldcFastBreaking, can->can_message4_7);
			}
			if (AllPtrToData[IO_doutBldcFastBreaking] != 0)
			{
				updata_data(IO_doutBldcCW, 0);
				updata_data(IO_doutBldcCCW, 0);
				uart_write_io_buff(IO_doutBldcCW, 2);
				uart_write_io_data(IO_doutBldcFastBreaking);
				break;
			}
			uart_write_io_data(IO_doutBldcFastBreaking);
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_doutSelectDcMotor:
	{
		if (can->can_message0_1 == 0x01)
		{
			if (can->can_dlc == 4) // 读取值
			{
				can->can_message4_7 = AllPtrToData[IO_doutSelectDcMotor];
				can->can_dlc = 8;
				CAN1_message_deposit_queue(can);
				break;
			}
			else if (can->can_dlc == 6) // 设置值
			{
				updata_data(IO_doutSelectDcMotor, can->can_message4_5);
			}
			else if (can->can_dlc == 8) // 设置值
			{
				updata_data(IO_doutSelectDcMotor, can->can_message4_7);
			}
			if (AllPtrToData[IO_doutSelectDcMotor] == 0)
			{
				adj_motor_control.adj_select_motor = 0;
			}
			else
			{
				adj_motor_control.adj_select_motor = 1;
			}
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_doutDcMotorCW:
	{
		if (can->can_message0_1 == 0x01)
		{
			if (can->can_dlc == 4) // 读取值
			{
				can->can_message4_7 = AllPtrToData[IO_doutDcMotorCW];
				can->can_dlc = 8;
				CAN1_message_deposit_queue(can);
				break;
			}
			else if (can->can_dlc == 6) // 设置值
			{
				updata_data(IO_doutDcMotorCW, can->can_message4_5);
			}
			else if (can->can_dlc == 8) // 设置值
			{
				updata_data(IO_doutDcMotorCW, can->can_message4_7);
			}

			if (AllPtrToData[IO_doutDcMotorCW] == 0)
			{
				adj_motor_control.adj_direction_cw = 0;
			}
			else
			{
				adj_motor_control.adj_direction_cw = 1;
			}
			if (AllPtrToData[IO_doutDcMotorCW] != 0 && AllPtrToData[IO_doutDcMotorCCW] != 0)
			{
				updata_data(IO_doutDcMotorCCW, 0);
				adj_motor_control.adj_direction_ccw = 0;
			}
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_doutDcMotorCCW:
	{
		if (can->can_message0_1 == 0x01)
		{
			if (can->can_dlc == 4) // 读取值
			{
				can->can_message4_7 = AllPtrToData[IO_doutDcMotorCCW];
				can->can_dlc = 8;
				CAN1_message_deposit_queue(can);
				break;
			}
			else if (can->can_dlc == 6) // 设置值
			{
				updata_data(IO_doutDcMotorCCW, can->can_message4_5);
			}
			else if (can->can_dlc == 8) // 设置值
			{
				updata_data(IO_doutDcMotorCCW, can->can_message4_7);
			}

			if (AllPtrToData[IO_doutDcMotorCCW] == 0)
			{
				adj_motor_control.adj_direction_ccw = 0;
			}
			else
			{
				adj_motor_control.adj_direction_ccw = 1;
			}

			if (AllPtrToData[IO_doutDcMotorCCW] != 0 && AllPtrToData[IO_doutDcMotorCW] != 0)
			{
				updata_data(IO_doutDcMotorCW, 0);
				adj_motor_control.adj_direction_cw = 0;
			}
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_doutDcMotorFastBreaking:
	{
		if (can->can_message0_1 == 0x01)
		{
			if (can->can_dlc == 4) // 读取值
			{
				can->can_message4_7 = AllPtrToData[IO_doutDcMotorFastBreaking];
				can->can_dlc = 8;
				CAN1_message_deposit_queue(can);
				break;
			}
			else if (can->can_dlc == 6) // 设置值
			{
				updata_data(IO_doutDcMotorFastBreaking, can->can_message4_5);
			}
			else if (can->can_dlc == 8) // 设置值
			{
				updata_data(IO_doutDcMotorFastBreaking, can->can_message4_7);
			}
			if (AllPtrToData[IO_doutDcMotorFastBreaking])
			{
				updata_data(IO_doutDcMotorCW, 0);
				updata_data(IO_doutDcMotorCCW, 0);
				adj_motor_control.adj_direction_cw = 0;
				adj_motor_control.adj_direction_ccw = 0;
				adj_motor_control.adj_fast_braking_flag = 1; // 快速关标志位
			}
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_doutPumpBldcCW:
	{
		if (can->can_message0_1 == 0x01)
		{
			if (can->can_dlc == 4) // 读取值
			{
				can->can_message4_7 = AllPtrToData[IO_doutPumpBldcCW];
				can->can_dlc = 8;
				CAN1_message_deposit_queue(can);
				break;
			}
			else if (can->can_dlc == 6) // 设置值
			{
				updata_data(IO_doutPumpBldcCW, can->can_message4_5);
			}
			else if (can->can_dlc == 8) // 设置值
			{
				updata_data(IO_doutPumpBldcCW, can->can_message4_7);
			}

			if (AllPtrToData[IO_doutPumpBldcCCW] != 0)
			{
				updata_data(IO_doutPumpBldcCCW, 0);
				uart_write_io_buff(IO_doutPumpBldcCW, 2);
				break;
			}
			uart_write_io_data(IO_doutPumpBldcCW);
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_doutPumpBldcCCW:
	{
		if (can->can_message0_1 == 0x01)
		{
			if (can->can_dlc == 4) // 读取值
			{
				can->can_message4_7 = AllPtrToData[IO_doutPumpBldcCCW];
				can->can_dlc = 8;
				CAN1_message_deposit_queue(can);
				break;
			}
			else if (can->can_dlc == 6) // 设置值
			{
				updata_data(IO_doutPumpBldcCCW, can->can_message4_5);
			}
			else if (can->can_dlc == 8) // 设置值
			{
				updata_data(IO_doutPumpBldcCCW, can->can_message4_7);
			}
			if (AllPtrToData[IO_doutPumpBldcCW] != 0)
			{
				updata_data(IO_doutPumpBldcCW, 0);
				uart_write_io_buff(IO_doutPumpBldcCW, 2);
				break;
			}
			uart_write_io_data(IO_doutPumpBldcCCW);
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_doutPumpBldcFastBreaking:
	{
		if (can->can_message0_1 == 0x01)
		{
			if (can->can_dlc == 4) // 读取值
			{
				can->can_message4_7 = AllPtrToData[IO_doutPumpBldcFastBreaking];
				can->can_dlc = 8;
				CAN1_message_deposit_queue(can);
				break;
			}
			else if (can->can_dlc == 6) // 设置值
			{
				updata_data(IO_doutPumpBldcFastBreaking, can->can_message4_5);
			}
			else if (can->can_dlc == 8) // 设置值
			{
				updata_data(IO_doutPumpBldcFastBreaking, can->can_message4_7);
			}
			if (AllPtrToData[IO_doutPumpBldcFastBreaking] != 0)
			{
				updata_data(IO_doutPumpBldcCW, 0);
				updata_data(IO_doutPumpBldcCCW, 0);
				uart_write_io_buff(IO_doutPumpBldcCW, 2);
				uart_write_io_data(IO_doutPumpBldcFastBreaking);
				break;
			}
			uart_write_io_data(IO_doutPumpBldcFastBreaking);
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_pwmBldc:
	{
		if (can->can_message0_1 == 0x03) // 设置目标速度
		{
			if (can->can_dlc == 4) // 读取值
			{
				can->can_message4_7 = AllPtrToData[IO_pwmBldc];
				can->can_dlc = 8;
				CAN1_message_deposit_queue(can);
				break;
			}
			else if (can->can_dlc == 6) // 设置值
			{
				updata_data(IO_pwmBldc, can->can_message4_5);
			}
			else if (can->can_dlc == 8) // 设置值
			{
				updata_data(IO_pwmBldc, can->can_message4_7);
			}
			uart_write_io_data(IO_pwmBldc); // 更新数据到BLDC板
		}
		else if (can->can_message0_1 == 22) // 设置加速时间
		{
			if (can->can_dlc == 4) // 读取值
			{
				can->can_message4_7 = AllPtrToData[IO_SubjoinBldcRampUPTime];
				can->can_dlc = 8;
				CAN1_message_deposit_queue(can);
				break;
			}
			else if (can->can_dlc == 6) // 设置值
			{
				updata_data(IO_SubjoinBldcRampUPTime, can->can_message4_5);
			}
			else if (can->can_dlc == 8) // 设置值
			{
				updata_data(IO_SubjoinBldcRampUPTime, can->can_message4_7);
			}
			uart_write_io_data(IO_SubjoinBldcRampUPTime); // 更新数据到BLDC板
		}
		else
		{
		}
	}
	break;

	case IO_pwmDcMotor:
	{
		if (can->can_message0_1 == 0x03) // 设置目标速度
		{
			if (can->can_dlc == 4) // 读取值
			{
				can->can_message4_7 = AllPtrToData[IO_pwmDcMotor];
				can->can_dlc = 8;
				CAN1_message_deposit_queue(can);
				break;
			}
			else if (can->can_dlc == 6) // 设置值
			{
				updata_data(IO_pwmDcMotor, can->can_message4_5);
			}
			else if (can->can_dlc == 8) // 设置值
			{
				updata_data(IO_pwmDcMotor, can->can_message4_7);
			}
			adj_motor_control.adj_motor_speed = AllPtrToData[IO_pwmDcMotor];
		}
		else if (can->can_message0_1 == 22) // 设置加速时间
		{
		}
		else
		{
		}
	}
	break;

	case IO_pwmPumpBldc:
	{
		if (can->can_message0_1 == 0x03) // 设置目标速度
		{
			if (can->can_dlc == 4) // 读取值
			{
				can->can_message4_7 = AllPtrToData[IO_pwmPumpBldc];
				can->can_dlc = 8;
				CAN1_message_deposit_queue(can);
				break;
			}
			else if (can->can_dlc == 6) // 设置值
			{
				updata_data(IO_pwmPumpBldc, can->can_message4_5);
			}
			else if (can->can_dlc == 8) // 设置值
			{
				updata_data(IO_pwmPumpBldc, can->can_message4_7);
			}
			uart_write_io_data(IO_pwmPumpBldc); // 更新数据到BLDC板
		}
		else if (can->can_message0_1 == 22) // 设置加速时间
		{
			if (can->can_dlc == 4) // 读取值
			{
				can->can_message4_7 = AllPtrToData[IO_SubjoinPumpBldcRampUPTime];
				can->can_dlc = 8;
				CAN1_message_deposit_queue(can);
				break;
			}
			else if (can->can_dlc == 6) // 设置值
			{
				updata_data(IO_SubjoinPumpBldcRampUPTime, can->can_message4_5);
			}
			else if (can->can_dlc == 8) // 设置值
			{
				updata_data(IO_SubjoinPumpBldcRampUPTime, can->can_message4_7);
			}
			uart_write_io_data(IO_SubjoinPumpBldcRampUPTime); // 更新数据到BLDC板
		}
		else
		{
		}
	}
	break;

	case IO_cntGrinderMotorTurns:
	{
		if (can->can_message0_1 == 0x01 && can->can_dlc == 4) // 读取值
		{
			can->can_message4_7 = AllPtrToData[IO_cntGrinderMotorTurns];
			can->can_dlc = 8;
			CAN1_message_deposit_queue(can);
		}
		else if (can->can_message0_1 == 0x01 && can->can_dlc == 8)
		{
			uart_write_io_data(IO_cntGrinderMotorTurns); // 更新数据到BLDC板
			AllPtrToData[IO_cntGrinderMotorTurns] = can->can_message4_7;
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case IO_cntPumpMotorTurns:
	{
		if (can->can_message0_1 == 0x01 && can->can_dlc == 4) // 读取值
		{
			can->can_message4_7 = AllPtrToData[IO_cntPumpMotorTurns];
			can->can_dlc = 8;
			CAN1_message_deposit_queue(can);
		}
		else if (can->can_message0_1 == 0x01 && can->can_dlc == 8)
		{
			uart_write_io_data(IO_cntPumpMotorTurns); // 更新数据到BLDC板
			AllPtrToData[IO_cntPumpMotorTurns] = can->can_message4_7;
		}
		else
		{
			judge_is_reported_message(can);
		}
	}
	break;

	case 0xff: // 设置节点模式模式
	{

		if (can->can_message0_1 == 0x01 && can->can_dlc == 4) // 读取值
		{
			can->can_message4_5 = node_message_value.main_state;
			can->can_message6_7 = node_guarding_count;
			can->can_dlc = 8;
			CAN1_message_deposit_queue(can);
		}
		else if (can->can_message0_1 == 0x01 && can->can_dlc == 5)
		{
			node_message_value.main_state = can->message[4];
			node_guarding_count = 0;
		}
	}
	break;

	default:
		break;
	}
#else
	switch (can->can_message0_3)
	{
	case id_bldc_motor_use:
	case id_bldc_select_motor:
	{
		if (can->can_dlc == 6)
		{
			if (can->can_message4_5 == 0)
			{
				bldc_control_data.select_motor = 0;
			}
			else if (can->can_message4_5 == 1)
			{
				bldc_control_data.select_motor = 1;
			}
		}
		else if (can->can_dlc == 4)
		{
			can->id_combine_10 = 1;
			can->can_dlc = 6;
			can->can_message4_5 = bldc_control_answer_data.select_motor;
			CAN1_Send(can);
		}
	}
	break;

	case id_bldc_direction_cw:
	{
		if (can->can_dlc == 6)
		{
			if (can->can_message4_5 == 0)
			{
				bldc_control_data.motor_state = 0;
			}
			else if (can->can_message4_5 == 1)
			{
				bldc_control_data.motor_state = 1;
				bldc_control_data.motor_dir = 0;
			}
		}
		else if (can->can_dlc == 4)
		{
			can->id_combine_10 = 1;
			can->can_dlc = 6;
			if (bldc_control_answer_data.motor_state == 1 && bldc_control_answer_data.motor_dir == 0) // 正在正转
			{
				can->can_message4_5 = 1;
			}
			else
			{
				can->can_message4_5 = 0;
			}
			CAN1_Send(can);
		}
	}
	break;

	case id_bldc_direction_ccw:
	{
		if (can->can_dlc == 6)
		{
			if (can->can_message4_5 == 0)
			{
				bldc_control_data.motor_state = 0;
			}
			else if (can->can_message4_5 == 1)
			{
				bldc_control_data.motor_state = 1;
				bldc_control_data.motor_dir = 1;
			}
		}
		else if (can->can_dlc == 4)
		{
			can->id_combine_10 = 1;
			can->can_dlc = 6;
			if (bldc_control_answer_data.motor_state == 1 && bldc_control_answer_data.motor_dir == 1) // 正在反转
			{
				can->can_message4_5 = 1;
			}
			else
			{
				can->can_message4_5 = 0;
			}
			CAN1_Send(can);
		}
	}
	break;

	case id_bldc_ramp_up_time:
	{
		if (can->can_dlc == 4)
		{
			can->id_combine_10 = 1;
			can->can_dlc = 6;
			can->can_message4_5 = bldc_control_data.ramp_up_time;
			CAN1_Send(can);
		}
		else if (can->can_dlc == 6)
		{
			bldc_control_data.ramp_up_time = can->can_message4_5;
		}
	}
	break;

	case id_bldc_motor_speed:
	{
		if (can->can_dlc == 4)
		{
			can->id_combine_10 = 1;
			can->can_dlc = 6;
			can->can_message4_5 = bldc_control_data.target_speed;
			CAN1_Send(can);
		}
		else if (can->can_dlc == 6)
		{
			bldc_control_data.target_speed = can->can_message4_5;
		}
	}
	break;

	case id_bldc_fast_braking:
	{
		if (can->can_dlc == 4)
		{
			can->id_combine_10 = 1;
			can->can_dlc = 6;
			can->can_message4_5 = bldc_control_data.fast_braking;
			CAN1_Send(can);
		}
		else if (can->can_dlc == 6)
		{
			bldc_control_data.fast_braking = can->can_message4_5;
			if (can->can_message4_5 == 0)
			{
				bldc_control_data.motor_state = 0;
			}
		}
	}
	break;

	case id_bldc_phase_current:
	{
		if (can->can_dlc == 4)
		{
			can->id_combine_10 = 1;
			can->can_dlc = 6;
			can->can_message4_5 = bldc_control_answer_data.motor_current;
			CAN1_Send(can);
		}
	}
	break;

	case id_bldc_phase_voltage:
	{
		if (can->can_dlc == 4)
		{
			can->id_combine_10 = 1;
			can->can_dlc = 6;
			can->can_message4_5 = bldc_control_answer_data.motor_voltage;
			CAN1_Send(can);
		}
	}
	break;

	case id_bldc_error:
	{
		if (can->can_dlc == 4)
		{
			can->id_combine_10 = 1;
			can->can_dlc = 6;
			can->can_message4_5 = bldc_control_answer_data.fault_code.val;
			CAN1_Send(can);
		}
	}
	break;

	case id_bldc_speed:
	{
		if (can->can_dlc == 4)
		{
			can->id_combine_10 = 1;
			can->can_dlc = 6;
			can->can_message4_5 = bldc_control_answer_data.actual_speed;
			CAN1_Send(can);
		}
	}
	break;

	case id_bldc_temp_mcu1:
	{
		if (can->can_dlc == 4)
		{
			can->id_combine_10 = 1;
			can->can_dlc = 6;
			can->can_message4_5 = get_interior_temperature();
			CAN1_Send(can);
		}
	}
	break;

	case id_bldc_temp_mcu2:
	{
		if (can->can_dlc == 4)
		{
			can->id_combine_10 = 1;
			can->can_dlc = 6;
			can->can_message4_5 = bldc_control_answer_data.mcu_temperature;
			CAN1_Send(can);
		}
	}
	break;

	case id_bldc_clear_rotation: // id_bldc_rotation_counter
	{
		if (can->can_dlc == 4)
		{
		}
		else if (can->can_dlc == 8 && can->can_message4_7 == 0)
		{
			//			can->id_combine_10 = 1;
			//			can->can_dlc = 6;
			bldc_control_data.clear_counter = 1;
			//			CAN1_Send(can);
		}
	}
	break;

	case id_adj_select_motor:
	{
		if (can->can_dlc == 4)
		{
			can->id_combine_10 = 1;
			can->can_dlc = 6;
			can->can_message4_5 = adj_motor_control.adj_select_motor;
			CAN1_Send(can);
		}
		else if (can->can_dlc == 6)
		{
			if (can->can_message4_5 == 0)
			{
				adj_motor_control.adj_select_motor = 0;
			}
			else if (can->can_message4_5 == 1)
			{
				adj_motor_control.adj_select_motor = 1;
			}
		}
	}
	break;

	case id_adj_direction_cw:
	{
		if (can->can_dlc == 4)
		{
			can->id_combine_10 = 1;
			can->can_dlc = 6;
			can->can_message4_5 = adj_motor_control.adj_direction_cw;
			CAN1_Send(can);
		}
		else if (can->can_dlc == 6)
		{
			if (can->can_message4_5 == 0)
			{
				adj_motor_control.adj_direction_cw = 0;
			}
			else if (can->can_message4_5 == 1)
			{
				adj_motor_control.adj_direction_cw = 1;
			}
		}
	}
	break;

	case id_adj_direction_ccw:
	{
		if (can->can_dlc == 4)
		{
			can->id_combine_10 = 1;
			can->can_dlc = 6;
			can->can_message4_5 = adj_motor_control.adj_direction_ccw;
			CAN1_Send(can);
		}
		else if (can->can_dlc == 6)
		{
			if (can->can_message4_5 == 0)
			{
				adj_motor_control.adj_direction_ccw = 0;
			}
			else if (can->can_message4_5 == 1)
			{
				adj_motor_control.adj_direction_ccw = 1;
			}
		}
	}
	break;

	case id_adj_ramp_up_time:
	{
		if (can->can_dlc == 4)
		{
			can->id_combine_10 = 1;
			can->can_dlc = 6;
			can->can_message4_5 = adj_motor_control.adj_ramp_up_time;
			CAN1_Send(can);
		}
		else if (can->can_dlc == 6)
		{
			adj_motor_control.adj_ramp_up_time = can->can_message4_5;
		}
	}
	break;

	case id_adj_motor_speed:
	{
		if (can->can_dlc == 4)
		{
			can->id_combine_10 = 1;
			can->can_dlc = 6;
			can->can_message4_5 = adj_motor_control.adj_motor_speed;
			CAN1_Send(can);
		}
		else if (can->can_dlc == 6)
		{
			adj_motor_control.adj_motor_speed = can->can_message4_5;
		}
	}
	break;

	case id_adj_fast_braking:
	{
		if (can->can_dlc == 4)
		{
			can->id_combine_10 = 1;
			can->can_dlc = 6;
			can->can_message4_5 = adj_motor_control.adj_fast_braking;
			CAN1_Send(can);
		}
		else if (can->can_dlc == 6)
		{
			if (can->can_message4_5 == 0)
			{
				adj_motor_control.adj_fast_braking = 0;
				adj_motor_control.adj_fast_braking_flag = 1; // 快速关标志位
				adj_motor_control.adj_direction_cw = 0;
				adj_motor_control.adj_direction_ccw = 0;
			}
			else if (can->can_message4_5 == 1)
			{
				adj_motor_control.adj_fast_braking = 1;
				adj_motor_control.adj_fast_braking_flag = 0;
			}
		}
	}
	break;

	case id_adj_current:
	{
		if (can->can_dlc == 4 && adj_motor_control.adj_select_motor < 2)
		{
			can->id_combine_10 = 1;
			can->can_dlc = 6;
			can->can_message4_5 = adj_motor_control.adj_current[adj_motor_control.adj_select_motor];
			CAN1_Send(can);
		}
	}
	break;

	case id_adj_error:
	{
		if (can->can_dlc == 4 && adj_motor_control.adj_select_motor < 2)
		{
			can->id_combine_10 = 1;
			can->can_dlc = 6;
			can->can_message4_5 = adj_motor_control.adj_error[adj_motor_control.adj_select_motor];
			CAN1_Send(can);
		}
	}
	break;

	case id_sensor1_period_high:
	{
		if (can->can_dlc == 4)
		{
			can->id_combine_10 = 1;
			can->can_dlc = 6;
			can->can_message4_5 = distance_inport2_pwm.tCaptureHigh;
			CAN1_Send(can);
		}
	}
	break;

	case id_sensor1_period_low:
	{
		if (can->can_dlc == 4)
		{
			can->id_combine_10 = 1;
			can->can_dlc = 6;
			can->can_message4_5 = distance_inport2_pwm.tCaptureLow;
			CAN1_Send(can);
		}
	}
	break;

	case id_sensor2_period_high:
	{
		if (can->can_dlc == 4)
		{
			can->id_combine_10 = 1;
			can->can_dlc = 6;
			can->can_message4_5 = distance_inport1_pwm.tCaptureHigh;
			CAN1_Send(can);
		}
	}
	break;

	case id_sensor2_period_low:
	{
		if (can->can_dlc == 4)
		{
			can->id_combine_10 = 1;
			can->can_dlc = 6;
			can->can_message4_5 = distance_inport1_pwm.tCaptureLow;
			CAN1_Send(can);
		}
	}
	break;

	case id_sensor1_state:
	{
		if (can->can_dlc == 4)
		{
			can->id_combine_10 = 1;
			can->can_dlc = 6;
			can->can_message4_5 = distance_inport2_pwm.error_code;
			CAN1_Send(can);
		}
	}
	break;

	case id_sensor2_state:
	{
		if (can->can_dlc == 4)
		{
			can->id_combine_10 = 1;
			can->can_dlc = 6;
			can->can_message4_5 = distance_inport1_pwm.error_code;
			CAN1_Send(can);
		}
	}
	break;

	default:
		break;
	}
#endif
}

/*
 * can消息处理
 */
void can_message_handler(can_data_t *can)
{
	if (can->id_combine_0_3 == 0x07)
	{
		can_id7_message_handler(can);
	}
	else if (can->id_combine_0_3 == 0x00)
	{
		can_id0_message_handler(can);
	}
	else if (can->id_combine_0_3 == 0x01)
	{
		can_id1_message_handler(can);
	}
	else if (can->id_combine_0_3 == 0x0a)
	{
		can_idA_message_handler(can);
	}
	else if (can->id_combine_0_3 == 0x0c)
	{
		can_idC_message_handler(can);
	}
}

/*
 * can接收数据处理
 */
void user_can_RX_handler(void)
{
	//	if(canReceiveDataHandle != NULL)
	{
		if (xQueueReceive(canReceiveDataHandle, RX_DATA.all_data, portMAX_DELAY)) // 请求消息
		{
#if !TEST_CAN_USART
			can_message_handler(&RX_DATA);
#else
			uart3_dma_transmit_buff(RX_DATA.message, RX_DATA.can_dlc);
#endif
		}
	}
}

/*
 * 节点保护检查
 */
void node_guarding_check(void)
{
	if (node_message_value.main_state != 50)
	{
		if (node_guarding_count < 0xffff)
		{
			node_guarding_count++;
		}
	}
}

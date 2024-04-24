/*
 * user_uart3.h
 *
 *  Created on: Jul 29, 2023
 *      Author: xiaoyeiqng
 */

#ifndef INC_USER_UART3_H_
#define INC_USER_UART3_H_

#define USE_NEW_AGREEMENT (1)//使用新串口协议

#define TEST_CAN_USART 0//1开启测试，0关闭测试。利用串口测试can总线，can总线转发串口接收到的数据，串口转发can总线接收到的数据，但每次发送的数据不要超过8个

#define USE_MYSELf_UART3_CONFIG 1//

#define FRAME_HEADER 0x55
#define ESCAPE_CHARACTER 0x54
#define UART3_RX_LEN 200

typedef struct {
	uint8_t Head;   // 取出数据的索引
	uint8_t Tail;   // 存入数据的索引
	uint8_t Lenght; // 待处理数据的长度
	uint8_t Buffer[UART3_RX_LEN];
} uart_rx_data_ty;

#if 0
enum{
	ID_BLDC_select_motor = 0,//选择BLDC电机
	ID_BLDC_direction,//启动方向
	ID_BLDC_ramp_up_time,//加速时间
	ID_BLDC_motor_use,//选择电机 0=grinder(default)  1=pump
	ID_BLDC_set_speed,//设置电机速度
	ID_BLDC_stop,//停止电机 CW/CCW也将复位
	ID_BLDC_phase_current,//读电流
	ID_BLDC_phase_voltage,//读电压
	ID_BLDC_fault,//故障代码
	ID_BLDC_read_speed,//读取当前转速
	ID_BLDC_rotation_counter,//反转计次
	ID_BLDC_temperature,//MCU温度

	ID_BLDC_max_number,//ID数目
};
typedef unsigned char ID_BLDC_index_t;

typedef struct
{
    // data item index number
	ID_BLDC_index_t id;

    // size of the data item
    uint16_t size;

    // pointer to the data item
    volatile void *ptrToData;

    // pointer to the sync counter variable
    volatile uint8_t *pSyncCounter;
} database_item_t;

typedef struct{
	uint16_t select_motor;//选择BLDC电机
	uint16_t direction;//启动方向
	uint16_t ramp_up_time;//加速时间
	uint16_t motor_use;//选择电机 0=grinder(default)  1=pump
	uint16_t set_speed_val;//设置电机速度
	uint16_t stop_val;//停止电机 CW/CCW也将复位
	uint16_t phase_current;//读电流
	uint16_t phase_voltage;//读电压
	uint16_t fault_val;//故障代码
	uint16_t read_speed_val;//读取当前转速
	uint32_t rotation_counter;//反转计次
	uint16_t temperature;//MCU温度
}database_data_t;
#endif

typedef enum {
	VER_QUERY_CMD = 0x81,
	STATE_ISSUED_CMD = 0x82,
	STATE_REPORT_CMD = 0x82,
	BOARD_RST_CMD = 0x80,
}tft_command_id_t;

typedef union{
	struct{
		uint8_t len;//数据长度
		uint8_t command;
		uint8_t select_motor;//选择电机
		uint8_t motor_state;//电机状态
		uint8_t motor_dir;//电机方向
		uint8_t ramp_up_time;//加速时间
		uint16_t target_speed;//电机的目标速度
		uint8_t fast_braking;//快速启动
		uint8_t clear_counter;//清除旋转计数器
		uint16_t CRC16;//CRC16校验值
	};
	uint8_t buff[12];
}bldc_motor_control_issued_t;

typedef union{
	struct{
		uint8_t overcurrent_U :1;//U过流
		uint8_t overcurrent_V :1;//V过流
		uint8_t overcurrent_W :1;//W过流
		uint8_t overvoltage :1;//过压
		uint8_t overtemp :1;//过温
		uint8_t not_hall_signal :1;//无霍尔信号
		uint8_t uart_error :1;//串口通讯异常
	};
	uint8_t val;
}bldc_motor_fault_code_t;

typedef union{
	struct{
		uint8_t select_motor;//选择电机
		uint8_t motor_dir;//电机方向
		uint16_t actual_speed;//电机的实际速度
		uint16_t motor_current;//电机相电流
		uint16_t motor_voltage;//电机相电压
		uint8_t motor_state;//电机状态
		uint8_t prompt_braking;//快速启动
		uint8_t mcu_temperature;//MCU温度
		bldc_motor_fault_code_t fault_code;//故障代码
	};
	uint8_t buff[12];
}bldc_motor_control_answer_t;

extern volatile uart_rx_data_ty rxData;

extern void uart3_rx_handler(void);
extern void user_uart3_init(void);
extern void uart3_dma_transmit_buff(uint8_t* buff, uint16_t len);
extern void uart3_tx_handler(void);


#endif /* INC_USER_UART3_H_ */

/*
 * user_can.h
 *
 *  Created on: Aug 1, 2023
 *      Author: Yeqing.Xiao
 */

#ifndef INC_USER_CAN_H_
#define INC_USER_CAN_H_

#define node_guarding_count_1s (200)
#define node_guarding_count_2s (node_guarding_count_1s * 2)
#define node_guarding_count_30s (node_guarding_count_1s * 30)

enum{
	IOP_k_KTYPE_BROADCAST,
	IOP_k_KTYPE_IO,
	IOP_k_KTYPE_ABR,
	IOP_k_KTYPE_DISP,
	IOP_k_KTYPE_COMM,
	IOP_k_KTYPE_KS,
	IOP_k_KTYPE_MOT,
	IOP_k_KTYPE_ILLUM,
	IOP_k_KTYPE_SENSOR,

};

typedef union{
	struct{

	uint16_t filter_EXID :3;//扩展ID的15-17
	uint16_t filter_IDE :1;//ID类型	0:标准ID	1:扩展ID
	uint16_t filter_RTR :1;//帧类型	0:数据帧	1:远程帧
	uint16_t filter_STID_0_3 :4;
//	uint16_t filter_STID_4_9 :6;
	uint16_t filter_STID_4_7 :4;
	uint16_t filter_STID_8 :1;
	uint16_t filter_STID_9 :1;
	uint16_t filter_STID_10 :1;
	};
	uint16_t val;
}can_16bit_filter_t;

typedef union{
	struct{
	uint32_t filter_UNUSED :1;//未使用
	uint32_t filter_RTR :1;//帧类型	0:数据帧	1:远程帧
	uint32_t filter_IDE :1;//ID类型	0:标准ID	1:扩展ID
	uint32_t filter_EXID :18;//扩展ID的0-17
	uint32_t filter_STID_0_3 :4;
//	uint32_t filter_STID_4_9 :6;
	uint16_t filter_STID_4_7 :4;
	uint16_t filter_STID_8 :1;
	uint16_t filter_STID_9 :1;
	uint32_t filter_STID_10 :1;
	};
	uint32_t val;
}can_32bit_filter_t;

typedef union{
	struct{
		uint16_t id_combine_0_3 :4;
//		uint16_t id_combine_4_9 :6;
		uint16_t id_combine_4_7 :4;
		uint16_t id_combine_8 :1;
		uint16_t id_combine_9 :1;
		uint16_t id_combine_10 :1;
	};
	struct{
		uint16_t can_id :11;
		uint16_t can_IDE :3;//值为CAN_ID_STD 或 CAN_ID_EXT
		uint16_t can_RTR :2;//值为CAN_RTR_DATA 或 CAN_RTR_REMOTE
		uint16_t can_dlc;//数据长度
		union{
			struct{
				uint16_t can_message0_1;
				uint16_t can_message2_3;
				uint16_t can_message4_5;
				uint16_t can_message6_7;
			};
			struct{
				uint32_t can_message0_3;
				uint32_t can_message4_7;
			};
			uint8_t message[8];
		};
	};
	uint8_t all_data[12];
}can_data_t;

typedef union
{
	struct{
		uint8_t main_state;//e.g. RUNN / IDLE / etc.
		uint8_t data1;//default value:0
		uint8_t important;//(tbd, default: 0)
		uint8_t reserved;
		uint8_t SW_H;
		uint8_t SW_M;
		uint8_t SW_L;
		uint8_t HW;//(grinder bldc = 0x14)
	};
	uint8_t all_buff[8];
}node_message_t;

typedef enum{
	/*
	 * BLDC电机指令
	 */
	id_bldc_select_motor		= 1 + (0 * 0x100) + (60 * 0x10000) + (0 * 0x1000000),//选择BLDC电机 00 60 00 01
	id_bldc_direction_cw		= 1 + (0 * 0x100) + (61 * 0x10000) + (0 * 0x1000000),//正转
	id_bldc_direction_ccw		= 1 + (0 * 0x100) + (62 * 0x10000) + (0 * 0x1000000),//反转
	id_bldc_ramp_up_time		= 22 + (0 * 0x100) + (92 * 0x10000) + (0 * 0x1000000),//加速时间
	id_bldc_motor_use			= 0 + (0 * 0x100) + (92 * 0x10000) + (0 * 0x1000000),//选择电机类型
	id_bldc_motor_speed		= 3 + (0 * 0x100) + (92 * 0x10000) + (0 * 0x1000000),//目标转速 RPM
	id_bldc_fast_braking		= 1 + (0 * 0x100) + (63 * 0x10000) + (0 * 0x1000000),//快速启动
	id_bldc_phase_current		= 1 + (0 * 0x100) + (0 * 0x10000) + (0 * 0x1000000),//相电流
	id_bldc_phase_voltage		= 1 + (0 * 0x100) + (3 * 0x10000) + (0 * 0x1000000),//相电压
	id_bldc_error				= 1 + (0 * 0x100) + (7 * 0x10000) + (0 * 0x1000000),//故障代码
	id_bldc_speed				= 1 + (0 * 0x100) + (4 * 0x10000) + (0 * 0x1000000),//实际转速
	id_bldc_rotation_counter	= 1 + (0 * 0x100) + (100 * 0x10000) + (0 * 0x1000000),//旋转计数
	id_bldc_temp_mcu1			= 1 + (0 * 0x100) + (1 * 0x10000) + (0 * 0x1000000),//mcu1温度
	id_bldc_temp_mcu2			= 1 + (0 * 0x100) + (2 * 0x10000) + (0 * 0x1000000),
	id_bldc_clear_rotation		= 1 + (0 * 0x100) + (100 * 0x10000) + (0 * 0x1000000),//旋转计数

	/*
	 * adj电机指令
	 */
	id_adj_select_motor		= 1 + (0 * 0x100) + (64 * 0x10000) + (0 * 0x1000000),
	id_adj_direction_cw		= 1 + (0 * 0x100) + (65 * 0x10000) + (0 * 0x1000000),
	id_adj_direction_ccw		= 1 + (0 * 0x100) + (66 * 0x10000) + (0 * 0x1000000),
	id_adj_ramp_up_time		= 22 + (0 * 0x100) + (93 * 0x10000) + (0 * 0x1000000),
	id_adj_motor_speed			= 3 + (0 * 0x100) + (93 * 0x10000) + (0 * 0x1000000),//目标转速，PWM
	id_adj_fast_braking		= 1 + (0 * 0x100) + (67 * 0x10000) + (0 * 0x1000000),
	id_adj_current				= 1 + (0 * 0x100) + (9 * 0x10000) + (0 * 0x1000000),
	id_adj_error				= 1 + (0 * 0x100) + (8 * 0x10000) + (0 * 0x1000000),

	/*
	 * 距离传感器指令
	 */
	id_sensor1_period_high		= 1 + (0 * 0x100) + (5 * 0x10000) + (0 * 0x1000000),//传感器1高电平计数值
	id_sensor1_period_low		= 1 + (0 * 0x100) + (6 * 0x10000) + (0 * 0x1000000),//传感器1低电平计数值
	id_sensor2_period_high		= 1 + (0 * 0x100) + (12 * 0x10000) + (0 * 0x1000000),//传感器2高电平计数值
	id_sensor2_period_low		= 1 + (0 * 0x100) + (13 * 0x10000) + (0 * 0x1000000),//传感器2低电平计数值
	id_sensor1_state			= 1 + (0 * 0x100) + (10 * 0x10000) + (0 * 0x1000000),//传感器1状态
	id_sensor2_state			= 1 + (0 * 0x100) + (11 * 0x10000) + (0 * 0x1000000),//传感器2状态

	/*
	 * 特殊功能指令 Special Functions
	 */
	id_autorespond_on			= 0 + (0 * 0x100),//data3:0x02 data4:0x80
	id_autorespond_off			= 0 + (0 * 0x100),//data3:0x02 data4:0x00
	id_autorespond_hystersis	= 14 + (0 * 0x100),
	id_periodical_autorespond_time = 18 + (0 * 0x100),

}can_id7_command_t;


extern void user_can_init(void);
extern void user_can_RX_handler(void);
extern void CAN1_Send(can_data_t* can);
extern void user_can_TX_handler(void);
extern void user_data_init(void);
extern void node_guarding_check(void);
extern void user_gpio_init(void);

#endif /* INC_USER_CAN_H_ */

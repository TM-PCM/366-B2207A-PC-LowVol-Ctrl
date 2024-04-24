/*
 * database.h
 *
 *  Created on: Feb 19, 2024
 *      Author: Yeqing.Xiao
 */

#ifndef INC_DATABASE_H_
#define INC_DATABASE_H_

enum
{
	IO_ainBldcPhaseCurrent,
	IO_ainCPU1Temperature,
	IO_ainCPU2Temperature,
	IO_ainBldcPhaseVoltage,
	IO_ainBldcSpeed,//BLDC实际转速(霍尔读取到的转速)
	IO_ainGrinderDistanceSensor1LowTime,
	IO_ainGrinderDistanceSensor1HighTime,
	IO_ainBldcError,
	IO_ainDcMotorError,
	IO_ainDcMotorCurrent,
	IO_ainDistanceSensor1State,
	IO_ainDistanceSensor2State,
	IO_ainGrinderDistanceSensor2LowTime,
	IO_ainGrinderDistanceSensor2HighTime,
	IO_ainBldcError_TBD,
//	IO_ainReserve015,
	IO_ainPumpBldcSpeed,//NEW IO
	IO_ainReserve016,
	IO_ainReserve017,
	IO_ainReserve018,
	IO_ainReserve019,
	IO_ainReserve020,
	IO_ainReserve021,
	IO_ainReserve022,
	IO_ainReserve023,
	IO_dinDistanceSensor1,//暂时应该是没有这条指令
	IO_dinDistanceSensor2,//暂时应该是没有这条指令
	IO_dinBldcIsMoving,
	IO_dinDcMotorIsMoving,
	IO_dinReserve028,
	IO_dinReserve029,
	IO_dinReserve030,
	IO_dinReserve031,
	IO_dinReserve032,
	IO_dinReserve033,
	IO_dinReserve034,
	IO_dinReserve035,
	IO_dinReserve036,
	IO_dinReserve037,
	IO_dinReserve038,
	IO_dinReserve039,
	IO_dinReserve040,
	IO_dinReserve041,
	IO_dinReserve042,
	IO_dinReserve043,
	IO_dinReserve044,
	IO_dinReserve045,
	IO_dinReserve046,
	IO_dinReserve047,
	IO_dinReserve048,
	IO_dinReserve049,
	IO_dinReserve050,
	IO_dinReserve051,
	IO_dinReserve052,
	IO_dinReserve053,
	IO_dinReserve054,
	IO_dinReserve055,
	IO_aoutReserve056,
	IO_aoutReserve057,
	IO_aoutReserve058,
	IO_aoutReserve059,
	IO_doutSelectBldc,
	IO_doutBldcCW,
	IO_doutBldcCCW,
	IO_doutBldcFastBreaking,
	IO_doutSelectDcMotor,
	IO_doutDcMotorCW,
	IO_doutDcMotorCCW,
	IO_doutDcMotorFastBreaking,
//	IO_doutReserve068,
//	IO_doutReserve069,
//	IO_doutReserve070,
	IO_doutPumpBldcCW,//NEW IO
	IO_doutPumpBldcCCW,//NEW IO
	IO_doutPumpBldcFastBreaking,//NEW IO
	IO_doutReserve071,
	IO_doutReserve072,
	IO_doutReserve073,
	IO_doutReserve074,
	IO_doutReserve075,
	IO_doutReserve076,
	IO_doutReserve077,
	IO_doutReserve078,
	IO_doutReserve079,
	IO_doutReserve080,
	IO_doutReserve081,
	IO_doutReserve082,
	IO_doutReserve083,
	IO_doutReserve084,
	IO_doutReserve085,
	IO_doutReserve086,
	IO_doutReserve087,
	IO_doutReserve088,
	IO_doutReserve089,
	IO_doutReserve090,
	IO_doutReserve091,
	IO_pwmBldc, //BLDC目标转速
	IO_pwmDcMotor, //DC电机目标转速
//	IO_pwmReserve094,
	IO_pwmPumpBldc,//NEW IO
	IO_pwmReserve095,
	IO_pwmReserve096,
	IO_pwmReserve097,
	IO_pwmReserve098,
	IO_pwmReserve099,
	IO_cntGrinderMotorTurns,
//	IO_cntReserve101,
	IO_cntPumpMotorTurns,//NEW IO
	IO_cntReserve102,
	IO_cntReserve103,

	IO_MaxNumber,
	IO_SubjoinBldcRampUPTime = IO_MaxNumber,//BLDC加速时间
	IO_SubjoinPumpBldcRampUPTime,//水泵BLDC加速时间

	IO_SubjoinMaxNumber,
	IO_SubjoinNumber = IO_SubjoinMaxNumber - IO_MaxNumber,//附加IO的个数
};
typedef unsigned char database_IO_index_t;

enum{
	SendOnTimeIndex = 0,
	SendOnChangeIndex = 1,

	SendTypeMaxNumber,
};

enum{
	TransferTypeOnTime = 1 << 0,//循环上报
	TransferTypeOnChange = 1 << 1,//值变化后上报
};

typedef union{
	struct{
	uint8_t transfer_type : 2;//开启了哪些上报模式(循环上报、变化上报或两者都开启了)
	uint8_t transfer_state : 1;//是否已在队列中
	uint8_t on_change_delay_flag : 1;//是否变化上报延时
	};
	uint8_t all_data;
}send_attribute_t;

typedef struct{
	uint8_t lenght;
	uint8_t io_queue[IO_MaxNumber + IO_SubjoinNumber];
}send_queue_t;

typedef struct
{
	database_IO_index_t io;
	uint8_t* SendOnChangeFlag;
//	uint16_t* SendOnChangeTargetTime;
//	uint16_t* SendOnChangeCount;
	uint16_t* SendOnTimeTargetTime;
	uint16_t* SendOnTimeCount;
	uint32_t* ptrToData;
}database_item_t;

extern void basedata_queue_handler(void);
extern void updata_data(database_IO_index_t io, uint32_t data);
extern void CAN_transfer_queue_handler(void);
extern void judge_is_reported_message(can_data_t* can);
extern void uart_write_io_data(database_IO_index_t io);
extern void uart_read_io_buff(database_IO_index_t io, uint8_t number);
extern void uart_read_io_data(database_IO_index_t io);
extern void uart_write_io_buff(database_IO_index_t io, uint8_t number);
extern void reset_all_data(void);

#endif /* INC_DATABASE_H_ */

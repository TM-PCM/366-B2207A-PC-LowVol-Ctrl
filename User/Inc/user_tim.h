/*
 * user_tim.h
 *
 *  Created on: Aug 2, 2023
 *      Author: Yeqing.Xiao
 */

#ifndef INC_USER_TIM_H_
#define INC_USER_TIM_H_

typedef struct{
	union{
		struct{
			uint16_t curCaptureVal;//当前的捕获值
			uint16_t round;//定时器溢出中断数
		};
		uint32_t allCurCapture;
	};
	uint32_t tCaptureHigh;//高电平维持时间 = allCurCapture - lastCaptureVal(下降沿捕获时计算)
	uint32_t tCaptureLow;//低电平维持时间 = allCurCapture - lastCaptureVal(上升沿捕获时计算)
	uint16_t lastCaptureVal;//上次的捕获值
	uint8_t flag;
	uint8_t error_code;
}tim_capture_t;


extern void user_tim_init(void);

#endif /* INC_USER_TIM_H_ */

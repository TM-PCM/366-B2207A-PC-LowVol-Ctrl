/*
 * user_adc.h
 *
 *  Created on: Aug 3, 2023
 *      Author: Yeqing.Xiao
 */

#ifndef INC_USER_ADC_H_
#define INC_USER_ADC_H_

#define ADC_MAX_CHANNEL 3

extern void user_adc_init(void);
extern void adc_buff_handler(void);
extern void adc_start(void);
extern int32_t get_interior_temperature(void);

#endif /* INC_USER_ADC_H_ */

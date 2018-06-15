/*
 * adc.h
 *
 *  Created on: Jun 14, 2018
 *      Author: ugopalatucci
 */

#ifndef ADC_H_
#define ADC_H_
#include "stm32f4xx_hal.h"
#include "common.h"
#include "usart.h"

extern ADC_HandleTypeDef hadc1;


void MX_ADC1_Init(void);

#endif /* ADC_H_ */

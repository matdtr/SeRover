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
extern DMA_HandleTypeDef hdma_adc1;
extern uint32_t ADC_BUF[3];

void MX_ADC1_Init(void);

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

#endif /* ADC_H_ */

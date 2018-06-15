/*
 * adc.c
 *
 *  Created on: Jun 14, 2018
 *      Author: ugopalatucci
 */

#include <line_handler.h>

uint32_t ADC_BUF[3];

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

}

void read_line(t_motorcommand* cmd){
	char msg[30];

	if (ADC_BUF[LEFT_DET] > LINE_DET_LIM && ADC_BUF[CENTER_DET] < LINE_DET_LIM && ADC_BUF[RIGHT_DET] > LINE_DET_LIM){
		cmd->command = 8;
	}else if (ADC_BUF[LEFT_DET] < LINE_DET_LIM && ADC_BUF[CENTER_DET] < LINE_DET_LIM && ADC_BUF[RIGHT_DET] > LINE_DET_LIM){
		cmd->command = 11;
	}else if (ADC_BUF[LEFT_DET] > LINE_DET_LIM && ADC_BUF[CENTER_DET] < LINE_DET_LIM && ADC_BUF[RIGHT_DET] > LINE_DET_LIM){
		cmd->command = 10;
	}
	send_command_motor(&huart6,cmd->command,cmd->value);

	sprintf(msg, "%d %d %d \n\r", ADC_BUF[LEFT_DET], ADC_BUF[CENTER_DET], ADC_BUF[RIGHT_DET]);
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 0xFFFF);

}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

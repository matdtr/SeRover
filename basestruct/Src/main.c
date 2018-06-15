/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <adc.h>
#include <adc.h>
#include "main.h"
#include "stm32f4xx_hal.h"
#include "dma.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdlib.h>
/* USER CODE BEGIN Includes */
#include "common.h"
#include "sonar_handler.h"
#include "ws2812_handler.h"
#include "motor_handler.h"
#include "autonomus_handler.h"
/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
char readBuf[7]; // 11
__IO ITStatus UartReady = SET;
int autonoma = 0;
uint32_t ADC_BUF[3];
uint32_t ADC_DATA[3];
uint16_t motor_speed = 0;
char dataz[20];
uint16_t front_sonar = 0;
uint16_t rear_sonar = 0;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int read_ble(char* c){
	//uint8_t retVal = -1;
	//strcpy(c,"b");
	 if(UartReady == SET) {
	    UartReady = RESET;
	    HAL_UART_Receive_IT(&huart1, (uint8_t*)readBuf, 7);
	    strcpy(c,readBuf);
	    HAL_UART_Transmit(&huart2, (uint8_t*)c, strlen(c),0xFFFFFF);
	    return 1;
	 } else {
		 return 0;
	 }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init(); // non utilizzato al momento
  MX_TIM11_Init();

  MX_USART6_UART_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  char msg[10] = "Start\n\r";
  HAL_UART_Transmit(&huart2, (uint8_t*)msg,strlen(msg), 0xFFFFFF);


  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);

  MX_I2C1_Init();

  HAL_UART_Transmit(&huart2, (uint8_t*)msg,strlen(msg), 0xFFFFFF);

  MX_ADC1_Init();
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_BUF, 3);

  HAL_UART_Transmit(&huart2, (uint8_t*)msg,strlen(msg), 0xFFFFFF);

  HAL_ADC_Start_IT(&hadc1);



  /* USER CODE BEGIN 2 */
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);

  HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
  HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 0, 0);

  HAL_TIM_Base_Init(&htim11);
  //HAL_TIM_Base_Start_IT(&htim11);


  char data3[100];
  char rangestring[10] ;
  uint16_t speed_command = 0;
  uint16_t new_speed_command = 0;
  uint16_t cnt1 = 0;
  uint16_t cnt2 = 0;
  uint16_t speed_d = 0;
  uint16_t range_sonar1 = 0;
  uint16_t range_sonar2 = 0;
  char c[11];
  int brightness = 0;
  int i = 0;

  t_motorcommand cmd;

  // ---- Motor Init -------
  motor_Init(&huart6);
  stop_motors(&huart6);

  // ---- LED Init -------
  ws2812_init_leds();

  uint32_t tick = HAL_GetTick();

  // ---- Sonar Init -------

  sonar_Init(&hi2c1, FRONT_SONAR_ADDR, (uint8_t)2);
  sonar_Init(&hi2c1, REAR_SONAR_ADDR, (uint8_t)2);

  HAL_UART_Transmit(&huart2, (uint8_t*)msg,strlen(msg), 0xFFFFFF);
  while (1) {
		i = read_ble(c);

		if (i == 1) {
			/*------ Parse del comando ---- */
			parse_command(c,&cmd);

			sprintf(data3, "TEST STRUCT %d --- %d ",cmd.command, cmd.value);
			HAL_UART_Transmit(&huart2, (uint8_t*) data3, strlen(data3),0xFFFFFF);

			switch (cmd.command){
			case 100:
				// automode start
				autonoma = 1;
				cmd.value = AUTOMODE_SPEED;
				cmd.command = 8;
				speed_d = ((AUTOMODE_SPEED * 9)/2);
				send_command_motor(&huart6,cmd.command,cmd.value);
				goto autonoma;
				break;
			case 101:
				//automode stop
				autonoma = 0;
				reset_commands(&cmd);
				break;
			case 102:
				// get info
				get_sensors_info(&huart1, motor_speed, brightness, range_sonar1, range_sonar2, ADC_DATA[LEFT_DET], ADC_DATA[CENTER_DET], ADC_DATA[RIGHT_DET]);
				break;
			case 99:
				// setup led
				goto led_setup;
				break;
			case 0:
				stop_motors(&huart6);
				speed_d = ((cmd.value * 9) / 2);
				break;
			default:
				send_command_motor(&huart6,cmd.command,cmd.value);
				speed_d = ((cmd.value * 9) / 2);

			}
		}

		/* Comando per cambiare la luminosità della matrice del led */

		led_setup:
		if((cmd.command == 99) && (cmd.value >1)){
			ws2812_set_color_matrix(cmd.value, cmd.value, cmd.value);
			brightness = cmd.value;
		}
		if ((cmd.command == 99) && (cmd.value == 0 )) {
			ws2812_set_color_matrix(0, 0, 0);
			brightness = cmd.value;
			HAL_Delay(100);
		}

		autonoma:
		/* Leggi i sonar per la guida autonoma */
		if (autonoma == 1){
			/* range_sonar1 = read_range(&hi2c1,FRONT_SONAR_ADDR);
			sprintf(rangestring, "Range: %lu \n", range_sonar1);
			HAL_UART_Transmit(&huart2, (uint8_t*) rangestring, strlen(rangestring), 0xFFFF);

			/* Se la distanza è minore di 20, fermati e aspetta un nuovo comando
			if (range_sonar1 < 20){
				reset_commands(&forward, &reverse, &right, &left, &speed_command);
				stop_motors(&huart6);
			}*/

			read_line(&cmd);
			// TODO fai qualcosa
		}

		/* ------ ENCODER --------- */
		if (HAL_GetTick() - tick > 100L) {
			new_speed_command = motor_encoder(&htim3,&htim4, &huart2, &cnt1,&cnt2,speed_d, cmd.value, &motor_speed);

			if (new_speed_command != cmd.value) {
				send_command_motor(&huart6,cmd.command,new_speed_command);
			}
			cmd.value = new_speed_command;
			new_speed_command = 0;
			cnt1 = __HAL_TIM_GET_COUNTER(&htim3);
			cnt2 = __HAL_TIM_GET_COUNTER(&htim4);
			tick = HAL_GetTick();
		}
		/* ----- FINE ENCODER ------  */
	}

}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
 /* Set transmission flag: transfer complete*/
 UartReady = SET;
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	//TODO callback timer11 ogni 0.5s devo leggere i valori del sonar.

	front_sonar = read_range(&hi2c1,FRONT_SONAR_ADDR);
	rear_sonar = read_range(&hi2c1,REAR_SONAR_ADDR);

	if((front_sonar < MIN_DISTANCE) || (rear_sonar < MIN_DISTANCE)){
		stop_motors(&huart6);
	}

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	char msg[30];

	ADC_DATA[0] = ADC_BUF[0];
	ADC_DATA[1] = ADC_BUF[1];
	ADC_DATA[2] = ADC_BUF[2];

}

void read_line(t_motorcommand* cmd){
	char msg[30] = "CIao";
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 0xFFFF);

	if (ADC_DATA[LEFT_DET] > LINE_DET_LIM && ADC_DATA[CENTER_DET] < LINE_DET_LIM && ADC_DATA[RIGHT_DET] > LINE_DET_LIM){
		cmd->command = 8;
	}else if (ADC_DATA[LEFT_DET] < LINE_DET_LIM && ADC_DATA[CENTER_DET] < LINE_DET_LIM && ADC_DATA[RIGHT_DET] > LINE_DET_LIM){
		cmd->command = 11;
	}else if (ADC_DATA[LEFT_DET] > LINE_DET_LIM && ADC_DATA[CENTER_DET] < LINE_DET_LIM && ADC_DATA[RIGHT_DET] > LINE_DET_LIM){
		cmd->command = 10;
	}
	send_command_motor(&huart6,cmd->command,cmd->value);

	sprintf(msg, "%d %d %d \n\r", ADC_DATA[LEFT_DET], ADC_DATA[CENTER_DET], ADC_DATA[RIGHT_DET]);
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 0xFFFF);

}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

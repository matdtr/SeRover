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
#include "main.h"
#include "stm32f4xx_hal.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include <string.h>
#include <stdlib.h>
/* USER CODE BEGIN Includes */
#include "common.h"
#include "sonar_handler.h"
#include "lane_handler.h"
#include "ws2812_handler.h"
#include "motor_handler.h"

/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
char readBuf[11];
__IO ITStatus UartReady = SET;

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
	    HAL_UART_Receive_IT(&huart1, (uint8_t*)readBuf, 11);
	    strcpy(c,readBuf);
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
  MX_TIM5_Init();
  MX_USART6_UART_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_3);

  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  char* data = "START ";
  char* data2= " RECEIVED ";
  char* data3[100];
  char* data4= " YES ";
  uint16_t speed_command = 0;
  uint16_t new_speed = 0;
  char c[11];


  /* USER CODE END 2 */
  int forward = 0;
    int reverse = 0 ;
    int right = 0;
    int left = 0;
    int bright= 1;
    int i = 0;
    motor_Init(&huart6);
    stop_motors(&huart6);
	ws2812_init_leds();

	uint16_t tick = HAL_GetTick();
	uint16_t counter = 0;
	drive_forward(&huart6, 1);
	HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data), 0xFFFFFF);
	uint8_t motor_command = 0;

    while (1)
    {
	   /* HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data), 0xFFFFFF);
	    HAL_UART_Receive(&huart1, (uint8_t*)readBuf,11,0xFFFFFF);
		sprintf(data3, "%s", readBuf);
	    HAL_UART_Transmit(&huart2, (uint8_t*)data3, strlen(data3), 0xFFFFFF);
	    */
	  	i = read_ble(c);

	  	if (i ==1){

			char* command = strtok(c, "#");
			forward = atoi(command);
			command = strtok(0, "#");
			reverse = atoi(command);
			command = strtok(0, "#");
			left = atoi(command);
			command = strtok(0, "#");
			right = atoi(command);
			command = strtok(0, "#");
			bright = atoi(command);

			sprintf(data3, "F: %d , RV: %d , RX: %d , SX: %d , BR: %d \n", forward,reverse,right,left,bright);
			HAL_UART_Transmit(&huart2, (uint8_t*)data3, strlen(data3), 0xFFFFFF);
			/* TODO: testareil codice per uso dei motori */

			/*if (forward == reverse == right == left == 0) {
				stop_motors(&huart6);
			}else{*/
				drive_forward(&huart6, forward);
				drive_backwards(&huart6, reverse);
				turn_right(&huart6, right);
				turn_left(&huart6, left);
			/*}

			if (forward > 0) {
				//avanti
				HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data), 0xFFFFFF);

				drive_forward(&huart6, forward);
				speed = forward;

			}
			if (reverse > 0) {
				// indietro
				drive_backwards(&huart6, reverse);

			}
			if (right > 0) {
				//destra
				turn_right(&huart6, right);

			}
			if (left > 0) {
				//sinistra
				turn_left(&huart6, left);
			}*/
			if (forward > 0 ){
				speed_command = forward;

			}else if (reverse > 0 ){
				speed_command = reverse;
			}else if (right > 0 ){
				speed_command = right;
			}else if (left > 0 ){
				speed_command = left;
			}else{
				speed_command = 0;
			}

			if((forward==11) && (reverse == 11) && (left == 1) && (right == 1)){
				/* TODO: autonomus mode func  */
			}

	  	}
		if (bright > 1) {
			/* TODO: testare accensione led */
			ws2812_set_color_matrix(bright, bright, bright);
		}
		if (bright == 0) {
			/* TODO: testare spegnimento led */
			ws2812_set_color_matrix(0, 0, 0);
			HAL_Delay(100);
		}

		if (HAL_GetTick() - tick > 50L){
			new_speed = motor_encoder(&htim4, &huart2, &counter, speed_command, new_speed);
			if (forward > 0) {
				//avanti
				drive_forward(&huart6, new_speed);
			}
			if (reverse > 0) {
				// indietro
				drive_backwards(&huart6, new_speed);
			}
			if (right > 0) {
				//destra
				turn_right(&huart6, new_speed);
			}
			if (left > 0) {
				//sinistra
				turn_left(&huart6, new_speed);
			}
			tick = HAL_GetTick();
		}
    }

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
 /* Set transmission flag: transfer complete*/
 UartReady = SET;
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

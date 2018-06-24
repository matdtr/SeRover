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
/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
char readBuf[7]; // 11
__IO ITStatus UartReady = SET;
int autonoma = 0;
uint32_t ADC_BUF[3];
uint16_t motor_speed = 0;
char dataz[20];
uint16_t front_sonar = 0;
uint16_t rear_sonar = 0;

uint16_t speed1 = AUTOMODE_SPEED;
uint16_t speed2 = AUTOMODE_SPEED;
double error_pre_speed_1 = 0;
double error_pre_speed_2 = 0;
double pid_i_pre1 = 0;
double pid_i_pre2 = 0;
//double kr = 0.65;
//double kf = 0.9;

double kr= 0.65;
double kf= 0.95;


int true = 1;
int stop_sonar = 0;
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


  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);

  MX_I2C1_Init();

  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);

  HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
  HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 0, 0);

  HAL_TIM_Base_Init(&htim11);
  HAL_TIM_Base_Start_IT(&htim11);


  char data3[100];
  uint16_t cnt1 = 0;
  uint16_t cnt2 = 0;
  char c[11];
  int brightness = 0;
  int red = 0;
  int green = 0;
  int blue = 0;
  int i = 0;
  int go = 0;
  int am = 0;
  t_motorcommand cmd;
  uint16_t new_speed1 = 0;
  uint16_t new_speed2 = 0;


  // ---- Motor Init -------
  motor_Init(&huart6);
  stop_motors(&huart6);

  // ---- LED Init -------
  ws2812_init_leds();


  // ---- Sonar Init -------

  sonar_Init(&hi2c1, FRONT_SONAR_ADDR, (uint8_t)2);
  HAL_Delay(100);
  sonar_Init(&hi2c1, REAR_SONAR_ADDR, (uint8_t)2);
  HAL_Delay(100);

  uint32_t tick = HAL_GetTick();
  while (1) {

		i = read_ble(c);


		if (i == 1) {
			/*------ Parse del comando ---- */
			parse_command(c,&cmd);

			sprintf(data3, "TEST STRUCT %d --- %d \n\r",cmd.command, cmd.value);
			HAL_UART_Transmit(&huart2, (uint8_t*) data3, strlen(data3),0xFFFFFF);

			switch (cmd.command){
			case 100:
				// automode start
				HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_BUF, 3);
				HAL_ADC_Start_IT(&hadc1);
				autonoma = 1;
				cmd.command = 8;
				cmd.value = AUTOMODE_SPEED;
				controls_from_command(cmd.command, cmd.value, cmd.value);
				speed1 = AUTOMODE_SPEED;
				speed2 = speed1;
				//ws2812_auto_mode();
				break;
			case 101:
				//automode stop
				HAL_ADC_Stop_DMA(&hadc1);
				HAL_ADC_Stop_IT(&hadc1);
				autonoma = 0;
				reset_commands(&cmd);
				speed1 = 0;
				speed2 = 0;
				stop_motors(&huart6);
				am = 3;
				break;
			case 102:
				// get info
				get_sensors_info(&huart1, motor_speed, brightness, front_sonar, rear_sonar, ADC_BUF[LEFT_DET], ADC_BUF[CENTER_DET], ADC_BUF[RIGHT_DET]);
				break;
			case 99:
				// setup led
				goto led_setup;
				break;
			case 0:
				stop_motors(&huart6);
				speed1 = cmd.value;
				speed2 = speed1;
				break;
			case 51:
				red = cmd.value;
				break;
			case 52:
				green = cmd.value;
				break;
			case 53:
				blue = cmd.value;
				cmd.value = 0;
				goto easterEgg;
				break;
			default:
				speed1 = cmd.value;
				speed2 = speed1;
				controls_from_command(cmd.command, speed1, speed2);
			}
			//reset_pid_variabiles();
		}
		/* Comando per cambiare la luminosità della matrice del led */
		easterEgg:
		if(cmd.command == 53){
			ws2812_set_color_matrix(red, green, blue);
			HAL_Delay(100);
		}

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

		/* Leggi i sonar per la guida autonoma */
		if (autonoma == 1){


		  if(true == 1 && stop_sonar == 0){
				front_sonar = read_range(&hi2c1,FRONT_SONAR_ADDR);
				stop_sonar = 1;
				sprintf(data3, "sonar 1 %d \n\r", front_sonar);
				HAL_UART_Transmit(&huart2, (uint8_t*) data3, strlen(data3),0xFFFFFF);
			}else if(true == 2 && stop_sonar == 0){
				rear_sonar = read_range(&hi2c1,REAR_SONAR_ADDR);
				stop_sonar = 1;
				true = 0 ;
				sprintf(data3, "Sonar 2 %d \n\r", rear_sonar);
				HAL_UART_Transmit(&huart2, (uint8_t*) data3, strlen(data3),0xFFFFFF);

			}

			if( (front_sonar > MIN_DISTANCE || front_sonar == 0) && (rear_sonar > MIN_DISTANCE || rear_sonar == 0) ){
				if((front_sonar != 0) || (go != 0) || (rear_sonar != 0)){
					speed1 = AUTOMODE_SPEED;
					speed2= AUTOMODE_SPEED;
					read_line(&cmd);
					go = 1;
				}
			}else{
				if((front_sonar != 0) || (go != 1) || (rear_sonar != 0)){
					stop_motors(&huart6);
					speed1 = 0;
					speed2 = 0;
					go = 0;
				}

			}

			read_line(&cmd);



		}

		/* ------ ENCODER --------- */
		 if ((HAL_GetTick() - tick > 100L)) {

			uint32_t diff = HAL_GetTick()-tick;
			new_speed1 = motor_encoder(&htim4, &cnt1, speed1, new_speed1, &motor_speed, &error_pre_speed_1, &pid_i_pre1, &cmd, diff);
			new_speed2 = motor_encoder(&htim3, &cnt2, speed2, new_speed2, &motor_speed, &error_pre_speed_2, &pid_i_pre2, &cmd,  diff);

			controls_from_command(cmd.command, new_speed1, new_speed2);

			cnt1 = __HAL_TIM_GET_COUNTER(&htim4);
			cnt2 = __HAL_TIM_GET_COUNTER(&htim3);


			tick = HAL_GetTick();
		}

		/* ----- FINE ENCODER ------  */
	}

}

void controls_from_command(int command, uint16_t new_speed1, uint16_t new_speed2){
	switch (command){
		case 9:
			drive_backwards(&huart6, new_speed2, new_speed1);
			break;
		case 8:
			drive_forward(&huart6, new_speed2, new_speed1);
			break;
		case 10:
			turn_right(&huart6, new_speed2, new_speed1);
			break;
		case 11:
			turn_left(&huart6, new_speed2, new_speed1);
			break;
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
 /* Set transmission flag: transfer complete*/
 UartReady = SET;
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	//TODO callback timer11 ogni 0.5s devo leggere i valori del sonar.
	true++;

	stop_sonar = 0;

	if(true > 2)
		true = 1;
}


void read_line(t_motorcommand* cmd){
	uint16_t tmp = cmd->command;
	char msg[30];
	int pippo = 0;
	if ((ADC_BUF[LEFT_DET] > LINE_DET_LIM) && (ADC_BUF[CENTER_DET] < LINE_DET_LIM) && (ADC_BUF[RIGHT_DET] > LINE_DET_LIM)){
		speed1 = AUTOMODE_SPEED;
		speed2 = speed1;
		cmd->command = 8;
		pippo = 1;

	}else if ((ADC_BUF[LEFT_DET] < LINE_DET_LIM) && (ADC_BUF[CENTER_DET] < LINE_DET_LIM) && (ADC_BUF[RIGHT_DET] > LINE_DET_LIM)){
		speed2 = (kf*AUTOMODE_SPEED);
		speed1 = kr*AUTOMODE_SPEED;
		cmd->command = 11; //giro a sx
		pippo = 2;

	} else if ((ADC_BUF[LEFT_DET] > LINE_DET_LIM) && (ADC_BUF[CENTER_DET] < LINE_DET_LIM) && (ADC_BUF[RIGHT_DET] < LINE_DET_LIM)){
		speed1 = (kf*AUTOMODE_SPEED);
		speed2 = kr*AUTOMODE_SPEED;
		cmd->command = 10; // giro a dx
		pippo = 3;

	}else if ((ADC_BUF[LEFT_DET] > LINE_DET_LIM) && (ADC_BUF[CENTER_DET] > LINE_DET_LIM) && (ADC_BUF[RIGHT_DET] < LINE_DET_LIM)){
		speed1 = (kf*AUTOMODE_SPEED)+1;
		speed2 = kr*AUTOMODE_SPEED;
		cmd->command = 10; // giro a dx essendo il centrale e quello a dx nostra bassi
		pippo = 4;

	}else if((ADC_BUF[LEFT_DET] < LINE_DET_LIM) && (ADC_BUF[CENTER_DET] > LINE_DET_LIM) && (ADC_BUF[RIGHT_DET] > LINE_DET_LIM)){
		speed2 = (kf*AUTOMODE_SPEED)+1;
		speed1 = (kr*AUTOMODE_SPEED);
		cmd->command = 11; //giro a sx essendo il centrale e quello a sx nostra basso
		pippo = 5;
	}

	if (tmp != cmd->command){
		if(cmd->command == 8 && (tmp == 10 || tmp == 11)){
			speed1 = 0.8 * AUTOMODE_SPEED;
			speed2 = speed1;
		}
		sprintf(msg, "IF: %d \n\r",pippo);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),0xFFFFFF);
		controls_from_command(cmd->command, speed1, speed2);
	}



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

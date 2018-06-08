/*
 * motor_handler.c
 *
 *  Created on: 06 giu 2018
 *      Author: mattiaditrolio
 */

#include "motor_handler.h"


void send_command_motor(UART_HandleTypeDef* huart,char command,char speed){
	char address = H_BRIDGE_ADDR;

	char checksum = (address + command + speed) & 0b01111111;

	HAL_UART_Transmit(huart, (uint8_t*)&address, sizeof(address), 0xFFFFFF);
	HAL_UART_Transmit(huart, (uint8_t*)&command, sizeof(command), 0xFFFFFF);
	HAL_UART_Transmit(huart, (uint8_t*)&speed, sizeof(speed), 0xFFFFFF);
	HAL_UART_Transmit(huart, (uint8_t*)&checksum, sizeof(checksum), 0xFFFFFF);


}

void drive_forward(UART_HandleTypeDef* huart, char speed){
	char command = 8;
	send_command_motor(huart,command,speed);
}

void drive_backwards(UART_HandleTypeDef* huart, char speed){
	char command = 9;
	send_command_motor(huart,command,speed);
}

void turn_right(UART_HandleTypeDef* huart, char speed){
	char command = 10;
	send_command_motor(huart,command,speed);
}

void turn_left(UART_HandleTypeDef* huart, char speed){
	char command = 11;
	send_command_motor(huart,command,speed);
}

void stop_motors(UART_HandleTypeDef* huart){
	turn_left(huart, 0);
	turn_right(huart,0);
	drive_backwards(huart, 0);
	drive_forward(huart, 0);
}

void motor_Init(UART_HandleTypeDef* huart)
{
	char address = H_BRIDGE_ADDR;
	char command = 11;
	char speed = 0;
	char checksum = (address + command + speed) & 0b01111111;
	HAL_UART_Transmit(huart, (uint8_t*)&address, sizeof(address), 0xFFFFFF);
	HAL_UART_Transmit(huart, (uint8_t*)&command, sizeof(command), 0xFFFFFF);
	HAL_UART_Transmit(huart, (uint8_t*)&speed, sizeof(speed), 0xFFFFFF);
	HAL_UART_Transmit(huart, (uint8_t*)&checksum, sizeof(checksum), 0xFFFFFF);

	HAL_Delay(1000);

}

void motor_encoder(TIM_HandleTypeDef* htim, UART_HandleTypeDef* huart, uint16_t* counter, uint16_t* speed_command){
	uint16_t cnt2 = 0;
	uint16_t diff = 0;
	uint16_t speed = 0;
	char msg[80];

	cnt2 = __HAL_TIM_GET_COUNTER(htim);
	if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) {
		if (cnt2 < *counter)
			diff = *counter - cnt2;
		else if(cnt2 == *counter)
			diff = 0;
		else
			diff = (65535 - cnt2) + *counter;
	} else {
		if (cnt2 > *counter)
			diff = cnt2 - *counter;
		else if(cnt2 == *counter)
			diff = 0;
		else
			diff = (65535 - *counter) + cnt2;
	}

	sprintf(msg, "Difference: %d\r\n", diff);
	HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg),0xFFFF);

	sprintf(msg, "CNT1: %d\r\n", *counter);
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 0xFFFF);
	sprintf(msg, "CNT2: %d\r\n", cnt2);
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 0xFFFF);

	speed = ((diff / 4) / 3);

	if ((TIM1->SMCR & 0x3) == 0x3) {
		speed /= 2;
	}


	sprintf(msg, "Speed: %d\r\n", speed);
	HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg),0xFFFF);

	*counter = __HAL_TIM_GET_COUNTER(htim);

	if (*speed_command+(*speed_command - speed)> 127){
			*speed_command = 127;
		}
	else
		*speed_command = *speed_command + (*speed_command - speed);
	sprintf(msg, "Speed Command: %d\r\n", *speed_command);

	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 0xFFFF);

	/*if (*speed_command > 127){
		*speed_command = 127;
	}*/
}

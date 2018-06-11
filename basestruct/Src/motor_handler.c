/*
 * motor_handler.c
 *
 *  Created on: 06 giu 2018
 *      Author: mattiaditrolio
 */

#include "motor_handler.h"
#include "math.h"

void send_command_motor(UART_HandleTypeDef* huart,char command,char speed){
	char address = H_BRIDGE_ADDR;

	char checksum = (address + command + speed) & 0b01111111;

	HAL_UART_Transmit(huart, (uint8_t*)&address, sizeof(address), 0xFFFFFF);
	HAL_UART_Transmit(huart, (uint8_t*)&command, sizeof(command), 0xFFFFFF);
	HAL_UART_Transmit(huart, (uint8_t*)&speed, sizeof(speed), 0xFFFFFF);
	HAL_UART_Transmit(huart, (uint8_t*)&checksum, sizeof(checksum), 0xFFFFFF);
	HAL_Delay(50);


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
	HAL_Delay(50);
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

uint16_t motor_encoder(TIM_HandleTypeDef* htim, UART_HandleTypeDef* huart, uint16_t* counter,uint16_t speed_d, uint16_t speed_command){
	uint16_t cnt2 = 0;
	uint16_t diff = 0;
	uint32_t speed = 0;
	uint16_t errore = 0;
	uint16_t tmp = 0;
	tmp = speed_command;
	char msg[80];
	cnt2 = __HAL_TIM_GET_COUNTER(htim);
	if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) {
		if (cnt2 < *counter)
			diff = *counter - cnt2;
		else
			diff = (65535 - cnt2) + *counter;
	} else {
		if (cnt2 > *counter)
			diff = cnt2 - *counter;
		else
			diff = (65535 - *counter) + cnt2;
	}

	speed = (((diff * 60)/ (64*19)));
	if (speed == 3233){
		speed = 0;
	}

	sprintf(msg, "Speed: %d\r\n", speed);
	HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg),0xFFFF);

	sprintf(msg, "Speed D: %d\r\n", speed_d);
	HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg),0xFFFF);

	errore = abs(speed_d - speed);

	sprintf(msg, "Errore: %d\r\n", errore);
	HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg),0xFFFF);

	if ((errore >= 0) && (errore< 15)){
		return speed_command;
	}else if((speed_d < speed) && (speed_d <= 250)){
		speed_command = speed_command - (ceil((errore*2)/9));
	}else if(speed_d > speed && (speed_d > 250)){
		speed_command = speed_command + ceil((ceil(((errore*2)/9)) /10));
	}else if((speed_d < speed) && (speed_d > 250)){
		speed_command = speed_command - (ceil((errore*2)/9));
	}else if(speed_d > speed && (speed_d < 250)){
		speed_command = speed_command + (ceil((errore*2)/9));
	}
	if (speed_command > 101){
		speed_command = 100;
	}

	errore = 0;
	if ((speed_command >= 6 + tmp) && speed_d >= 250 ){
		speed_command = tmp+ 2;
	}
	return speed_command;

}

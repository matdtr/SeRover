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
	turn_left(huart, 0);
	turn_right(huart,0);
	char command = 8;
	send_command_motor(huart,command,speed);
}

void drive_backwards(UART_HandleTypeDef* huart, char speed){
	turn_left(huart, 0);
	turn_right(huart,0);
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

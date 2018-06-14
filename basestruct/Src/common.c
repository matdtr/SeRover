/*
 * common.c
 *
 *  Created on: 06 giu 2018
 *      Author: mattiaditrolio
 */

#include "common.h"

void reset_commands(int* forward, int* reverse, int* right, int* left, uint16_t* speed_command){
	*forward = 0;
	*right = 0;
	*left = 0;
	*reverse = 0;
	*speed_command = 0;
}


void parse_command(char* c, int* forward, int* reverse, int* right, int* left, int* bright){
	char* command = strtok(c, "#");
	*forward = atoi(command);
	command = strtok(0, "#");
	*reverse = atoi(command);
	command = strtok(0, "#");
	*left = atoi(command);
	command = strtok(0, "#");
	*right = atoi(command);
	command = strtok(0, "#");
	if (command != NULL){
		*bright = atoi(command);
	}
}

void get_sensors_info(UART_HandleTypeDef* huart, uint16_t motor_speed, int bright, uint16_t range_sonar1, uint16_t range_sonar2, uint8_t line1, uint8_t line2, uint8_t line3){
	char data [30];
	sprintf(data, "Motor speed: %u\n", motor_speed);
	HAL_UART_Transmit(huart, (uint8_t*) &data, strlen(&data),0xFFFFFF);
	sprintf(data, "Brightness: %d\n", bright);
	HAL_UART_Transmit(huart, (uint8_t*) &data, strlen(&data),0xFFFFFF);
	sprintf(data, "Range Front: %u\n", range_sonar1);
	HAL_UART_Transmit(huart, (uint8_t*) &data, strlen(&data),0xFFFFFF);
	sprintf(data, "Range Retro: %u\n", range_sonar2);
	HAL_UART_Transmit(huart, (uint8_t*) &data, strlen(&data),0xFFFFFF);
	sprintf(data, "Line1: %u\n", line1);
	HAL_UART_Transmit(huart, (uint8_t*) &data, strlen(&data),0xFFFFFF);
	sprintf(data, "Line2: %u\n", line2);
	HAL_UART_Transmit(huart, (uint8_t*) &data, strlen(&data),0xFFFFFF);
	sprintf(data, "Line3: %u\n", line3);
	HAL_UART_Transmit(huart, (uint8_t*) &data, strlen(&data),0xFFFFFF);
}

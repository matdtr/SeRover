/*
 * common.c
 *
 *  Created on: 06 giu 2018
 *      Author: mattiaditrolio
 */

#include "common.h"


void reset_commands(t_motorcommand* motorcommand){
	motorcommand->command = 0;
	motorcommand->value = 0;
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

void parse_command(char* c, t_motorcommand* motorcommand){
	char* string = strtok(c, "#");
	motorcommand->command = atoi(string);
	string = strtok(0, "#");
	motorcommand->value = atoi(string);
}



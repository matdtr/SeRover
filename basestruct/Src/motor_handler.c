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

uint16_t motor_encoder(TIM_HandleTypeDef* htim, UART_HandleTypeDef* huart, uint16_t* counter, uint16_t speed_command, uint16_t actual_speed){
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

	speed = ((diff / 4) / 3);

	if ((TIM1->SMCR & 0x3) == 0x3) {
		speed /= 2;
	}


	sprintf(msg, "Speed: %d\r\n", speed);
	HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg),0xFFFF);

	*counter = __HAL_TIM_GET_COUNTER(htim);

	// Proporzione 127: 84 = Velocità_desiderata : Velcoità_calcolata
	// 127 massimo comando e 84 massima velocità
	// Velocità convertita in comandi dagli encoder
	int encoder_speed = ceil((float)speed*1.5);

	sprintf(msg, "Encoder Speed: %d\r\n", encoder_speed);

	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 0xFFFF);
	// 84 è la massima velocità.
	if (encoder_speed > 127){
		encoder_speed = 127;
	}
	if (encoder_speed > speed_command){
		actual_speed = actual_speed - (encoder_speed - speed_command);
		//actual_speed = speed_command - 1;
		if (actual_speed > 127)
			actual_speed = 0;
	}else if (encoder_speed < speed_command){
		actual_speed = actual_speed + (speed_command - encoder_speed);
		//actual_speed = speed_command + 10;
		if (actual_speed > 127)
			actual_speed = 127;
	}


	sprintf(msg, "Speed Command: %d\r\n", actual_speed);

	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 0xFFFF);
	return actual_speed;
}

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


void drive_forward(UART_HandleTypeDef* huart, char speed1, char speed2){
	char command = 0;
	send_command_motor(huart,command,speed1);
	command = 4;
	send_command_motor(huart,command,speed2);
}

void drive_backwards(UART_HandleTypeDef* huart, char speed1, char speed2){
	char command = 1;
	send_command_motor(huart,command,speed1);
	command = 5;
	send_command_motor(huart,command,speed2);
}

void turn_right(UART_HandleTypeDef* huart, char speed1, char speed2){
	char command = 1;
	send_command_motor(huart,command,speed1);
	command = 4;
	send_command_motor(huart,command,speed2);
}

void turn_left(UART_HandleTypeDef* huart, char speed1, char speed2){
	char command = 0;
	send_command_motor(huart,command,speed1);
	command = 5;
	send_command_motor(huart,command,speed2);
}

void stop_motors(UART_HandleTypeDef* huart){
	drive_forward(huart, 0, 0);
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

/*uint16_t motor_encoder(TIM_HandleTypeDef* htim,TIM_HandleTypeDef* htim2, uint16_t* counter,uint16_t* counter2,uint16_t speed_d, uint16_t speed_command, uint16_t* motor_speed){
	uint16_t cnt2 = 0;
	uint16_t cnt3 = 0;
	uint16_t diff = 0;
	uint16_t diff2 = 0;
	uint32_t speed1 = 0;
	uint32_t speed2 = 0;
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

	speed1 = (((diff * 60)/ (64*19)))*10;
	if (speed1 == 32330){
		speed1 = 0;
	}

	cnt3 = __HAL_TIM_GET_COUNTER(htim2);
	if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim2)) {
		if (cnt3 < *counter2)
			diff2 = *counter2 - cnt3;
		else
			diff2 = (65535 - cnt3) + *counter2;
	} else {
		if (cnt3 > *counter2)
			diff2 = cnt3 - *counter2;
		else
			diff2 = (65535 - *counter2) + cnt3;
	}

	speed2 = (((diff2 * 60)/ (64*19)))*10;
	if (speed2 == 32330){
		speed2 = 0;
	}

	if(speed1<speed2){
		*motor_speed = speed1;
	}else {
		*motor_speed = speed2;
	}

	errore = abs(speed_d - *motor_speed);


	//TODO  regolazione della retroazione
	if (errore < 10 ){
		return speed_command;
	}
	else if((speed_d < *motor_speed) && (speed_d <= 250)){
		speed_command = speed_command - (ceil((errore*2)/9));
	}else if(speed_d > *motor_speed && (speed_d > 250)){
		speed_command = speed_command + ceil((ceil(((errore*2)/9)) /10));
	}else if((speed_d < *motor_speed) && (speed_d > 250)){
		speed_command = speed_command - (ceil((errore*2)/9));
	}else if(speed_d > *motor_speed && (speed_d < 250)){
		speed_command = speed_command + (ceil((errore*2)/9)) ;
	}
	if (speed_command > 101){
		speed_command = 100;
	}

	errore = 0;
	if ((speed_command >= 6 + tmp) && speed_d >= 250 ){
		speed_command = tmp+ 2;
	}
	return speed_command;

}*/


uint16_t motor_encoder(TIM_HandleTypeDef* htim, UART_HandleTypeDef* huart, uint16_t* counter, uint16_t speed_des,  uint16_t speed_command, uint16_t* motor_speed, double* error_pre, double* pid_i_pre, t_motorcommand* cmd){
	double kp = 1;
	double kd = 2;
	double ki = 0.3;
	uint16_t cnt2 = 0;
	uint16_t diff = 0;
<<<<<<< HEAD
	double speed = 0;
	double errore = 0;
	double pid_p = 0;
	double pid_d = 0;
	double pid_i = 0;
	double pid =0;
	double speed_d = (speed_des*9)/2;
	uint16_t final = 0;
=======
	uint32_t speed = 0;
	int errore = 0;
	float pid_p = 0;
	float pid_d = 0;
	float pid_i = 0;
	int speed_d = (speed_des*9)/2;
	int final = 0;
>>>>>>> parent of 0c44d72... encoder aggiustato
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

	speed = ((diff * 60)/ (64*19))*10;
	if (speed == 32330){
		speed = 0;
	}

	*motor_speed = speed;

	errore = speed_d - speed;

	pid_p = kp*errore;
	pid_d = kd * ((errore - *error_pre)/100);

	if (abs(errore) < 15){
		return speed_command;
	}
	pid_i = *pid_i_pre + (ki*errore);
	*pid_i_pre = pid_i;
	*error_pre = errore;
	pid = ((pid_p + pid_d + pid_i)*2)/9;

<<<<<<< HEAD
	if (pid > 127)
		return 127;

	if (pid < -127){
		pid = -127;
	}
	if (pid < 0){
=======

	final = speed_command + ceil(((pid_p + pid_d + pid_i)*2)/9);
	if (final > 127){
		final = 127;
	}
	/*
	if (final < 0){
>>>>>>> parent of 0c44d72... encoder aggiustato
		switch (cmd->command){
			case 8:
				cmd->command += 1;
				break;
			case 9:
				cmd->command -= 1;
				break;
			case 10:
				cmd->command += 1;
				break;
			case 11:
				cmd->command -= 1;
				break;
		}
	}

<<<<<<< HEAD
	final = abs(pid);
	if (final > 127){
		final = 127;
	}
	if (speed != 0){
		sprintf(msg, "---- %d ----\n\r", (final));
		HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg),0xFFFFFF);
		sprintf(msg, "%lf %lf %lf %.2lf\n\r", (speed), (errore), speed_d, pid);
		HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg),0xFFFFFF);
	}
=======
	sprintf(msg, "%d %d\n\r", (speed_command), ceil(((pid_p + pid_d + pid_i)*2)/9));
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg),0xFFFFFF);
	sprintf(msg, "%d %d\n\r", (final), cmd->command);
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg),0xFFFFFF);

	*/
	return final;

>>>>>>> parent of 0c44d72... encoder aggiustato

	return final;
}

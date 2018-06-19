/*
 * motor_handler.h
 *
 *  Created on: 06 giu 2018
 *      Author: mattiaditrolio
 */

#ifndef MOTOR_HANDLER_H_
#define MOTOR_HANDLER_H_

#include "common.h"

/*! brief Set the UART to comunicate with motor
 *
 * @param huart UART Handler
 * @param command commnad to send to the motors
 * @param speed  the speed
 * @return None
 */
void send_command_motor(UART_HandleTypeDef* huart,char command,char speed);

void drive_forward(UART_HandleTypeDef* huart, char speed1, char speed2);

void drive_backwards(UART_HandleTypeDef* huart, char speed1, char speed2);

void turn_right(UART_HandleTypeDef* huart, char speed1, char speed2);

void turn_left(UART_HandleTypeDef* huart,  char speed1, char speed2);

void stop_motors(UART_HandleTypeDef* huart);

void motor_Init(UART_HandleTypeDef* huart);

uint16_t motor_encoder(TIM_HandleTypeDef* htim, UART_HandleTypeDef* huart, uint16_t* counter, uint16_t speed_des, uint16_t speed_command, uint16_t* motor_speed, double* error_pre, double* pid_i_pre, t_motorcommand* cmd);

#endif /* MOTOR_HANDLER_H_ */

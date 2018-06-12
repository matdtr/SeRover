/*
 * autonomus_handler.c
 *
 *  Created on: 07 giu 2018
 *      Author: carmineannunziata
 */


#include "autonomus_handler.h"
#include "lane_handler.h"
#include "sonar_handler.h"
#include "motor_handler.h"
#define MIN_SONAR 50

void movement(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef* huart, uint16_t dev_address){
	int speed=10;


	while (1)
	{

			if ((read_lane_left()==1 && read_lane_center()==1 && read_lane_right()==1) && read_range_front(hi2c,dev_address)>MIN_SONAR) {
			 drive_forward(huart, speed);
			 }
			else if(read_lane_left()==0){
			 turn_right(huart, speed);
			}
			else if(read_lane_right()==0){
			 turn_left(huart, speed);
			}
			else{
				if (read_range_front(hi2c,dev_address)<MIN_SONAR)
				{drive_backwards(huart, speed);
				drive_forward(huart, speed*4);}
				else{
				stop_motors(huart);}
			}
	}

}


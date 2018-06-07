/*
 * lane_handler.c
 *
 *  Created on: 05 giu 2018
 *      Author: mattiaditrolio
 */


#include "lane_handler.h"

int read_lane_left(){
	if ( HAL_GPIO_ReadPin(LANE_GPIO_Port, LANE_LEFT) == 0){
			  return 1;
			}else{
			  return 0;
			}
}

int read_lane_center(){
	if ( HAL_GPIO_ReadPin(LANE_GPIO_Port, LANE_CENTER) == 0){
			  return 1;
			}else{
			  return 0;
			}
}

int read_lane_right(){
	if ( HAL_GPIO_ReadPin(LANE_GPIO_Port, LANE_RIGHT) == 0){
			  return 1;
			}else{
			  return 0;
			}
}



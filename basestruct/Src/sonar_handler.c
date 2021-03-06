/*
 * sonar_handler.c
 *
 *  Created on: 05 giu 2018
 *      Author: mattiaditrolio
 */

#include "sonar_handler.h"
#include "common.h"

uint16_t read_range(I2C_HandleTypeDef *hi2c, uint16_t dev_address) {
	  uint8_t range;
	  uint8_t range_hb;

	  send_command_sonar(hi2c,dev_address);
	  HAL_Delay(100);
	  HAL_I2C_Mem_Read(hi2c, dev_address, 2, 1, &range, 1, HAL_MAX_DELAY);
	  range_hb = range;

	  HAL_I2C_Mem_Read(hi2c, dev_address, 3, 1, &range, 1, HAL_MAX_DELAY);

	  return ((uint16_t)range_hb << 8) | range;
}


void send_command_sonar(I2C_HandleTypeDef *hi2c, uint16_t dev_address){
	uint8_t command[2];
	command[0] = 0x0;
	command[1] = 81;

	HAL_I2C_Master_Transmit(hi2c, dev_address, (uint8_t*)&command, sizeof(command), HAL_MAX_DELAY);

}

void sonar_Init(I2C_HandleTypeDef *hi2c, uint16_t dev_address, uint8_t gain){
	uint8_t* gain_pointer;
	*gain_pointer = gain;
	HAL_I2C_Mem_Write(hi2c, dev_address, 1, 1, (uint8_t *)gain_pointer, 1, HAL_MAX_DELAY);
}

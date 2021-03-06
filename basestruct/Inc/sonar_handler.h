/*
 * sonar_handler.h
 *
 *  Created on: 05 giu 2018
 *      Author: mattiaditrolio
 */

#ifndef SONAR_HANDLER_H_
#define SONAR_HANDLER_H_

#include "common.h"
/*! brief Reads the value from the front sensor
 *
 * @param hi2c i2c handler
 * @param dev_address device address
 * @return value
 */
uint16_t read_range(I2C_HandleTypeDef *hi2c, uint16_t dev_address);

/*! brief	Send the command to initiate a ranging
 *
 * @param hi2c i2c handler
 * @param dev_address device address
 * @return none
 */
void send_command_sonar(I2C_HandleTypeDef *hi2c, uint16_t dev_ddress);

void sonar_Init(I2C_HandleTypeDef *hi2c, uint16_t dev_address, uint8_t gain);

#endif /* SONAR_HANDLER_H_ */

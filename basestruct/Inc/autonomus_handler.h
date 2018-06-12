/*
 * autonomus_handler.h
 *
 *  Created on: 07 giu 2018
 *      Author: carmineannunziata
 */

#ifndef AUTONOMUS_HANDLER_H_
#define AUTONOMUS_HANDLER_H_

#include "common.h"

void movement(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef* huart, uint16_t dev_address);

#endif /* AUTONOMUS_HANDLER_H_ */

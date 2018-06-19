#ifndef COMMON_H_
#define COMMON_H_

#include "mxconstants.h"
#include "stm32f4xx_hal.h"

#include <string.h>
#include <stdlib.h>
#define N_LEDS (32)

typedef struct motorcommand
{
  int command;
  int value;

} t_motorcommand;


/*
 *  Reset di tutti i comandi
 */
void reset_commands(t_motorcommand* motorcommand);

/*
 *  Parse del comando ricevuto tramite bluetooth
 */
void parse_command(char* c, t_motorcommand* motorcommand);

/*
 *  Send sensors info to Web Application
 */
void get_sensors_info(UART_HandleTypeDef* huart, uint16_t motor_speed, int bright, uint16_t range_sonar1, uint16_t range_sonar2, uint8_t line1, uint8_t line2, uint8_t line3);

#endif // COMMON_H_

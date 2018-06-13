#ifndef COMMON_H_
#define COMMON_H_

#include "mxconstants.h"
#include "stm32f4xx_hal.h"

#define N_LEDS (32)

/*
 *  Reset di tutti i comandi
 */
void reset_commands(int* forward, int* reverse, int* right, int* left, uint16_t* speed_command);

/*
 *  Parse del comando ricevuto tramite bluetooth
 */
void parse_command(char* c, int* forward, int* reverse, int* right, int* left, int* bright);


#endif // COMMON_H_

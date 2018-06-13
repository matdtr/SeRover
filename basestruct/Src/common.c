/*
 * common.c
 *
 *  Created on: 06 giu 2018
 *      Author: mattiaditrolio
 */

#include "common.h"


void reset_commands(int* forward, int* reverse, int* right, int* left, uint16_t* speed_command){
	*forward = 0;
	*right = 0;
	*left = 0;
	*reverse = 0;
	*speed_command = 0;
}


void parse_command(char* c, int* forward, int* reverse, int* right, int* left, int* bright){
	char* command = strtok(c, "#");
	*forward = atoi(command);
	command = strtok(0, "#");
	*reverse = atoi(command);
	command = strtok(0, "#");
	*left = atoi(command);
	command = strtok(0, "#");
	*right = atoi(command);
	command = strtok(0, "#");
	if (command == NULL){
		*bright = 1;
	}else {
		*bright = atoi(command);
	}
}

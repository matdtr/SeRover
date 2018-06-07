/*
 * lane_handler.h
 *
 *  Created on: 05 giu 2018
 *      Author: mattiaditrolio
 */

#ifndef LANE_HANDLER_H_
#define LANE_HANDLER_H_

#include "common.h"
/*! brief Reads Lane Left Sensor
 *
 * @param none
 * @return 1 if true, 0 if false
 */
int read_lane_left();
/*! brief Reads Lane Center Sensor
 *
 * @param none
 * @return 1 if true, 0 if false
 */
int read_lane_center();
/*! brief Reads Lane Right Sensor
 *
 * @param none
 * @return 1 if true, 0 if false
 */
int read_lane_right();


#endif /* LANE_HANDLER_H_ */

#ifndef WS2812_HANDLER_H
#define WS2812_HANDLER_H

#include "common.h"

#define MAX(a,b) ((a>b)?a:b)
#define MIN(a,b) ((a<b)?a:b)

/*! brief Init ws2812 matrix
 *
 * @param none
 * @return none
 */
void ws2812_init_leds();
/*! brief Sets the color of the matrix
 *
 * @param red red value
 * @param green green value
 * @param blue blue value
 * @return none
 */
void ws2812_set_color_matrix(uint8_t red, uint8_t green, uint8_t blue);
/*! brief Starts the PWM signal generation in DMA mode to set colors
 *
 * @param none
 * @return none
 */
void ws2812_apply_colors();
/*! brief Turns off the led matrix
 *
 * @param none
 * @return none
 */
void ws2812_turn_off_leds();
/*! brief Sets the color of a single led
 *
 * @param LEDnumber index of the led
 * @param RED red value
 * @param GREEN green value
 * @param BLUE blue value
 * @return none
 */
void set_led_color(uint32_t LEDnumber, uint8_t RED, uint8_t GREEN, uint8_t BLUE);

//void ws2812_fadeEffect( uint8_t maxLevel );
//void ws2812_setColor(int ledIdx, const uint8_t *color);

// >>>>> Color definitions
#define	OFF		0
#define FULL	255
#define MID		150
#define SOFT	15
static const uint8_t BLACK[] =
{ OFF, OFF, OFF };
static const uint8_t RED[] =
{ FULL, OFF, OFF };
static const uint8_t GREEN[] =
{ OFF, FULL, OFF };
static const uint8_t BLUE[] =
{ OFF, OFF, FULL };
static const uint8_t WHITE[] =
{ FULL, FULL, FULL };
static const uint8_t SOFT_WHITE[] =
{ SOFT, SOFT, SOFT };
static const uint8_t MID_WHITE[] =
{ MID, MID, MID };
static const uint8_t MID_RED[] =
{ MID, OFF, OFF };
static const uint8_t MID_GREEN[] =
{ OFF, MID, OFF };
static const uint8_t MID_BLUE[] =
{ OFF, OFF, MID };
static const uint8_t SOFT_RED[] =
{ SOFT, OFF, OFF };
static const uint8_t SOFT_GREEN[] =
{ OFF, SOFT, OFF };
static const uint8_t SOFT_BLUE[] =
{ OFF, OFF, SOFT };
// <<<<< Color definitions


#endif

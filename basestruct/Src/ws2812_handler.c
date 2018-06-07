#include "ws2812_handler.h"

#include "tim.h"
#include "math.h"

#define N_LEDS	(32)

#define H_VAL 75
#define L_VAL 30
#define BITS_PER_LED (3*8)
//#define BIT_BUF_SIZE (((N_LEDS+1) * BITS_PER_LED) ) // Added a "virtual led" to avoid PWM to generate false bits
#define BIT_BUF_SIZE (((N_LEDS) * BITS_PER_LED) + 50 + 1 +50 )

uint32_t ws2812BitBuf[BIT_BUF_SIZE];

void ws2812_init_leds()
{
	for (int i = 0; i < BIT_BUF_SIZE; i++)
		ws2812BitBuf[i] = 0;

	fillBufferBlack();
}

void ws2812_turn_off_leds()
{
	for (int i = 0; i < N_LEDS; i++)
		set_led_color(i,0,0,0);
	fillBufferBlack();
	ws2812_apply_colors();

}

void fillBufferBlack() {
	/*Fill LED buffer - ALL OFF*/
	uint32_t index, buffIndex;
	buffIndex = 0;

	for (index = 0; index < 50; index++) {
		ws2812BitBuf[buffIndex] = 0;
		buffIndex++;
	}
	for (index = 0; index < BIT_BUF_SIZE; index++) {
		ws2812BitBuf[buffIndex] = L_VAL;
		buffIndex++;
	}
	ws2812BitBuf[buffIndex] = L_VAL;
	buffIndex++;
	for (index = 0; index < 50; index++) {
		ws2812BitBuf[buffIndex] = 0;
		buffIndex++;
	}
	ws2812_apply_colors();
}

void ws2812_set_color_matrix(uint8_t red, uint8_t green, uint8_t blue){

	for (int i = 0; i < N_LEDS; i++)
	{
		set_led_color(i,red,green,blue);
	}

	ws2812_apply_colors();
}

void set_led_color(uint32_t LEDnumber, uint8_t RED, uint8_t GREEN, uint8_t BLUE) {
	uint8_t tempBuffer[24];
	uint32_t i;
	uint32_t LEDindex;
	LEDindex = LEDnumber % N_LEDS;

	for (i = 0; i < 8; i++) // GREEN data
		tempBuffer[i] = ((GREEN << i) & 0x80) ? H_VAL : L_VAL;
	for (i = 0; i < 8; i++) // RED
		tempBuffer[8 + i] = ((RED << i) & 0x80) ? H_VAL : L_VAL;
	for (i = 0; i < 8; i++) // BLUE
		tempBuffer[16 + i] = ((BLUE << i) & 0x80) ? H_VAL : L_VAL;

	for (i = 0; i < 24; i++)
		ws2812BitBuf[50 + LEDindex * 24 + i] = tempBuffer[i];
}

void ws2812_apply_colors()
{
	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t*) ws2812BitBuf,BIT_BUF_SIZE);
}


/* void ws2812_setColor(int ledIdx, const uint8_t *color)
{
	if (ledIdx >= N_LEDS)
		return;

	uint8_t r = color[0];
	uint8_t g = color[1];
	uint8_t b = color[2];

	int i = ledIdx * BITS_PER_LED;
	uint8_t mask;
	mask = 0x80;
	while (mask)
	{
		ws2812BitBuf[i] = (mask & g) ? H_VAL : L_VAL;
		mask >>= 1;
		i++;
	}
	mask = 0x80;
	while (mask)
	{
		ws2812BitBuf[i] = (mask & r) ? H_VAL : L_VAL;
		mask >>= 1;
		i++;
	}
	mask = 0x80;
	while (mask)
	{
		ws2812BitBuf[i] = (mask & b) ? H_VAL : L_VAL;
		mask >>= 1;
		i++;
	}
}*/

/*
void ws2812_fadeEffect( uint8_t maxLevel  )
{
	uint32_t delay = (uint32_t)(ceilf(255.0/maxLevel)*2.0f);

	for (int t = 0; t < maxLevel; t++)
	{
		for (int i = 0; i < N_LEDS; i++)
		{
			uint8_t color[] =
			{ t, 0, 0 };
			ws2812_setColor(i, color);
		}

		ws2812_applyColors();
		HAL_Delay(delay);
	}

	for (int t = maxLevel; t != 0; t--)
	{
		for (int i = 0; i < N_LEDS; i++)
		{
			uint8_t color[] =
			{ t, 0, 0 };
			ws2812_setColor(i, color);
		}

		ws2812_applyColors();
		HAL_Delay(delay);
	}

	for (int t = 0; t < maxLevel; t++)
	{
		for (int i = 0; i < N_LEDS; i++)
		{
			uint8_t color[] =
			{ 0, t, 0 };
			ws2812_setColor(i, color);
		}

		ws2812_applyColors();
		HAL_Delay(delay);
	}

	for (int t = maxLevel; t != 0; t--)
	{
		for (int i = 0; i < N_LEDS; i++)
		{
			uint8_t color[] =
			{ 0, t, 0 };
			ws2812_setColor(i, color);
		}

		ws2812_applyColors();
		HAL_Delay(delay);
	}

	for (int t = 0; t < maxLevel; t++)
	{
		for (int i = 0; i < N_LEDS; i++)
		{
			uint8_t color[] =
			{ 0, 0, t };
			ws2812_setColor(i, color);
		}

		ws2812_applyColors();
		HAL_Delay(delay);
	}

	for (int t = maxLevel; t != 0; t--)
	{
		for (int i = 0; i < N_LEDS; i++)
		{
			uint8_t color[] =
			{ 0, 0, t };
			ws2812_setColor(i, color);
		}

		ws2812_applyColors();
		HAL_Delay(delay);
	}

	ws2812_turnOffLeds();
}*/


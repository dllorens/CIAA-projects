/*
 * ledrgb.h
 *
 *  Created on: 1/6/2016
 *      Author: GCS-IUA
 */

#ifndef LEDRGB_H_
#define LEDRGB_H_

#include "led.h"
#include "stdint.h"


typedef struct{
	uint8_t red;
	uint8_t green;
	uint8_t blue;
} rgbColor;


void RgbLedInit(void);
void RgbLedSetColor_1(uint8_t red, uint8_t green, uint8_t blue);
void RgbLedSetColor_2(rgbColor color);
void RgbLedSetPeriod(uint8_t _cyclesPerPeriod);

void RgbLedUpdate(void);

#endif /* LEDRGB_H_ */

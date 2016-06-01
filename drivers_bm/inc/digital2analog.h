/*
 * digital2analog.h
 *
 *  Created on: 1/6/2016
 *      Author: GCS-IUA
 */

#ifndef DIGITAL2ANALOG_H_
#define DIGITAL2ANALOG_H_

#include "stdint.h"

#define lpc4337			1
#define mk60fx512vlq15	2

#define DAC_BIT_RESOLUTION	1024
#define DAC_MAX_OUTPUT		3.3

void DACInit(void);
void DACWrite(uint32_t digitalCount);

#endif /* DIGITAL2ANALOG_H_ */

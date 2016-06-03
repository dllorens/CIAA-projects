/*
 * squaresignal.h
 *
 *  Created on: 3/6/2016
 *      Author: GCS-IUA
 */

#ifndef SQUARESIGNAL_H_
#define SQUARESIGNAL_H_

#include "stdint.h"

void SignalSquareInit(void);
void SignalSquareSetOffset(double value);
void SignalSquareSetAmplitude(double value);
void SignalSquareSetPeriod(double value);
void SignalSquareSetSamples(uint32_t samples);

double SignalSquareGetNextValue(void);
void SignalSquareReset(void);

#endif /* SQUARESIGNAL_H_ */

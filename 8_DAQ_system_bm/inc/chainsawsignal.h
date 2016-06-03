/*
 * chainsawsignal.h
 *
 *  Created on: 1/6/2016
 *      Author: GCS-IUA
 */

#ifndef CHAINSAWSIGNAL_H_
#define CHAINSAWSIGNAL_H_

#include "stdint.h"

void SignalChainSawInit(void);
void SignalChainSawSetAmplitude(double value);
void SignalChainSawSetPeriod(double value);
void SignalChainSawSetSamples(uint32_t samples);

double SignalChainSawGetNextValue(void);
void SignalChainSawReset(void);

#endif /* CHAINSAWSIGNAL_H_ */

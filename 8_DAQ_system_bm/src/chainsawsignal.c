/*
 * chainsawsignal.c
 *
 *  Created on: 1/6/2016
 *      Author: GCS-IUA
 */

#include "chainsawsignal.h"

static double amplitude;
static double period;
static uint32_t samplesPerPeriod;
static uint32_t currentSample;

void SignalChainSawInit(void){
	amplitude = 1.0f;
	period = 1.0f;

	samplesPerPeriod = 20;
	currentSample = 0;
}

void SignalChainSawSetAmplitude(double value){
	amplitude = value;
}

void SignalChainSawSetPeriod(double value){
	period = value;
}

void SignalChainSawSetSamples(uint32_t samples){
	samplesPerPeriod = samples;
}

double SignalChainSawGetNextValue(void){
	double result;

	++currentSample;
	if( currentSample > samplesPerPeriod ){
		currentSample = 0;
	}

	result = amplitude * ((double)currentSample/(double)samplesPerPeriod);

	return result;
}

void SignalChainSawReset(void){
	currentSample = 0;
}

/*
 * squaresignal.c
 *
 *  Created on: 3/6/2016
 *      Author: GCS-IUA
 */

#include "squaresignal.h"


static double amplitude;
static double period;
static double offset;
static uint32_t samplesPerPeriod;
static uint32_t currentSample;

void SignalSquareInit(void){
	amplitude = 1.0f;
	period = 1.0f;
	offset = 0.0f;

	samplesPerPeriod = 20;
	currentSample = 0;
}

void SignalSquareSetOffset(double value){
	offset = value;
}

void SignalSquareSetAmplitude(double value){
	amplitude = value;
}

void SignalSquareSetPeriod(double value){
	period = value;
}

void SignalSquareSetSamples(uint32_t samples){
	samplesPerPeriod = samples;
}

double SignalSquareGetNextValue(void){
	double result;

	++currentSample;
	if( currentSample > samplesPerPeriod ){
		currentSample = 0;
	}

	if( currentSample > samplesPerPeriod / 2){
		result = amplitude;
	}
	else{
		result = 0.0;
	}

	result += offset;

	return result;
}

void SignalSquareReset(void){
	currentSample = 0;
}

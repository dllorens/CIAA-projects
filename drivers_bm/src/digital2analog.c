/*
 * digital2analog.c
 *
 *  Created on: 1/6/2016
 *      Author: GCS-IUA
 */

#ifndef CPU
#error CPU shall be defined
#endif
#if (lpc4337 == CPU)
#include "chip.h"
#elif (mk60fx512vlq15 == CPU)
#else
#endif

#include "digital2analog.h"

void DACInit(void){
	Chip_SCU_DAC_Analog_Config();
	Chip_DAC_Init(LPC_DAC);
	Chip_DAC_ConfigDAConverterControl(LPC_DAC, DAC_DMA_ENA);
}

void DACWrite(uint32_t digitalCount){
	//uint32_t digitalCounts = (uint32_t)(value/(double)DAC_MAX_OUTPUT*(double)(DAC_BIT_RESOLUTION-1));
	Chip_DAC_UpdateValue(LPC_DAC, digitalCount);
}

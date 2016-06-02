/*
 * ADC.h
 *
 *  Created on: 2/6/2016
 *      Author: GCS-IUA
 */

#ifndef ADC_H_
#define ADC_H_

/*==================[inclusions]=============================================*/
#include "stdint.h"
#include "lpc_types.h"

/*==================[macros]=================================================*/
#define lpc4337            1
#define mk60fx512vlq15     2

#define ADC_CHANNEL_0	0
#define ADC_CHANNEL_1	1

FlagStatus ADCInit(uint8_t adcChannel);
double ADCGetMaxVoltage(void);
uint8_t ADCGetBitResolution(void);

//void ADCStart(void);
//void ADCConfig

uint32_t ADCReadBlocking(uint8_t adcChannel);

#endif /* ADC_H_ */

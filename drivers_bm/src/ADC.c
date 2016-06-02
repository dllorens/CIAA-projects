/*
 * ADC.c
 *
 *  Created on: 2/6/2016
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
#include "ADC.h"

#define DEFAULT_ADC		LPC_ADC0
#define DEFAULT_ADC_ADD	0					// ID del ADC (posición en un vector de 0 a 2)

static ADC_CLOCK_SETUP_T 	adcClockConfig;
static ADC_START_MODE_T		adcStartMode;
static ADC_EDGE_CFG_T		adcEdgeMode;
static ADC_STATUS_T			adcStatus;
static FlagStatus			adcReadStatus;

static LPC_ADC_T	*selectedADC;
static uint32_t		selectedADCAddress;

static uint8_t ADCGetChannelNumber(uint8_t channelName);
static void ADCDefaultConfig(void);

FlagStatus ADCInit(uint8_t adcChannel){

	ADCDefaultConfig();

	uint8_t channel = ADCGetChannelNumber(adcChannel);
	Chip_SCU_ADC_Channel_Config(selectedADCAddress,channel);

	Chip_ADC_Init(selectedADC, &adcClockConfig );
	Chip_ADC_EnableChannel(selectedADC, adcChannel, ENABLE);

	return adcReadStatus;
}

/*
void ADCStart(void){
	// Iniciar a convertir valores en el ADC
	adcStartMode = ADC_START_NOW;
	Chip_ADC_SetStartMode(selectedADC, adcStartMode, adcEdgeMode);
}
*/

double ADCGetMaxVoltage(void){
	return 3.3;
}

uint8_t ADCGetBitResolution(void){
	return 10;
}
uint32_t ADCReadBlocking(uint8_t channel){
	uint16_t value;
	uint8_t adcChannel = ADCGetChannelNumber(channel);

	// Iniciar la conversión
	Chip_ADC_SetStartMode(selectedADC, ADC_START_NOW, adcEdgeMode);

	// Esperar a que se complete la conversión
	while(Chip_ADC_ReadStatus(selectedADC,adcChannel,ADC_DR_DONE_STAT) != SUCCESS ){
	}

	// Leer el valor de la conversión
	Chip_ADC_ReadValue(selectedADC,channel,&value);

	return (uint32_t)value;
}

static uint8_t ADCGetChannelNumber(uint8_t channelName){
	switch(channelName){
	case ADC_CHANNEL_0:
		return ADC_CH0;
		break;
	case ADC_CHANNEL_1:
		return ADC_CH1;
		break;
	default:
		return ADC_CH0;
		break;
	}
}

static void ADCDefaultConfig(void){
	selectedADC = DEFAULT_ADC;
	selectedADCAddress = DEFAULT_ADC_ADD;

	adcClockConfig.adcRate = 10000;
	adcClockConfig.bitsAccuracy = ADC_10BITS;
	adcClockConfig.burstMode = DISABLE;

	adcStartMode = ADC_NO_START;

	adcEdgeMode = ADC_TRIGGERMODE_RISING;

	adcStatus = ADC_DR_DONE_STAT;
}

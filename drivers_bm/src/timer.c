/*
 * timer.c
 *
 *  Created on: 31/5/2016
 *      Author: GCS-IUA
 */

#include "timer.h"
#include "chip.h"

#define RIT_TIMER_IRQ	11

void InitTimer(uint8_t timer){
	switch(timer){
	case RIT_TIMER:
		Chip_RIT_Init(LPC_RITIMER);
		SetDelay(RIT_TIMER,100);
		break;
	default:
		break;
	}
}

void SetDelay(uint8_t timer, uint32_t milliseconds){
	switch(timer){
	case RIT_TIMER:
		Chip_RIT_SetTimerInterval(LPC_RITIMER,milliseconds);
		break;
	default:
		break;
	}
}

void StartTimer(uint8_t timer){
	switch(timer){
	case RIT_TIMER:
		NVIC_EnableIRQ(RIT_TIMER_IRQ);
		break;
	default:
		break;
	}
}

void CLearTimer(uint8_t timer){
	switch(timer){
	case RIT_TIMER:
		Chip_RIT_ClearInt(LPC_RITIMER);
		break;
	default:
		break;
	}
}

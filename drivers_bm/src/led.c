/* Copyright 2016, XXXXXXXXX  
 * All rights reserved.
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \brief Blinking Bare Metal driver led
 **
 **
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */

/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Baremetal Bare Metal LED Driver
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 *
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * yyyymmdd v0.0.1 initials initial version
 */

/*==================[inclusions]=============================================*/

#ifndef CPU
#error CPU shall be defined
#endif
#if (lpc4337 == CPU)
#include "chip.h"
#elif (mk60fx512vlq15 == CPU)
#else
#endif
#include "led.h"

/*==================[macros and definitions]=================================*/
#define PIN_GROUP_2		2
#define PIN_NUMBER_0 	0
#define PIN_NUMBER_1 	1
#define PIN_NUMBER_2 	2
#define PIN_NUMBER_10 	10
#define PIN_NUMBER_11 	11
#define PIN_NUMBER_12 	12

#define REDLED_PORT 	1
#define GREENLED_PORT	1
#define YELOWLED_PORT	0
#define RGBLED_PORT		5

#define REDLED_BIT		1<<11
#define GREENLED_BIT	1<<12
#define YELOWLED_BIT	1<<14
#define RGBLEDRED_BIT 	1<<0
#define RGBLEDGREEN_BIT	1<<1
#define RGBLEDBLUE_BIT	1<<2

#define OUTPUT 	!0
/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
/** \brief Main function
 *
 * This is the main entry point of the software.
 *
 * \returns 0
 *
 * \remarks This function never returns. Return value is only to avoid compiler
 *          warnings or errors.
 */

void InitLed(void){
	 Chip_GPIO_Init(LPC_GPIO_PORT);

	 // Led RGB
	 Chip_SCU_PinMux(PIN_GROUP_2,PIN_NUMBER_0,MD_PUP,FUNC4);
	 Chip_SCU_PinMux(PIN_GROUP_2,PIN_NUMBER_1,MD_PUP,FUNC4);
	 Chip_SCU_PinMux(PIN_GROUP_2,PIN_NUMBER_2,MD_PUP,FUNC4);

	 // Led amarillo
	 Chip_SCU_PinMux(PIN_GROUP_2,PIN_NUMBER_10,MD_PUP,FUNC0);

	 // Led rojo
	 Chip_SCU_PinMux(PIN_GROUP_2,PIN_NUMBER_11,MD_PUP,FUNC0);

	 // Led verde
	 Chip_SCU_PinMux(PIN_GROUP_2,PIN_NUMBER_12,MD_PUP,FUNC0);

	 // Configurar los puertos para salida
	 Chip_GPIO_SetDir(LPC_GPIO_PORT, RGBLED_PORT , RGBLEDBLUE_BIT ,OUTPUT);
	 Chip_GPIO_SetDir(LPC_GPIO_PORT, RGBLED_PORT , RGBLEDGREEN_BIT ,OUTPUT);
	 Chip_GPIO_SetDir(LPC_GPIO_PORT, RGBLED_PORT , RGBLEDRED_BIT ,OUTPUT);

	 Chip_GPIO_SetDir(LPC_GPIO_PORT, REDLED_PORT , REDLED_BIT ,OUTPUT);
	 Chip_GPIO_SetDir(LPC_GPIO_PORT, YELOWLED_PORT , YELOWLED_BIT ,OUTPUT);
	 Chip_GPIO_SetDir(LPC_GPIO_PORT, GREENLED_PORT , GREENLED_BIT ,OUTPUT);

	 // Apagar todos los leds
	 Chip_GPIO_SetPortOutLow(LPC_GPIO_PORT, RGBLED_PORT,RGBLEDBLUE_BIT);
	 Chip_GPIO_SetPortOutLow(LPC_GPIO_PORT, RGBLED_PORT,RGBLEDGREEN_BIT);
	 Chip_GPIO_SetPortOutLow(LPC_GPIO_PORT, RGBLED_PORT,RGBLEDRED_BIT);

	 Chip_GPIO_SetPortOutLow(LPC_GPIO_PORT, REDLED_PORT, REDLED_BIT);

	 Chip_GPIO_SetPortOutLow(LPC_GPIO_PORT, YELOWLED_PORT,YELOWLED_BIT);

	 Chip_GPIO_SetPortOutLow(LPC_GPIO_PORT, GREENLED_PORT,GREENLED_BIT);
}

void LedOn(const uint8_t led){
	switch(led){
	case REDLED:
		 Chip_GPIO_SetPortOutHigh(LPC_GPIO_PORT, REDLED_PORT, REDLED_BIT);
		 break;
	case GREENLED:
		 Chip_GPIO_SetPortOutHigh(LPC_GPIO_PORT, GREENLED_PORT,GREENLED_BIT);
		break;
	case YELOWLED:
		 Chip_GPIO_SetPortOutHigh(LPC_GPIO_PORT, YELOWLED_PORT,YELOWLED_BIT);
		break;
	case RGBLED:
		 Chip_GPIO_SetPortOutHigh(LPC_GPIO_PORT, RGBLED_PORT,RGBLEDBLUE_BIT);
		 Chip_GPIO_SetPortOutHigh(LPC_GPIO_PORT, RGBLED_PORT,RGBLEDGREEN_BIT);
		 Chip_GPIO_SetPortOutHigh(LPC_GPIO_PORT, RGBLED_PORT,RGBLEDRED_BIT);
		 break;
	case RGBLED_BLUE:
		Chip_GPIO_SetPortOutHigh(LPC_GPIO_PORT, RGBLED_PORT,RGBLEDBLUE_BIT);
		break;
	case RGBLED_GREEN:
		Chip_GPIO_SetPortOutHigh(LPC_GPIO_PORT, RGBLED_PORT,RGBLEDGREEN_BIT);
		break;
	case RGBLED_RED:
		Chip_GPIO_SetPortOutHigh(LPC_GPIO_PORT, RGBLED_PORT,RGBLEDRED_BIT);
		break;
	case RGBLED_VIOLT:
		Chip_GPIO_SetPortOutHigh(LPC_GPIO_PORT, RGBLED_PORT,RGBLEDRED_BIT);
		Chip_GPIO_SetPortOutHigh(LPC_GPIO_PORT, RGBLED_PORT,RGBLEDBLUE_BIT);
		break;
	default:
		break;
	}
}

void LedOff(const uint8_t led){
	switch(led){
	case REDLED:
		 Chip_GPIO_SetPortOutLow(LPC_GPIO_PORT, REDLED_PORT, REDLED_BIT);
		 break;
	case GREENLED:
		 Chip_GPIO_SetPortOutLow(LPC_GPIO_PORT, GREENLED_PORT,GREENLED_BIT);
		break;
	case YELOWLED:
		 Chip_GPIO_SetPortOutLow(LPC_GPIO_PORT, YELOWLED_PORT,YELOWLED_BIT);
		break;
	case RGBLED:
		 Chip_GPIO_SetPortOutLow(LPC_GPIO_PORT, RGBLED_PORT,RGBLEDBLUE_BIT);
		 Chip_GPIO_SetPortOutLow(LPC_GPIO_PORT, RGBLED_PORT,RGBLEDGREEN_BIT);
		 Chip_GPIO_SetPortOutLow(LPC_GPIO_PORT, RGBLED_PORT,RGBLEDRED_BIT);
		 break;
	case RGBLED_BLUE:
		Chip_GPIO_SetPortOutLow(LPC_GPIO_PORT, RGBLED_PORT,RGBLEDBLUE_BIT);
		break;
	case RGBLED_GREEN:
		Chip_GPIO_SetPortOutLow(LPC_GPIO_PORT, RGBLED_PORT,RGBLEDGREEN_BIT);
		break;
	case RGBLED_RED:
		Chip_GPIO_SetPortOutLow(LPC_GPIO_PORT, RGBLED_PORT,RGBLEDRED_BIT);
		break;
	case RGBLED_VIOLT:
		Chip_GPIO_SetPortOutLow(LPC_GPIO_PORT, RGBLED_PORT,RGBLEDRED_BIT);
		Chip_GPIO_SetPortOutLow(LPC_GPIO_PORT, RGBLED_PORT,RGBLEDBLUE_BIT);
		break;
	default:
		break;
	}
}

void ToggleLed(const uint8_t led){
	switch(led){
	case REDLED:
		Chip_GPIO_SetPortToggle(LPC_GPIO_PORT, REDLED_PORT, REDLED_BIT);
		break;
	case GREENLED:
		Chip_GPIO_SetPortToggle(LPC_GPIO_PORT, GREENLED_PORT,GREENLED_BIT);
		break;
	case YELOWLED:
		Chip_GPIO_SetPortToggle(LPC_GPIO_PORT, YELOWLED_PORT,YELOWLED_BIT);
		break;
	case RGBLED:
		Chip_GPIO_SetPortToggle(LPC_GPIO_PORT, RGBLED_PORT,RGBLEDBLUE_BIT);
		Chip_GPIO_SetPortToggle(LPC_GPIO_PORT, RGBLED_PORT,RGBLEDGREEN_BIT);
		Chip_GPIO_SetPortToggle(LPC_GPIO_PORT, RGBLED_PORT,RGBLEDRED_BIT);
		break;
	case RGBLED_BLUE:
		Chip_GPIO_SetPortToggle(LPC_GPIO_PORT, RGBLED_PORT,RGBLEDBLUE_BIT);
		break;
	case RGBLED_GREEN:
		Chip_GPIO_SetPortToggle(LPC_GPIO_PORT, RGBLED_PORT,RGBLEDGREEN_BIT);
		break;
	case RGBLED_RED:
		Chip_GPIO_SetPortToggle(LPC_GPIO_PORT, RGBLED_PORT,RGBLEDRED_BIT);
		break;
	case RGBLED_VIOLT:
		Chip_GPIO_SetPortToggle(LPC_GPIO_PORT, RGBLED_PORT,RGBLEDRED_BIT);
		Chip_GPIO_SetPortToggle(LPC_GPIO_PORT, RGBLED_PORT,RGBLEDBLUE_BIT);
		break;
	default:
		break;
	}
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/


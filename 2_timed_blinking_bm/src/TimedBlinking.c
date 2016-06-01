/* Copyright 2016, XXXXXX
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

/** \brief Blinking Bare Metal example source file
 **
 ** This is a mini example of the CIAA Firmware.
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */

/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Baremetal Bare Metal example source file
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

#include "TimedBlinking.h"       /* <= own header */

#include <stdint.h>

#include "../../drivers_bm/inc/led.h"
#include "../../drivers_bm/inc/timer.h"

/*==================[macros and definitions]=================================*/
#define TOGGLE_CYCLES 3
#define MAXCOUNTER 10000000

/*==================[internal data declaration]==============================*/
int32_t	counter = MAXCOUNTER;

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/
int32_t	i;
uint8_t		currentLed;
int32_t		accelCounter = 1000000;
uint8_t		elapsedCycles = 0;

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



int main(void)
{
	/* perform the needed initialization here */

	InitLed();					// Configurar los leds de la placa
	InitTimer(RIT_TIMER);		// Configurar el timer
	SetDelay(RIT_TIMER,80);		// Establecer un delay de 250 milisegundos en el timer

	// Prender el led rojo antes de empezar el encendido secuencial
	currentLed = REDLED;
	LedOn(currentLed);

	StartTimer(RIT_TIMER);		// Iniciar el timer

	for(;;){

	}

	return 0;
}

void ISR_RIT(void){
	ToggleLed(currentLed);	// apagar el led actual

	switch(currentLed){
	case REDLED:
		currentLed = YELOWLED;
		break;
	case YELOWLED:
		currentLed = GREENLED;
		break;
	case GREENLED:
		currentLed = RGBLED_RED;
		break;
	case RGBLED_RED:
		currentLed = RGBLED_BLUE;
		break;
	case RGBLED_BLUE:
		currentLed = RGBLED_GREEN;
		break;
	case RGBLED_GREEN:
		currentLed = RGBLED_VIOLT;
		break;
	case RGBLED_VIOLT:
		currentLed = REDLED;
		break;
	default:
		break;
	}

	ToggleLed(currentLed); // Encender el led actual

	CLearTimer(RIT_TIMER);	// Liberar la bandera del timer
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/


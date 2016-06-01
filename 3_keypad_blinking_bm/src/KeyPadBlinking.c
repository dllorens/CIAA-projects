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

#include "KeyPadBlinking.h"       /* <= own header */

#include <stdint.h>

#include "../../drivers_bm/inc/led.h"
#include "../../drivers_bm/inc/timer.h"
#include "../../drivers_bm/inc/buttons.h"

/*==================[macros and definitions]=================================*/
#define TOGGLE_CYCLES 3
#define MAXCOUNTER 10000000
#define CYCLESPRESSED	3			// Cantidad de ciclos antes de tomar un botón como presionado

#define NEXT_CMD	0
#define PREV_CMD	1
#define UP_CMD		2
#define DWN_CMD		3

/*==================[internal data declaration]==============================*/
int32_t	counter = MAXCOUNTER;
uint8_t commandList[4];					// Vector para vincular comandos con teclas
bool bttnStates[4] = {false, false, false, false};
uint8_t bttnCyclePressedCounters[4] = {0,0,0,0};


/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/
int32_t	i;
uint8_t		currentLed;
int32_t		accelCounter 	= 1000000;
//uint8_t		elapsedCycles 	= 0;
uint8_t		delayLedToggle	= 0;					// Contador para la cantidad de ciclos a esperar para cambiar el estado del led actual
uint8_t		elapsedToggleCycles = 0;				// Contador para la cantidad de ciclos desde que cambiamos por última vez el estado del led actual
uint8_t 	ledList[LEDCOUNT];

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
	InitButtons();
	InitTimer(RIT_TIMER);		// Configurar el timer
	SetDelay(RIT_TIMER,15);		// Establecer un delay de 250 milisegundos en el timer

	// Configurar lista de comandos
	commandList[NEXT_CMD] 	= BTTN_1;
	commandList[PREV_CMD] 	= BTTN_2;
	commandList[UP_CMD] 	= BTTN_3;
	commandList[DWN_CMD] 	= BTTN_4;

	// Lista de leds
	ledList[0]	= RGBLED;
	ledList[1]	= YELOWLED;
	ledList[2]	= REDLED;
	ledList[3]	= GREENLED;

	currentLed = ledList[0];

	// Prender el led rojo antes de empezar el encendido secuencial
	//currentLed = REDLED;
	LedOn(currentLed);

	StartTimer(RIT_TIMER);		// Iniciar el timer

	for(;;){

	}

	return 0;
}

void ISR_RIT(void){

	// Leer el estado actual de los botones
	bttnStates[NEXT_CMD] = ReadButton(commandList[NEXT_CMD]);
	bttnStates[PREV_CMD] = ReadButton(commandList[PREV_CMD]);
	bttnStates[UP_CMD] = ReadButton(commandList[UP_CMD]);
	bttnStates[DWN_CMD] = ReadButton(commandList[DWN_CMD]);

	// NEXT COMMAND
	if( bttnStates[NEXT_CMD] == true ){
		if( bttnCyclePressedCounters[NEXT_CMD] < CYCLESPRESSED ){
			// Esperar a que el botón se mantenga apretado por más tiempo
			bttnCyclePressedCounters[NEXT_CMD] = bttnCyclePressedCounters[NEXT_CMD] + 1;
		}
		else{
			// El botón ya estuvo presionado la cantidad de ciclos predefinidos.
			// Realizar acción
			bttnCyclePressedCounters[NEXT_CMD] = 0;
			LedOff(ledList[currentLed]);			// Apagar el led actual
			if( currentLed >= LEDCOUNT ){
				currentLed = 0;
			}
			else{
				++currentLed;
			}
			LedOn(ledList[currentLed]);		// Encender el nuevo led
			elapsedToggleCycles = 0;		// Reiniciar el contador de frecuencia de parpadeo
		}
	}
	else{
		bttnCyclePressedCounters[NEXT_CMD] = 0;
	}

	// PREV COMMAND
	if( bttnStates[PREV_CMD] == true ){
		if( bttnCyclePressedCounters[PREV_CMD] < CYCLESPRESSED ){
			// Esperar a que el botón se mantenga apretado por más tiempo
			bttnCyclePressedCounters[PREV_CMD] = bttnCyclePressedCounters[PREV_CMD] + 1;
		}
		else{
			// El botón ya estuvo presionado la cantidad de ciclos predefinidos.
			// Realizar acción
			bttnCyclePressedCounters[PREV_CMD] = 0;
			LedOff(ledList[currentLed]);			// Apagar el led actual
			if( currentLed <= 1 ){
				currentLed = LEDCOUNT;
			}
			else{
				--currentLed;
			}
			LedOn(ledList[currentLed]);		// Encender el nuevo led
			elapsedToggleCycles = 0;		// Reiniciar el contador de frecuencia de parpadeo
		}
	}
	else{
		bttnCyclePressedCounters[PREV_CMD] = 0;
	}

	// UP COMMAND
	if( bttnStates[UP_CMD] == true ){
		if( bttnCyclePressedCounters[UP_CMD] < CYCLESPRESSED ){
			// Esperar a que el botón se mantenga apretado por más tiempo
			bttnCyclePressedCounters[UP_CMD] = bttnCyclePressedCounters[UP_CMD] + 1;
		}
		else{
			// El botón ya estuvo presionado la cantidad de ciclos predefinidos.
			// Realizar acción
			bttnCyclePressedCounters[UP_CMD] = 0;
			if( delayLedToggle <= 0 ){
				delayLedToggle = 0;
			}
			else{
				--delayLedToggle;
			}
		}
	}
	else{
		bttnCyclePressedCounters[UP_CMD] = 0;
	}

	// DOWN COMMAND
	if( bttnStates[DWN_CMD] == true ){
		if( bttnCyclePressedCounters[DWN_CMD] < CYCLESPRESSED ){
			// Esperar a que el botón se mantenga apretado por más tiempo
			bttnCyclePressedCounters[DWN_CMD] = bttnCyclePressedCounters[DWN_CMD] + 1;
		}
		else{
			// El botón ya estuvo presionado la cantidad de ciclos predefinidos.
			// Realizar acción
			bttnCyclePressedCounters[DWN_CMD] = 0;
			if( delayLedToggle >= UINT8_MAX ){
				delayLedToggle = UINT8_MAX;
			}
			else{
				++delayLedToggle;
			}
		}
	}
	else{
		bttnCyclePressedCounters[DWN_CMD] = 0;
	}

	// Cambiar el estado del led actual
	if( elapsedToggleCycles < delayLedToggle ){
		++elapsedToggleCycles;
	}
	else{
		ToggleLed(ledList[currentLed]);				// Cambiar el estado del led actual
		elapsedToggleCycles = 0;					// Reiniciar el contador de ciclos de para parpadear
	}


	/*
	ToggleLed(currentLed);	// apagar el led actual

	switch(currentLed){
	case REDLED:
		currentLed = BLUELED;
		break;
	case BLUELED:
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
*/
	CLearTimer(RIT_TIMER);	// Liberar la bandera del timer
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/


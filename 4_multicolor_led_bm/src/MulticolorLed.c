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

#include "MulticolorLed.h"       /* <= own header */

#include <stdint.h>

#include "../../drivers_bm/inc/led.h"
#include "../../drivers_bm/inc/ledrgb.h"
#include "../../drivers_bm/inc/timer.h"
#include "../../drivers_bm/inc/buttons.h"

/*==================[macros and definitions]=================================*/
#define TIMER_DELAY_MS	1
#define BTTN_DELAY_MS	15
#define RGBLED_DELAY_MS	20
#define TOGGLE_CYCLES 3
#define MAXCOUNTER 10000000
#define CYCLESPRESSED	4			// Cantidad de ciclos antes de tomar un botón como presionado

#define NEXT_CMD	0
#define PREV_CMD	1
#define UP_CMD		2
#define DWN_CMD		3

/*==================[internal data declaration]==============================*/
int32_t	counter = MAXCOUNTER;

// Sistema de comandos por teclas
uint8_t commandList[4];					// Vector para vincular comandos con teclas
bool bttnStates[4] = {true, true, true, true};
uint8_t bttnCyclePressedCounters[4] = {0,0,0,0};
uint32_t	buttonsCounter = 0;				// Contador para ejecutar las funciones de comandos de teclas
uint32_t	buttonsCycles = (uint32_t)((float)(BTTN_DELAY_MS)/(float)(TIMER_DELAY_MS));				// Cantidad de ciclos que equivalen al tiempo en ms para ejecutar los comandos de las teclas

// Cambio de colores del led RGB
static rgbColor	ledColor;
static rgbColor	colorList[16];
static uint8_t 	currentColor = 0;
static uint8_t	ledUpdatecycles = (uint8_t)((float)(RGBLED_DELAY_MS)/(float)(TIMER_DELAY_MS));				// Cantidad de ciclos que equivalen al tiempo en ms para ejecutar los comandos de las teclas

/*==================[internal functions declaration]=========================*/
static void InitColorList(void);

/*==================[internal data definition]===============================*/
int32_t	i;
uint8_t		currentLed;
int32_t		accelCounter 	= 1000000;
//uint8_t		elapsedCycles 	= 0;
uint8_t		delayLedToggle	= 0;					// Contador para la cantidad de ciclos a esperar para cambiar el estado del led actual
uint8_t		elapsedToggleCycles = 0;				// Contador para la cantidad de ciclos desde que cambiamos por última vez el estado del led actual
uint8_t 	ledList[LEDCOUNT];

uint64_t	timeCounter = 0;

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

	InitLed();								// Configurar los leds de la placa
	RgbLedInit();
	InitButtons();
	InitTimer(RIT_TIMER);					// Configurar el timer
	SetDelay(RIT_TIMER,TIMER_DELAY_MS);		// Establecer un delay de 250 milisegundos en el timer

	// Configurar lista de comandos
	commandList[NEXT_CMD] 	= BTTN_2;
	commandList[PREV_CMD] 	= BTTN_1;
	commandList[UP_CMD] 	= BTTN_3;
	commandList[DWN_CMD] 	= BTTN_4;

	// Lista de leds
	ledList[0]	= RGBLED;
	ledList[1]	= YELOWLED;
	ledList[2]	= REDLED;
	ledList[3]	= GREENLED;

	currentLed = 0;

	// Prender el led rgb antes de empezar el encendido secuencial
	//currentLed = REDLED;
	LedOn(ledList[currentLed]);

	// Led RGB
	ledColor.red = 255;
	ledColor.green = 0;
	ledColor.blue = 0;

	InitColorList();				// Iniciar la lista de colores

	RgbLedSetColor_2(ledColor);
	RgbLedSetPeriod(ledUpdatecycles);

	StartTimer(RIT_TIMER);		// Iniciar el timer

	// Loop principal
	for(;;){

		// Detectar el accionamiento de las teclas y ejecutar los comandos de las teclas
		if( buttonsCounter >= buttonsCycles ){
			buttonsCounter = 0;

			// NEXT COMMAND
			if( bttnStates[NEXT_CMD] == false ){
				if( bttnCyclePressedCounters[NEXT_CMD] < CYCLESPRESSED ){
					// Esperar a que el botón se mantenga apretado por más tiempo
					bttnCyclePressedCounters[NEXT_CMD] = bttnCyclePressedCounters[NEXT_CMD] + 1;
				}
				else{
					// El botón ya estuvo presionado la cantidad de ciclos predefinidos.
					// Realizar acción
					bttnCyclePressedCounters[NEXT_CMD] = 0;
					/*
					LedOff(ledList[currentLed]);			// Apagar el led actual
					if( currentLed >= (LEDCOUNT - 1) ){
						currentLed = 0;
					}
					else{
						++currentLed;
					}
					LedOn(ledList[currentLed]);		// Encender el nuevo led
					elapsedToggleCycles = 0;		// Reiniciar el contador de frecuencia de parpadeo
					 */
				}
			}
			else{
				bttnCyclePressedCounters[NEXT_CMD] = 0;
			}

			// PREV COMMAND
			if( bttnStates[PREV_CMD] == false ){
				if( bttnCyclePressedCounters[PREV_CMD] < CYCLESPRESSED ){
					// Esperar a que el botón se mantenga apretado por más tiempo
					bttnCyclePressedCounters[PREV_CMD] = bttnCyclePressedCounters[PREV_CMD] + 1;
				}
				else{
					// El botón ya estuvo presionado la cantidad de ciclos predefinidos.
					// Realizar acción
					bttnCyclePressedCounters[PREV_CMD] = 0;
					LedOff(ledList[currentLed]);			// Apagar el led actual
					if( currentColor <= 0 ){
						currentColor = 4;
					}
					else{
						--currentColor;
					}

					// Cambiar el color del led
					RgbLedSetColor_2(colorList[currentColor]);
				}
			}
			else{
				bttnCyclePressedCounters[PREV_CMD] = 0;
			}

			// UP COMMAND
			if( bttnStates[UP_CMD] == false ){
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
			if( bttnStates[DWN_CMD] == false ){
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

		}
	}

	return 0;
}

void ISR_RIT(void){

	++timeCounter;

	// Leer el estado actual de los botones
	bttnStates[NEXT_CMD] = ReadButton(commandList[NEXT_CMD]);
	bttnStates[PREV_CMD] = ReadButton(commandList[PREV_CMD]);
	bttnStates[UP_CMD] = ReadButton(commandList[UP_CMD]);
	bttnStates[DWN_CMD] = ReadButton(commandList[DWN_CMD]);
	++buttonsCounter;

	RgbLedUpdate();

	CLearTimer(RIT_TIMER);	// Liberar la bandera del timer
}

static void InitColorList(void){
	rgbColor color;

	// Color 1
	color.red = 253;
	color.green = 244;
	color.blue = 81;
	colorList[0] = color;

	// Color 2
	color.red = 89;
	color.green = 237;
	color.blue = 244;
	colorList[1] = color;

	// Color 3
	color.red = 88;
	color.green = 92;
	color.blue = 245;
	colorList[2] = color;

	// Color 4
	color.red = 245;
	color.green = 88;
	color.blue = 245;
	colorList[3] = color;

	// Color 5
	color.red = 250;
	color.green = 166;
	color.blue = 39;
	colorList[4] = color;

	// Color 6
	color.red = 0;
	color.green = 0;
	color.blue = 0;
	colorList[5] = color;

	// Color 7
	color.red = 0;
	color.green = 0;
	color.blue = 0;
	colorList[6] = color;

	// Color 8
	color.red = 0;
	color.green = 0;
	color.blue = 0;
	colorList[7] = color;

	// Color 9
	color.red = 0;
	color.green = 0;
	color.blue = 0;
	colorList[8] = color;
}
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/


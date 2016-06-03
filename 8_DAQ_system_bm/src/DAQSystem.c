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

#include "DAQSystem.h"       /* <= own header */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "../../drivers_bm/inc/led.h"
//#include "../../drivers_bm/inc/ledrgb.h"
#include "../../drivers_bm/inc/timer.h"
#include "../../drivers_bm/inc/buttons.h"
#include "../../drivers_bm/inc/digital2analog.h"
#include "../../drivers_bm/inc/ADC.h"
#include "../../drivers_bm/inc/uart.h"
#include "squaresignal.h"

/*==================[macros and definitions]=================================*/
#define TIMER_DELAY_MS	1
#define BTTN_DELAY_MS	15
#define RGBLED_DELAY_MS	20
#define WRITE_SERIAL_DELAY_MS	250
#define READ_SERIAL_DELAY_MS	50
#define OUTPUT_SIGNAL_DELAY_MS	100

#define TOGGLE_CYCLES 3
#define MAXCOUNTER 10000000
#define CYCLESPRESSED	4			// Cantidad de ciclos antes de tomar un botón como presionado

#define OFF_UP_CMD		0
#define OFF_DWN_CMD		1
#define GAIN_UP_CMD		2
#define GAIN_DWN_CMD	3

// ADC definitions
#define ADC_INPUT_CHANNEL ADC_CHANNEL_1

// UART communication
#define BAUD_RATE	UART_BR_115200

/*==================[internal data declaration]==============================*/
int32_t	counter = MAXCOUNTER;

// Sistema de comandos por teclas
uint8_t commandList[4];					// Vector para vincular comandos con teclas
bool bttnStates[4] = {true, true, true, true};
uint8_t bttnCyclePressedCounters[4] = {0,0,0,0};
uint32_t	buttonsCounter = 0;				// Contador para ejecutar las funciones de comandos de teclas
uint32_t	buttonsCycles = (uint32_t)((float)(BTTN_DELAY_MS)/(float)(TIMER_DELAY_MS));				// Cantidad de ciclos que equivalen al tiempo en ms para ejecutar los comandos de las teclas

// Cambio de colores del led RGB
//static rgbColor	ledColor;
//static rgbColor	colorList[16];
//static uint8_t 	currentColor = 0;
//static uint8_t	ledUpdatecycles = (uint8_t)((float)(RGBLED_DELAY_MS)/(float)(TIMER_DELAY_MS));				// Cantidad de ciclos que equivalen al tiempo en ms para ejecutar los comandos de las teclas

// Cambio de amplitud y periodo de la señal de control
static double controlSignalAmplitud;
static double controlSignalPeriod;
static double controlSignalOffset;
static uint32_t controlSignalSamplesPerPeriod;

// Lectura del ADC
static uint32_t signalDigitalValue;
static volatile bool newSignalValue = false;
static double adcMaxVoltage;
static uint8_t adcBitResolution;
static uint16_t adcMaxDigitalCount;

static double maxSignalValue = 3.0;
static double minSignalValue = 0.0;
static double signalEpsilon = 0.05;

// Salida de datos por el UART
static uint32_t uartCurrentSendingData;
static bool sendDataToSerial = false;
static uint8_t lastByteSend = 0;
static uint32_t writeSerialCounter = 0;
static uint32_t	writeSerialCycles = (uint32_t)((float)(WRITE_SERIAL_DELAY_MS)/(float)(TIMER_DELAY_MS));				// Cantidad de ciclos que equivalen al tiempo en ms para ejecutar los comandos de las teclas
static uint8_t	signalCharArray[4];

// Entrada de datos por el UART
static uint8_t	receivedByte;
static uint8_t	readSerialCounter = 0;
static uint32_t	readSerialCycles = (uint32_t)((float)(READ_SERIAL_DELAY_MS)/(float)(TIMER_DELAY_MS));

/*==================[internal functions declaration]=========================*/

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
static void ProcessSignalValueToArray_uint32(uint32_t value);

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
//	RgbLedInit();
	InitButtons();
	DACInit();
	ADCInit(ADC_INPUT_CHANNEL);
	UARTInit(BAUD_RATE);

	InitTimer(RIT_TIMER);					// Configurar el timer
	SetDelay(RIT_TIMER,TIMER_DELAY_MS);		// Establecer un delay de 250 milisegundos en el timer

	// Configurar lista de comandos
	commandList[GAIN_UP_CMD] 	= BTTN_1;
	commandList[GAIN_DWN_CMD] 	= BTTN_2;
	commandList[OFF_UP_CMD] 	= BTTN_3;
	commandList[OFF_DWN_CMD] 	= BTTN_4;

	// Lista de leds
	ledList[0]	= RGBLED;
	ledList[1]	= YELOWLED;
	ledList[2]	= REDLED;
	ledList[3]	= GREENLED;

	currentLed = 1;

	// Prender el led rgb antes de empezar el encendido secuencial
	//currentLed = REDLED;
	LedOn(RGBLED);

	// Señal diente de control cuadra a 10 Hz (100ms)
	controlSignalAmplitud = 1.0;
	controlSignalPeriod = OUTPUT_SIGNAL_DELAY_MS;
	controlSignalOffset = 1.0;
	controlSignalSamplesPerPeriod = (uint32_t)(controlSignalPeriod / (double)TIMER_DELAY_MS );

	SignalSquareInit();
	SignalSquareSetAmplitude(controlSignalAmplitud);
	SignalSquareSetPeriod(controlSignalPeriod);
	SignalSquareSetOffset(controlSignalOffset);
	SignalSquareSetSamples(controlSignalSamplesPerPeriod);

	// Leer configuración del ADC
	adcMaxVoltage = ADCGetBitResolution();
	adcBitResolution = ADCGetBitResolution();
	adcMaxDigitalCount = (uint16_t)(pow(2,adcBitResolution)) - 1;

	signalEpsilon = 0.01 * maxSignalValue;		// epsilon para detectar el valor máximo y mínimo
	signalDigitalValue = 0;

	StartTimer(RIT_TIMER);		// Iniciar el timer

	// Loop principal
	for(;;){

		// Detectar el accionamiento de las teclas y ejecutar los comandos de las teclas
		if( buttonsCounter >= buttonsCycles ){
			buttonsCounter = 0;

			// NEXT COMMAND
			if( bttnStates[GAIN_UP_CMD] == false ){
				if( bttnCyclePressedCounters[GAIN_UP_CMD] < CYCLESPRESSED ){
					// Esperar a que el botón se mantenga apretado por más tiempo
					bttnCyclePressedCounters[GAIN_UP_CMD] = bttnCyclePressedCounters[GAIN_UP_CMD] + 1;
				}
				else{
					// El botón ya estuvo presionado la cantidad de ciclos predefinidos.
					// Realizar acción
					bttnCyclePressedCounters[GAIN_UP_CMD] = 0;

					maxSignalValue -= 0.05;
				}
			}
			else{
				bttnCyclePressedCounters[GAIN_UP_CMD] = 0;
			}

			// PREV COMMAND
			if( bttnStates[GAIN_DWN_CMD] == false ){
				if( bttnCyclePressedCounters[GAIN_DWN_CMD] < CYCLESPRESSED ){
					// Esperar a que el botón se mantenga apretado por más tiempo
					bttnCyclePressedCounters[GAIN_DWN_CMD] = bttnCyclePressedCounters[GAIN_DWN_CMD] + 1;
				}
				else{
					// El botón ya estuvo presionado la cantidad de ciclos predefinidos.
					// Realizar acción
					bttnCyclePressedCounters[GAIN_DWN_CMD] = 0;

					maxSignalValue += 0.05;

				}
			}
			else{
				bttnCyclePressedCounters[GAIN_DWN_CMD] = 0;
			}

			// UP COMMAND
			if( bttnStates[OFF_UP_CMD] == false ){
				if( bttnCyclePressedCounters[OFF_UP_CMD] < CYCLESPRESSED ){
					// Esperar a que el botón se mantenga apretado por más tiempo
					bttnCyclePressedCounters[OFF_UP_CMD] = bttnCyclePressedCounters[OFF_UP_CMD] + 1;
				}
				else{
					// El botón ya estuvo presionado la cantidad de ciclos predefinidos.
					// Realizar acción
					bttnCyclePressedCounters[OFF_UP_CMD] = 0;

					minSignalValue -= 0.05;

				}
			}
			else{
				bttnCyclePressedCounters[OFF_UP_CMD] = 0;
			}

			// DOWN COMMAND
			if( bttnStates[OFF_DWN_CMD] == false ){
				if( bttnCyclePressedCounters[OFF_DWN_CMD] < CYCLESPRESSED ){
					// Esperar a que el botón se mantenga apretado por más tiempo
					bttnCyclePressedCounters[OFF_DWN_CMD] = bttnCyclePressedCounters[OFF_DWN_CMD] + 1;
				}
				else{
					// El botón ya estuvo presionado la cantidad de ciclos predefinidos.
					// Realizar acción
					bttnCyclePressedCounters[OFF_DWN_CMD] = 0;

					minSignalValue += 0.05;
				}
			}
			else{
				bttnCyclePressedCounters[OFF_DWN_CMD] = 0;
			}

		}

		if( newSignalValue ){
			double signalValue = (double)signalDigitalValue / (double)adcMaxDigitalCount * adcMaxVoltage;
			newSignalValue = false;

			// Salida por el puerto serie
			if( sendDataToSerial == false ){
				uartCurrentSendingData = signalDigitalValue;
				ProcessSignalValueToArray_uint32(signalDigitalValue);
				sendDataToSerial = true;
			}


			// Valor máximo
			if( signalValue > (maxSignalValue - signalEpsilon) ){
				if( signalValue < (maxSignalValue + signalEpsilon) ){
					//LedOn(REDLED);
				}
			}
			else{
				//LedOff(REDLED);
			}

			// Valor mínimo
			if( signalValue < (minSignalValue + signalEpsilon) ){
				if( signalValue > (minSignalValue - signalEpsilon) ){
					//LedOn(GREENLED);
				}
			}
			else{
				//LedOff(GREENLED);
			}

			//signalValue
		}

		// Receive data throw serial port
		if( readSerialCounter >= readSerialCycles ){
			readSerialCounter = 0;

			// Leer datos del puerto serie;
			receivedByte = UARTReadByte();

			switch(receivedByte){
			case 'r':
				ToggleLed(REDLED);
				break;
			case 'v':
				ToggleLed(GREENLED);
				break;
			case 'a':
				ToggleLed(YELOWLED);
				break;
			default:
				break;
			}
		}

		// Send data throw Serial port
		if( writeSerialCounter >= writeSerialCycles ){

			if( sendDataToSerial == true ){
				uint8_t currentByte;

				// Transformar a caracter
				currentByte = signalCharArray[lastByteSend] + '0';

				if( UARTReady() == true ){
					UARTSendByte(currentByte);

					if( lastByteSend == (sizeof(signalCharArray) - 1) ){
						sendDataToSerial = false;
						lastByteSend = 0;

						UARTSendByte('\r');

						writeSerialCounter = 0;		// Resetear el ciclo del contador
					}
					else if( lastByteSend <= (sizeof(signalCharArray) - 1) ){
						++lastByteSend;
					}

				}
			}
		}
	}

	return 0;
}

void ISR_RIT(void){

	// actualizar contadores de las distintas funciones
	++timeCounter;
	++buttonsCounter;
	++writeSerialCounter;
	++readSerialCounter;

	// Leer el estado actual de los botones
	bttnStates[GAIN_UP_CMD] = ReadButton(commandList[GAIN_UP_CMD]);
	bttnStates[GAIN_DWN_CMD] = ReadButton(commandList[GAIN_DWN_CMD]);
	bttnStates[OFF_UP_CMD] = ReadButton(commandList[OFF_UP_CMD]);
	bttnStates[OFF_DWN_CMD] = ReadButton(commandList[OFF_DWN_CMD]);

	// Salida de señal cuadrada por el DAC
	double controlSignalValue = SignalSquareGetNextValue();
	uint32_t controlSignalDigitalValue = (uint32_t)(controlSignalValue/(double)DAC_MAX_OUTPUT*(double)(DAC_BIT_RESOLUTION-1));
	DACWrite(controlSignalDigitalValue);

	// Lectura de la señal por el ADC
	signalDigitalValue = ADCReadBlocking(ADC_INPUT_CHANNEL);
	newSignalValue = true;

	CLearTimer(RIT_TIMER);	// Liberar la bandera del timer
}

static void ProcessSignalValueToArray_uint32(uint32_t value){
	uint8_t i;
	uint32_t dividendo = value;
	uint32_t divisor;
	uint32_t resto;

	for( i = 0 ; i < 4 ; ++i ){
		divisor = (uint32_t)pow(10,3-i);
		signalCharArray[i] = (uint8_t)((double)dividendo / divisor);
		resto = dividendo % divisor;
		dividendo = resto;
	}

}
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/


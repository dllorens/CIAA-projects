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
#define WRITE_SERIAL_DELAY_MS	1000
#define READ_SERIAL_DELAY_MS	50
#define OUTPUT_SIGNAL_DELAY_MS	100
#define ADC_DELAY_MS			10

#define TOGGLE_CYCLES 3
#define MAXCOUNTER 10000000
#define CYCLESPRESSED	6			// Cantidad de ciclos antes de tomar un botón como presionado

#define OFF_UP_CMD		0
#define OFF_DWN_CMD		1
#define GAIN_UP_CMD		2
#define GAIN_DWN_CMD	3

// ADC definitions
#define ADC_INPUT_CHANNEL ADC_CHANNEL_1

// UART communication
#define BAUD_RATE	UART_BR_115200
#define MSG_1		0
#define MSG_2		1
#define MSG_3		2
#define MSG_4		3
#define MSG_COUNT	4					// Cantidad de mensages a enviar

// Signal processing defines
#define MAX_GAIN_COUNT	5
#define MAX_OFF_COUNT	5

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
uint32_t	adcCounter = 0;				// Contador para ejecutar las funciones de comandos de teclas
uint32_t	adcCycles = (uint32_t)((float)(ADC_DELAY_MS)/(float)(TIMER_DELAY_MS));

// Procesamiento de la señal adquirida
static double gainsList[MAX_GAIN_COUNT] = {0.8f,0.9f,1.0f,1.1f,1.2f};
static double offsetList[MAX_OFF_COUNT] = {-0.2f,-0.1f,0.0f,0.1f,0.2f};
static uint8_t currentGainIndex = 2;
static uint8_t currentOffsetIndex = 2;
static double currentGain;
static double currentOffset;
static double maxSignalValue = 0.0;
static double minSignalValue = 0.0;
static double meanSignalValue = 0.0;
static uint32_t	processedSamples = 0;
static uint32_t samplesWindowSize = (uint32_t)(1.0/(float)(ADC_DELAY_MS)*1000.0);

// Salida de datos por el UART
static uint32_t uartCurrentSendingData;
static bool sendDataToSerial = false;
static uint8_t lastByteSend = 0;
static uint32_t writeSerialCounter = 0;
static uint32_t	writeSerialCycles = (uint32_t)((float)(WRITE_SERIAL_DELAY_MS)/(float)(TIMER_DELAY_MS));				// Cantidad de ciclos que equivalen al tiempo en ms para ejecutar los comandos de las teclas
static uint8_t	signalCharArray[4];
static uint8_t	valueCharArray_uint8[4];
static char message_1[] = "Gain:";
static char message_2[] = "Offset:";
static char message_3[] = "Min:";
static char message_4[] = "Max:";
static char currentMsg[32];
static uint8_t currentSendingMsgIndex = 0;
static uint8_t currentMsgSize = 0;
static uint8_t messagesSent = 0;			// Cantidad de mensajes enviados
static bool finishedSendMessage = true;

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
static void ProcessValueToArray_uint32(uint32_t value);
static void ProcessValueToArray_double(double value);
static uint8_t BuildMessageToSend(char str1[],uint8_t str2[]);

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
	adcMaxVoltage = ADCGetMaxVoltage();
	adcBitResolution = ADCGetBitResolution();
	adcMaxDigitalCount = (uint16_t)(pow(2,adcBitResolution)) - 1;

	// Configuración del procesamiento de señal.
	currentGain = gainsList[currentGainIndex];
	currentOffset = offsetList[currentOffsetIndex];

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

					++currentGainIndex;
					if(currentGainIndex >= MAX_GAIN_COUNT){
						currentGainIndex = (uint8_t)(MAX_GAIN_COUNT - 1);
					}

					currentGain = gainsList[currentGainIndex];
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

					if(currentGainIndex > 1){
						--currentGainIndex;
					}
					else{
						currentGainIndex = 0;
					}

					currentGain = gainsList[currentGainIndex];
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

					++currentOffsetIndex;
					if( currentOffsetIndex >= MAX_OFF_COUNT){
						currentOffsetIndex = (uint8_t)(MAX_OFF_COUNT - 1);
					}

					currentOffset = offsetList[currentOffsetIndex];
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

					if(currentOffsetIndex > 0){
						--currentOffsetIndex;
					}
					else{
						currentOffsetIndex = 0;
					}

					currentOffset = offsetList[currentOffsetIndex];
				}
			}
			else{
				bttnCyclePressedCounters[OFF_DWN_CMD] = 0;
			}

		}

		// Procesar el ADC
		if( adcCounter >= adcCycles ){
			adcCounter = 0;

			// Lectura de la señal por el ADC
			signalDigitalValue = ADCReadBlocking(ADC_INPUT_CHANNEL);

			//newSignalValue = true;

			// Convertir el valor leído a voltaje
			double signalValue = (double)signalDigitalValue / (double)adcMaxDigitalCount * adcMaxVoltage;
			//newSignalValue = false;

			// Procesar el valor de la señal con la ganancia y el offset actual
			double processedSignalValue = currentGain * signalValue + currentOffset;


			// Calcular el promedio
			if( processedSamples == 0 ){
				meanSignalValue = processedSignalValue;
				maxSignalValue = processedSignalValue;
				minSignalValue = processedSignalValue;
			}
			else{
				meanSignalValue += processedSignalValue;
			}

			// Valor máximo
			if( maxSignalValue < processedSignalValue ){
				maxSignalValue = processedSignalValue;
			}
			else{
				// No hacer nada con el máximo de la señal
			}

			// Valor mínimo
			if( minSignalValue > processedSignalValue ){
				minSignalValue = processedSignalValue;
			}
			else{
				// No hacer nada con la señal
			}

			++processedSamples;

			// Controlar la cantidad de muestras procesadas
			if( processedSamples >= samplesWindowSize ){
				// Reiniciar el mínimo y el máximo
				maxSignalValue = meanSignalValue / processedSamples;
				minSignalValue = maxSignalValue / processedSamples;

				// Reiniciar el contador de muestras
				processedSamples = 0;
			}

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

			// Construir el mensaje a enviar
			if( finishedSendMessage == true ){
				switch(currentSendingMsgIndex){
				case MSG_1:
					ProcessValueToArray_uint32((uint32_t)(10*currentGain));
					currentMsgSize = BuildMessageToSend(message_1,valueCharArray_uint8);
					finishedSendMessage = false;
					break;
				case MSG_2:
					ProcessValueToArray_uint32((uint32_t)(1000*currentOffset));
					currentMsgSize = BuildMessageToSend(message_2,valueCharArray_uint8);
					finishedSendMessage = false;
					break;
				case MSG_3:
					ProcessValueToArray_uint32((uint32_t)(100*minSignalValue));
					currentMsgSize = BuildMessageToSend(message_3,valueCharArray_uint8);
					finishedSendMessage = false;
					break;
				case MSG_4:
					ProcessValueToArray_uint32((uint32_t)(100*maxSignalValue));
					currentMsgSize = BuildMessageToSend(message_4,valueCharArray_uint8);
					finishedSendMessage = false;
					break;
				default:
					break;
				}

				++currentSendingMsgIndex;
				if( currentSendingMsgIndex >= MSG_COUNT ){
					currentSendingMsgIndex = 0;
				}
			}

			//if( sendDataToSerial == true ){
				uint8_t currentByte;

				// Transformar valor de la señal a caracter
				//currentByte = signalCharArray[lastByteSend] + '0';
				currentByte = currentMsg[lastByteSend];

				// Si está libre el UART enviar el byte si no esperar al próximo ciclo de ejecución
				if( UARTReady() == true ){
					UARTSendByte(currentByte);

					if( lastByteSend == currentMsgSize ){
						finishedSendMessage = true;
						lastByteSend = 0;
						++messagesSent;

						UARTSendByte('\n');
						UARTSendByte('\r');
					}
					else if( lastByteSend <= currentMsgSize ){
						++lastByteSend;
					}

					if( messagesSent == MSG_COUNT ){
						messagesSent = 0;
						sendDataToSerial = false;
						writeSerialCounter = 0;		// Resetear el ciclo del contador
						//UARTSendByte('\r');
						ToggleLed(GREENLED);
					}

				}
			//}
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
	++adcCounter;

	// Leer el estado actual de los botones
	bttnStates[GAIN_UP_CMD] = ReadButton(commandList[GAIN_UP_CMD]);
	bttnStates[GAIN_DWN_CMD] = ReadButton(commandList[GAIN_DWN_CMD]);
	bttnStates[OFF_UP_CMD] = ReadButton(commandList[OFF_UP_CMD]);
	bttnStates[OFF_DWN_CMD] = ReadButton(commandList[OFF_DWN_CMD]);

	// Salida de señal cuadrada por el DAC
	double controlSignalValue = SignalSquareGetNextValue();
	uint32_t controlSignalDigitalValue = (uint32_t)(controlSignalValue/(double)DAC_MAX_OUTPUT*(double)(DAC_BIT_RESOLUTION-1));
	DACWrite(controlSignalDigitalValue);

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

static void ProcessValueToArray_uint32(uint32_t value){
	uint8_t i;
	uint32_t dividendo = value;
	uint32_t divisor;
	uint32_t resto;

	for( i = 0 ; i < 4 ; ++i ){
		divisor = (uint32_t)pow(10,3-i);
		valueCharArray_uint8[i] = (uint8_t)((double)dividendo / divisor);
		resto = dividendo % divisor;
		dividendo = resto;
	}
}

static void ProcessValueToArray_double(double value){
	uint8_t i;
	uint32_t dividendo = value;
	uint32_t divisor;
	uint32_t resto;

	for( i = 0 ; i < 4 ; ++i ){
		divisor = (uint32_t)pow(10,3-i);
		valueCharArray_uint8[i] = (uint8_t)((double)dividendo / divisor);
		resto = dividendo % divisor;
		dividendo = resto;
	}
}

static uint8_t BuildMessageToSend(char str1[],uint8_t value[]){
	uint8_t i;
	uint8_t msgSize = 0;

	for( i = 0 ; i <= sizeof(str1) ; ++i ){
		currentMsg[msgSize] = (*str1);
		++str1;
		++msgSize;
	}

	currentMsg[msgSize] = ' ';
	++msgSize;

	for( i = 0 ; i < sizeof(value) ; ++i ){
		currentMsg[msgSize] = (*value) + '0';		// Pasar a numeros imprimibles
		++(value);
		++msgSize;
	}

	return msgSize;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/


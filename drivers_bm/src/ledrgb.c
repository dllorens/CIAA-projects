/*
 * ledrgb.c
 *
 *  Created on: 1/6/2016
 *      Author: GCS-IUA
 */

#include "ledrgb.h"


static rgbColor currentColor;
static uint8_t	cyclesPerPeriod;		// Cantidad de ciclos en unidades de tiempo base por periodo
static uint8_t	timeBaseCounter;		// Contador para el tiempo base

static uint8_t	redIntensityCycles;		// Cantidad de ciclos que debe estar prendido el led rojo prop al valor deseado
static uint8_t	greenIntensityCycles;	// Cantidad de ciclos que debe estar prendido el led rojo prop al valor deseado
static uint8_t	blueIntensityCycles;	// Cantidad de ciclos que debe estar prendido el led rojo prop al valor deseado

static uint8_t	ellapsedRedCycles = 0;
static uint8_t	ellapsedGreenCycles = 0;
static uint8_t	ellapsedBlueCycles = 0;


static void ScaleColorToCycles(void);

void RgbLedInit(void){


	currentColor.red = 255;
	currentColor.green = 255;
	currentColor.blue = 255;

	cyclesPerPeriod = 20;

	ScaleColorToCycles();

	// Contadores para prender y apagar los leds
	timeBaseCounter = cyclesPerPeriod;		// Iniciar el contador base igual al periodo
	ellapsedRedCycles = 0;
	ellapsedGreenCycles = 0;
	ellapsedBlueCycles = 0;

	// Inicializar el led rbg y apagarlo
	InitLed();
	LedOff(RGBLED_RED);
	LedOff(RGBLED_GREEN);
	LedOff(RGBLED_BLUE);
}

void RgbLedSetColor_1(uint8_t red, uint8_t green, uint8_t blue){
	currentColor.red = red;
	currentColor.green = green;
	currentColor.blue = blue;

	ScaleColorToCycles();
}

void RgbLedSetColor_2(rgbColor color){
	currentColor = color;
	ScaleColorToCycles();
}

void RgbLedSetPeriod(uint8_t _cyclesPerPeriod){
	cyclesPerPeriod = _cyclesPerPeriod;
	ScaleColorToCycles();
}

/*
 * Esta función se debe llamar a la máxima velocidad
 * para poder tener la base de tiempo
 */
void RgbLedUpdate(void){
	// Supongo que paso una unidad de tiempo de la base de tiempo.
	++timeBaseCounter;

	if( timeBaseCounter >= cyclesPerPeriod ){
		// Prender todos los leds

		if( redIntensityCycles > 0 ){
			LedOn(RGBLED_RED);
		}

		if( greenIntensityCycles > 0 ){
			LedOn(RGBLED_GREEN);
		}

		if( blueIntensityCycles > 0 ){
			LedOn(RGBLED_BLUE);
		}

		// Reiniciar el contador
		timeBaseCounter = 0;
		ellapsedRedCycles = 0;
		ellapsedGreenCycles = 0;
		ellapsedBlueCycles = 0;
	}
	else{

		// Controlar el estado del led Verde
		if( ellapsedRedCycles < redIntensityCycles ){
			++ellapsedRedCycles;
		}
		else{
			LedOff(RGBLED_RED);
		}

		// Controlar el estado del led verde
		if( ellapsedGreenCycles < greenIntensityCycles ){
			++ellapsedGreenCycles;
		}
		else{
			LedOff(RGBLED_GREEN);
		}

		// Controlar el estado del led Azul
		if( ellapsedBlueCycles < blueIntensityCycles ){
			++ellapsedBlueCycles;
		}
		else{
			LedOff(RGBLED_BLUE);
		}
	}
}

static void ScaleColorToCycles(void){
	redIntensityCycles = (uint8_t) ((float)(currentColor.red)/255.0f*cyclesPerPeriod);
	greenIntensityCycles = (uint8_t) ((float)(currentColor.green)/255.0f*cyclesPerPeriod);
	blueIntensityCycles = (uint8_t) ((float)(currentColor.blue)/255.0f*cyclesPerPeriod);
}

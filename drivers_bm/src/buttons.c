/*
 * keypad.c
 *
 *  Created on: 31/5/2016
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

#include "buttons.h"

#define PIN_GROUP_1		1
#define PIN_NUMBER_0	0
#define PIN_NUMBER_1	1
#define PIN_NUMBER_2	2
#define PIN_NUMBER_6	6

#define INPUT	0

#define BTTN_1_PORT	0
#define BTTN_2_PORT	0
#define BTTN_3_PORT	0
#define BTTN_4_PORT	1

#define BTTN_1_BIT	1<<4
#define BTTN_2_BIT	1<<8
#define BTTN_3_BIT	1<<9
#define BTTN_4_BIT	1<<9

#define BTTN_1_PIN	4
#define BTTN_2_PIN	8
#define BTTN_3_PIN	9
#define BTTN_4_PIN	9


void InitButtons(void){

	Chip_GPIO_Init(LPC_GPIO_PORT);

	Chip_SCU_PinMux(PIN_GROUP_1,PIN_NUMBER_0,MD_PUP|MD_EZI|MD_ZI,FUNC0);	/* mapea P1 0 en GPIO 0[4], SW1 */
	Chip_SCU_PinMux(PIN_GROUP_1,PIN_NUMBER_1,MD_PUP|MD_EZI|MD_ZI,FUNC0);	/* mapea P1 1 en GPIO 0[8], SW2 */
	Chip_SCU_PinMux(PIN_GROUP_1,PIN_NUMBER_2,MD_PUP|MD_EZI|MD_ZI,FUNC0);	/* mapea P1 2 en GPIO 0[9], SW3 */
	Chip_SCU_PinMux(PIN_GROUP_1,PIN_NUMBER_6,MD_PUP|MD_EZI|MD_ZI,FUNC0);	/* mapea P1 6 en GPIO 1[9], SW4 */

	// Configurar los botones para entrada
	Chip_GPIO_SetDir(LPC_GPIO_PORT, BTTN_1_PORT , BTTN_1_BIT ,INPUT);
	Chip_GPIO_SetDir(LPC_GPIO_PORT, BTTN_2_PORT , BTTN_2_BIT ,INPUT);
	Chip_GPIO_SetDir(LPC_GPIO_PORT, BTTN_3_PORT , BTTN_3_BIT ,INPUT);
	Chip_GPIO_SetDir(LPC_GPIO_PORT, BTTN_4_PORT , BTTN_4_BIT ,INPUT);
}

bool ReadButton(uint8_t bttn){
	switch(bttn){
	case BTTN_1:
		return Chip_GPIO_GetPinState(LPC_GPIO_PORT,BTTN_1_PORT,BTTN_1_PIN);
		break;
	case BTTN_2:
		return Chip_GPIO_GetPinState(LPC_GPIO_PORT,BTTN_2_PORT,BTTN_2_PIN);
		break;
	case BTTN_3:
		return Chip_GPIO_GetPinState(LPC_GPIO_PORT,BTTN_3_PORT,BTTN_3_PIN);
		break;
	case BTTN_4:
		return Chip_GPIO_GetPinState(LPC_GPIO_PORT,BTTN_4_PORT,BTTN_4_PIN);
		break;
	default:
		return false;
	}
}

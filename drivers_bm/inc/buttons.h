/*
 * keypad.h
 *
 *  Created on: 31/5/2016
 *      Author: GCS-IUA
 */

#ifndef BUTTONS_H_
#define BUTTONS_H_

#include "stdint.h"
#include "stdbool.h"

#define lpc4337            1
#define mk60fx512vlq15     2

#define BTTN_1	0
#define BTTN_2	1
#define BTTN_3	2
#define BTTN_4	3

#define BTTN_COUNT	4		// cantidad de botones en la placa

#define BTTN_PRESSED	1
#define BTTN_RELEASED	0

void InitButtons(void);
bool ReadButton(uint8_t bttn);

#endif /* BUTTONS_H_ */

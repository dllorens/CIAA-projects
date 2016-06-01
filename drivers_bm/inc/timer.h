/*
 * timer.h
 *
 *  Created on: 31/5/2016
 *      Author: GCS-IUA
 */

#ifndef TIMER_H_
#define TIMER_H_

#include "stdint.h"

#define RIT_TIMER	0

void InitTimer(uint8_t timer);
void SetDelay(uint8_t timer, uint32_t milliseconds);
void StartTimer(uint8_t timer);
void CLearTimer(uint8_t timer);

#endif /* TIMER_H_ */

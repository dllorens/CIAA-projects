/*
 * uart.h
 *
 *  Created on: 2/6/2016
 *      Author: GCS-IUA
 */

#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include <stdbool.h>

#define lpc4337			1
#define mk60fx512vlq15	2

#define UART_BR_115200	0

void UARTInit(uint8_t baudRate);
void UARTSendByte(uint8_t byte);
bool UARTReady(void);
uint8_t UARTReadByte(void);

#endif /* UART_H_ */

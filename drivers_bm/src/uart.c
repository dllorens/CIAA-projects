/*
 * uart.c
 *
 *  Created on: 2/6/2016
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

#include "uart.h"

#define UART2_PORT		7
#define UART2_TX_PIN	1
#define UART2_RX_PIN	2

static uint32_t uartBaudRate;

static uint32_t GetBaudRateValue(uint8_t baud);

void UARTInit(uint8_t baudRate){
	Chip_SCU_PinMux(UART2_PORT,UART2_TX_PIN,MD_PDN,FUNC6);					// P7_1: UART2_TXD
	Chip_SCU_PinMux(UART2_PORT,UART2_RX_PIN,MD_PLN|MD_EZI|MD_ZI,FUNC6);		// P7_2: UART2_RXD

	uartBaudRate = GetBaudRateValue(baudRate);

	Chip_UART_Init(LPC_USART2);
	Chip_UART_SetBaud(LPC_USART2,uartBaudRate);
	Chip_UART_SetupFIFOS(LPC_USART2, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV0);
	Chip_UART_TXEnable(LPC_USART2);
}

void UARTSendByte(uint8_t byte){
	Chip_UART_SendByte(LPC_USART2,byte);
}

bool UARTReady(void){
	switch(Chip_UART_CheckBusy(LPC_USART2)){
	case RESET:
		return true;
		break;
	case SET:
		return false;
		break;
	default:
		return false;
		break;
	}
}

uint8_t UARTReadByte(void){
	uint8_t byte;
	byte = Chip_UART_ReadByte(LPC_USART2);
	return byte;
}

static uint32_t GetBaudRateValue(uint8_t baud){
	switch(baud){
	case UART_BR_115200:
		return 115200;
		break;
	default:
		return 115200;
		break;
	}
}

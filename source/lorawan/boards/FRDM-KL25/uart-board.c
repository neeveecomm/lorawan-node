/*!
 * \file      uart-board.c
 *
 * \brief     Target board UART driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include "utilities.h"
#include "board.h"
#include "uart-board.h"

/*!
 * Number of times the UartPutBuffer will try to send the buffer before
 * returning ERROR
 */
#define TX_BUFFER_RETRY_COUNT                       10

uint8_t RxData = 0;
uint8_t TxData = 0;

extern Uart_t Uart2;

void UartMcuInit( Uart_t *obj, UartId_t uartId, PinNames tx, PinNames rx )
{
}

void UartMcuConfig( Uart_t *obj, UartMode_t mode, uint32_t baudrate, WordLength_t wordLength, StopBits_t stopBits, Parity_t parity, FlowCtrl_t flowCtrl )
{
}

void UartMcuDeInit( Uart_t *obj )
{
}

uint8_t UartMcuPutChar( Uart_t *obj, uint8_t data )
{
	return 0; // OK
}

uint8_t UartMcuGetChar( Uart_t *obj, uint8_t *data )
{
	return 0; // OK
}

uint8_t UartMcuPutBuffer( Uart_t *obj, uint8_t *buffer, uint16_t size )
{
	return 0; // OK
}

uint8_t UartMcuGetBuffer( Uart_t *obj, uint8_t *buffer, uint16_t size, uint16_t *nbReadBytes )
{
    return 0; // OK
}

void USART2_IRQHandler( void )
{
}

/*
 * File:		uart.h
 * Purpose:	Provide common ColdFire uart routines for polled serial IO
 *
 * Notes:
 */

#ifndef __uart_H__
#define __uart_H__

#include "stdio.h"
/********************************************************************/


void uart_init (UART_Type * uartch, int sysclk, int baud);
char uart_getchar (UART_Type * channel);
void uart_putchar (UART_Type * channel, char ch);
int uart_getchar_present (UART_Type * channel);
void uart0_init (UARTLP_Type * uartch, int sysclk, int baud);
char uart0_getchar (UARTLP_Type * channel);
void uart0_putchar (UARTLP_Type * channel, char ch);
int uart0_getchar_present (UARTLP_Type * channel);

/********************************************************************/

#endif /* __uart_H__ */

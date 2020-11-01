/*
 * uart.h
 *
 * Created: 9/3/2018 12:23:05 PM
 *  Author: Sean
 */ 

#ifndef  UART_H_
#define UART_H_

#include <asf.h>

#define UART_SD_SERIAL				&USARTD0
#define UART_SD_SERIAL_BAUDRATE		115200
#define UART_SD_SERIAL_CHAR_LEN		USART_CHSIZE_8BIT_gc
#define UART_SD_SERIAL_PARITY		USART_PMODE_DISABLED_gc
#define UART_SD_SERIAL_STOP_BIT		true

#define UART_SD_TX_PIN				PIN3_bm
#define UART_SD_PORT				PORTD


#define UART_GPS_SERIAL				&USARTC0
#define UART_GPS_SERIAL_BAUDRATE	9600
#define UART_GPS_SERIAL_CHAR_LEN	USART_CHSIZE_8BIT_gc
#define UART_GPS_SERIAL_PARITY		USART_PMODE_DISABLED_gc
#define UART_GPS_SERIAL_STOP_BIT	true

#define UART_GPS_PORT				PORTC

void uart_sd_init(void);
void uart_gps_init(void);

#endif
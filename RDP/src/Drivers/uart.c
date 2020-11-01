/*
 * uart.c
 *
 * Created: 9/3/2018 12:22:41 PM
 *  Author: swidmier
 */ 

#include <asf.h>
#include "uart.h"

void uart_sd_init()
{
	sysclk_enable_peripheral_clock(UART_SD_SERIAL);	// enable the USART's clock
	// initialize a configuration struct with USART settings
	static usart_serial_options_t usart_config = {
		.baudrate	=	UART_SD_SERIAL_BAUDRATE,
		.charlength =	UART_SD_SERIAL_CHAR_LEN,
		.paritytype =	UART_SD_SERIAL_PARITY,
		.stopbits	=	UART_SD_SERIAL_STOP_BIT
	};
	
	UART_SD_PORT.DIR |= UART_SD_TX_PIN;	// set the USART transmit pin to output
	
	stdio_serial_init(UART_SD_SERIAL, &usart_config); // function maps the serial output to printf,
	
}

void uart_gps_init()
{
	sysclk_enable_peripheral_clock(UART_GPS_SERIAL);	// enable the USART's clock
	// initialize a configuration struct with USART settings
	static usart_serial_options_t usart_config = {
		.baudrate	=	UART_GPS_SERIAL_BAUDRATE,
		.charlength =	UART_GPS_SERIAL_CHAR_LEN,
		.paritytype =	UART_GPS_SERIAL_PARITY,
		.stopbits	=	UART_GPS_SERIAL_STOP_BIT
	};
	
	stdio_serial_init(UART_GPS_SERIAL, &usart_config);
	USARTC0.CTRLA |= 0x10;
	PMIC.CTRL |= PMIC_LOLVLEX_bm;
}
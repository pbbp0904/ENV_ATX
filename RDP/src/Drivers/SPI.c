/*
Editors:
Erik Korzon
*/

#include "drivers/SPI.h"
#include <asf.h>

void spi_init(SPI_t* targetspi, PORT_t* port)
{
	sysclk_enable_peripheral_clock(targetspi);	//Enable peripheral clock
	port->DIRSET |= 0b10110000;	//Initialize 
	port->DIR &= 0b10111111;
	port->OUT |= 0b10110000;
	targetspi->CTRL = 0b11010000;	//16 MHz
}

uint8_t spi_read(SPI_t* targetspi)
{
	delay_ms(1);
	targetspi->DATA = 0xFF;
	while(!(targetspi->STATUS >> 7));
	delay_ms(1);
	//printf("spi_read:%u\n",targetspi->DATA);	//Actually reading in all 1's. Next use this code on an xplained board and see if it's functional or if it's the sensor
	return targetspi->DATA;
}

void spi_write(SPI_t* targetspi, uint8_t data)
{
	delay_ms(1);
	targetspi->DATA = data;
	while(!(targetspi->STATUS >> 7));
	delay_ms(1);
}

void spi_select(ioport_pin_t pin)
{
	delay_ms(1);
	ioport_set_pin_low(pin);
	delay_ms(1);
}

void spi_deselect(ioport_pin_t pin)
{
	delay_ms(1);
	ioport_set_pin_high(pin);
	delay_ms(1);
}
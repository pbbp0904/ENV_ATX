/*
Editors:
Erik Korzon
*/

#ifndef SPI_H_
#define SPI_H_
#include <asf.h>

void spi_init(SPI_t* targetspi, PORT_t* port);
uint8_t spi_read(SPI_t* targetspi);
void spi_write(SPI_t* targetspi, uint8_t data);
void spi_select(ioport_pin_t pin);
void spi_deselect(ioport_pin_t pin);

#endif /* INCFILE1_H_ */
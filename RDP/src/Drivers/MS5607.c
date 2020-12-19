/*
Editors:
Erik Korzon
Myles Chatman
Christopher Helmerich
*/

#include "MS5607.h"
#include <stdio.h>
#include <inttypes.h>
#include <math.h>

char altSmooth [10];

uint16_t read16(SPI_t* targetspi)
{
	uint16_t result = ((uint16_t)spi_read(targetspi)) << 8;
	result += (uint16_t)spi_read(targetspi);
	return (uint16_t)result;
}

uint32_t read24(SPI_t* targetspi)
{
	uint32_t result = ((uint32_t)spi_read(targetspi)) << 16;
	result += (uint32_t)spi_read(targetspi) << 8;
	result += (uint32_t)spi_read(targetspi);
	return (uint32_t)result;
}

void pressure_reset(MS5607* sensor)
{
	spi_select(sensor->ss_pin);
	spi_write(MS5607_SPI, 0x1E);
	spi_deselect(sensor->ss_pin);
	delay_ms(5);
}

void pressure_init(MS5607* sensor)
{
	pressure_reset(sensor);
	
	spi_select(sensor->ss_pin);
	spi_write(MS5607_SPI, 0xA2);
	sensor->C1 = read16(MS5607_SPI);
	spi_deselect(sensor->ss_pin);
	
	spi_select(sensor->ss_pin);
	spi_write(MS5607_SPI, 0xA4);
	sensor->C2 = read16(MS5607_SPI);
	spi_deselect(sensor->ss_pin);
	
	spi_select(sensor->ss_pin);
	spi_write(MS5607_SPI, 0xA6);
	sensor->C3 = read16(MS5607_SPI);
	spi_deselect(sensor->ss_pin);
	
	spi_select(sensor->ss_pin);
	spi_write(MS5607_SPI, 0xA8);
	sensor->C4 = read16(MS5607_SPI);
	spi_deselect(sensor->ss_pin);
	
	spi_select(sensor->ss_pin);
	spi_write(MS5607_SPI, 0xAA);
	sensor->C5 = read16(MS5607_SPI);
	spi_deselect(sensor->ss_pin);
	
	spi_select(sensor->ss_pin);
	spi_write(MS5607_SPI, 0xAC);
	sensor->C6 = read16(MS5607_SPI);
	spi_deselect(sensor->ss_pin);
}

double readMS5607(MS5607* sensor)
{
	
	//Pressure Data (D1)//
	spi_select(sensor->ss_pin);
	spi_write(MS5607_SPI, 0x48); //Tell MS5607 to record raw pressure (D1)
	spi_deselect(sensor->ss_pin);
	delay_ms(10);
	
	spi_select(sensor->ss_pin);
	spi_write(MS5607_SPI, 0x0); //Tell MS5607 to read off data
	sensor->D1 = read24(MS5607_SPI);
	spi_deselect(sensor->ss_pin);
	
	//Temperature Data (D2)//
	spi_select(sensor->ss_pin);
	spi_write(MS5607_SPI, 0x58); //Tell MS5607 to record raw temperature (D2)
	spi_deselect(sensor->ss_pin);
	delay_ms(10);
	
	spi_select(sensor->ss_pin);
	spi_write(MS5607_SPI, 0x0); //Tell MS5607 to read off data
	sensor->D2 = read24(MS5607_SPI);
	spi_deselect(sensor->ss_pin);
	
	
	//Get usable data from raw data//
	int32_t altitude = 0;
	float dt = 0, T2 = 0;
	int32_t TEMP = 0, P = 0;
	int64_t OFF = 0, SENS = 0, OFF2 = 0, SENS2 = 0;
	
	dt = sensor->D2 - ((float)sensor->C5 * 256);
	OFF = ((float)sensor->C2 * 131072) + (((float)sensor->C4 * dt) / 64);
	SENS = ((float)sensor->C1 * 65536) + (((float)sensor->C3 * dt) / 128);
	TEMP = 2000 + (dt * ((float)sensor->C6 / 8388608));
	msTemp=TEMP;
	if (TEMP < 2000)
	{
		T2 = dt * dt / 2147483648;
		OFF2 = 61 * (pow((TEMP - 2000), 2.0)) / 16;
		SENS2 = 2 * (pow((TEMP - 2000), 2.0));
		if(TEMP < -1500)
		{
			OFF2 = OFF2 + 15 * (pow((TEMP + 1500), 2));
			SENS2 = SENS2 + 8 * (pow((TEMP + 1500), 2));
		}
		TEMP = TEMP - T2;
		OFF = OFF - OFF2;
		SENS = SENS - SENS2;
	}

	P = ((sensor->D1 * SENS / 2097152) - OFF) / 32768; //in pascals
	msPressure = P;
	//altitude = (TEMP / -0.0065) * (pow ( ((float)P / 101325), 0.190198216) - 1);
	altitude = (1- pow(((float)P/101325),0.190284)) * 44307.69;

	//printf("temp:%ld\tpres:%ld\taltitude:%ld\n", TEMP, P, altitude);
	//printf("D1: %lu D2: %lu P: %lu \n",sensor->D1,sensor->D2,P);
	//printf("Pitot Tube MS5607 Calibration Constants:\nC1:%u\nC2:%u\nC3:%u\nC4:%u\nC5:%u\nC6:%u\n",sensor->C1,sensor->C2,sensor->C3,sensor->C4,sensor->C5,sensor->C6);

	return msPressure;
}

void updateAltitude(void)
{
	alt = readMS5607(&static_sensor); 
	smoothedAlt = (alpha * alt) + (1-alpha) * lastSmoothedAlt; 
	lastSmoothedAlt = smoothedAlt; 
	currentAltitude = smoothedAlt - groundAlt;
}

void getPitotSpeed(void)
{
	staticPressure = msPressure;
	staticTemp = msTemp;
	totalPressure = (int32_t)readMS5607(&pitot_sensor);
	delayTime+=65;
	totalPressure = msPressure;
	pitotSpeed = (int32_t)100*sqrt((2*(abs(totalPressure-staticPressure-speedCompensationConstant)))/1.225);
	if(pitotSpeed>200000||pitotSpeed<0)
	{
		pitotSpeed=0;
	}
}
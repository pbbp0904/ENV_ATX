/*
Editors:
Erik Korzon
*/


#ifndef MS5607_H_
#define MS5607_H_

#include <asf.h>
#include "drivers/SPI.h"

#define MS5607_SPI &SPID //Changed from &SPID for the breakout board test

typedef struct MS5607
{
	//Calibration Stuff//
	ioport_pin_t ss_pin;	//The MS5607's Slave Select pin
	uint16_t C1;			//6 calibration constants 
	uint16_t C2;
	uint16_t C3;
	uint16_t C4;
	uint16_t C5;
	uint16_t C6;
	//Measurement Stuff//
	uint32_t D1;	//Raw pressure
	uint32_t D2;	//Raw temperature
	int32_t temperature;	//Print with %ld
	int64_t pressure;	//Print with %ld
} MS5607;

int32_t msPressure;
int32_t staticPressure;
int32_t totalPressure;
uint32_t staticTemp;
uint32_t msTemp;
int32_t pitotSpeed;
uint16_t read16(SPI_t* targetspi);
uint32_t read24(SPI_t* targetspi);

extern int16_t alt;
extern MS5607 static_sensor;
extern MS5607 pitot_sensor;
extern uint16_t delayTime;
extern int16_t smoothedAlt;
extern float alpha;
extern int16_t lastSmoothedAlt;
extern int16_t currentAltitude;
extern int16_t groundAlt;
extern int16_t speedCompensationConstant;

void pressure_reset(MS5607* sensor);
void pressure_init(MS5607* sensor);
double readMS5607(MS5607* sensor);
void updateAltitude(void);
void getPitotSpeed(void);

#endif /* MS5607_H_ */
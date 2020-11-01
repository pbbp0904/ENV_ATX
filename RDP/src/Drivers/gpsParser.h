/*
Editors:
Myles Chatman
Christopher Helmerich
*/


#ifndef GPSPARSER_H_
#define GPSPARSER_H_

//ALL GPS GLOBAL VARIABLES
char latitude [30];
char longitude [30];
char satNumber [30];
char readAlt [30];
char kphSpeed [30];
double kphSpeedDouble;
int16_t gpsSpeed;
char date [30];
char dummy [1000];
char gpsString [30];


extern StringRing *gps;

void parseGPS(void);

ISR(USARTC0_RXC_vect);

#endif /* GPSPARSER_H_ */
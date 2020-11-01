/*
Editors:
Myles Chatman
Christopher Helmerich
*/

#include <asf.h>
#include <string.h>
#include <stdio.h>
#include <nvm.h>
#include <math.h>

#include "Drivers/ctl_stringring.h"
#include "Drivers/MS5607.h"
#include "Drivers/gpsParser.h"

 void parseGPS(void)
 {
	int commaCount;

	//Initializing USART(USARTC0)
	// Set the TxD pin high
	PORTC.OUTSET = PIN3_bm;     // set PORTC OUT register bit 3 to 1

	// Set the TxD pin as an output // set PORTC DIR register bit 3 to 1
	PORTC.DIRSET = PIN3_bm;

	// Set the Baud rate
	USARTC0_BAUDCTRLB = 0b11001100;     //Just to be sure that BSCALE is 0        was 0x40
	USARTC0_BAUDCTRLA = 0b11110101; // 207 (prescalar for 9600 baudrate)        was 12

	// Set the Frame Format
	USARTC0.CTRLC = 0x03;      // async, no parity, 8 bit data, 1 stop bit
	USARTC0.CTRLA = 0;	   //interrupts

	//  Enable the transmitter and/or receiver
	USARTC0_CTRLB = USART_TXEN_bm | USART_RXEN_bm;  //Enables retriever & transmitter
	USARTC0.CTRLA |= USART_RXCINTLVL0_bm;
	PMIC_CTRL |= PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm;
	PMIC_CTRL |= PMIC_RREN_bm;
	irq_initialize_vectors();
	cpu_irq_enable();
	delay_ms(50);
	
	//Parsing
	for(int16_t i=0;i<6;i++)
		{
		delay_ms(50);
		sscanf(gps->readTail,"%s",gpsString);
		if(IsStringRingReadyForParse(gps))
		{	
			if(memcmp(gps->readTail, "$GNGGA", 6) == 0)
			{
				//printf(gps->readTail);
				sscanf(gps->readTail,"%s",dummy);   //Turns gps->readTail into a string dummy
				int16_t placeOfLastComma = 0;
				int16_t placeOfTracker = 0;
				commaCount=0;
				while (placeOfTracker<strlen(dummy)) 
				{
					if(dummy[placeOfTracker]==44)
					{
						for(int16_t i=placeOfLastComma+1;i<placeOfTracker;i++)
						{
							switch(commaCount)
							{
								case 1:
									date[i-placeOfLastComma-1]=dummy[i];
								break;
							
								case 2:
									latitude[i-placeOfLastComma-1]=dummy[i];
								break;
							
								case 4:
									longitude[i-placeOfLastComma-1]=dummy[i];
								break;
							
								case 7:
									satNumber[i-placeOfLastComma-1]=dummy[i];
								break;
							
								case 9:
									readAlt[i-placeOfLastComma-1]=dummy[i];
								break;
							}
						}
						commaCount++;
						placeOfLastComma=placeOfTracker;
					}
					placeOfTracker++;
				}
				kphSpeedDouble=atof(kphSpeed);
				gpsSpeed=(int16_t)kphSpeedDouble*27.77;		
			}	
			else if(memcmp(gps->readTail, "$GNVTG", 6) == 0)
			{
				sscanf(gps->readTail,"%s",dummy);   //Turns gps->readTail into a string
				int16_t placeOfLastComma = 0;
				int16_t placeOfTracker = 0;
				commaCount=0;
				while (placeOfTracker<strlen(dummy)) 
				{
					if(dummy[placeOfTracker]==44)
					{
						for(int16_t i=placeOfLastComma+1;i<placeOfTracker;i++)
						{
							switch(commaCount)
							{
								case 7:
									kphSpeed[i-placeOfLastComma-1]=dummy[i];
								break;
							}
						}
						commaCount++;
						placeOfLastComma=placeOfTracker;
					}
					placeOfTracker++;
				}
			}		
		}
		StringRingSeekNextReadableString(gps);
	}
	
	if(strlen(latitude)==0)
	{
		latitude[0]=48;
	}
	
	if(strlen(longitude)==0)
	{
		longitude[0]=48;
	}
	
	if(strlen(satNumber)==0)
	{
		satNumber[0]=48;
	}
	
	if(strlen(readAlt)==0)
	{
		readAlt[0]=48;
	}
 }

ISR(USARTC0_RXC_vect)//Interrupt
{
	StringRingWrite(gps, USARTC0.DATA);
}
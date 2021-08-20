#include <asf.h>
#include <string.h>
#include <nvm.h>
#include <math.h>
#include "Drivers/uart.h"
#include "Drivers/adc.h"
#include "Drivers/imu.h"
#include "Drivers/mpu9250.h"
//#include "Drivers/ctl_stringring.h"
//#include "Drivers/gpsParser.h"
#include "Drivers/nmea.h"
#include "Drivers/MS5607.h"

MS5607 static_sensor;
extern int32_t msPressure;
// int16_t currentAltitude;
int16_t alt;
// int16_t smoothedAlt;
// int16_t lastSmoothedAlt;
// float alpha = 0.36;

extern char latitude [30];
extern char longitude [30];
extern char satNumber [30];
extern char readAlt [30];
extern char kphSpeed [30];
uint16_t adcValues [4];
uint16_t i;
uint16_t j;
uint8_t *gpsPacketString [10];
extern int16_t gpsSpeed;
//StringRing *gps = NULL;







volatile uint8_t status=0;
volatile uint8_t BuffIndex=0;
char USARTBuffer[80];
char NMEAPacket[80];
char GPGGA[80];//working buffer
//GPGGA data
char   Lat[9];
char Long[10];
char GPSTime[6];
char NS[1];
char EW[1];
char FixQuality;
char Altitude[7];

// GPRMC data
char Speed[5];//in knots
char Heading[5];
char GPSDate[6];




typedef struct {
	char buffer[2048];
	uint16_t head;
	uint16_t tail;
	uint16_t maxlen;
	} ring_buf_t;

int ring_push(ring_buf_t *r, uint8_t data) {
	// push onto head, then move head up
	int next;
	
	next = r->head+1;
	if(next >= r->maxlen) { // loop back around to beginning
		next = 0;
	}
	if(next == r->tail) { // buffer full
		return -1;
	}
	
	r->buffer[r->head] = data; // load data into head
	r->head = next;			   // head at next offset
	
	return 0;				   // success
}

int ring_pop(ring_buf_t *r, uint8_t *data) {
	// pop from tail, then move tail up
	int next;
	
	if(r->head == r->tail) { // no data
		return -1;
	}
	
	next =  r->tail + 1;
	if(next >= r->maxlen) {
		next = 0;
	}
	
	*data = r->buffer[r->tail]; // load data from tail
	r->tail = next;				// tail at next offset
	return 0;					// success
}

volatile ring_buf_t gps_ring;

ISR(USARTC0_RXC_vect)
{
	char data=USARTC0.DATA;
	if(ring_push(&gps_ring, data)) {
		//out of space in gps ring buffer
	}
}


int main (void)
{
	// ************** INITS ************** //
	gps_ring.head = 0;
	gps_ring.tail = 0;
	gps_ring.maxlen = 2048;
	char gps_data;
	int GPSLock = 0;
	int GPSCounter = 0;
	
	board_init();
	sei();

	delay_ms(10);
	sysclk_init();
	
	sysclk_enable_module(SYSCLK_PORT_C, SYSCLK_HIRES); //Enable GPS HiRes
	sysclk_enable_peripheral_clock(&ADCA);             //Voltage
	adc_init();
	
	uart_gps_init_default();
	//printf("$PUBX,41,1,0007,0003,38400,0*20\r\n");
	//uart_gps_init();
	
	
	delay_ms(30);
	// COMMANDS FOUND AT (https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf)
	//  THIS COMMAND SETS FLIGHT MODE (airborne mode)
	uint8_t setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC };
	for(int j=0;j<3;j++)
	{
		for(int i=0; i<sizeof(setNav)/sizeof(uint8_t); i++)
		{
			printf("%c",setNav[i]);
		}
		printf("\n");
		delay_ms(30);
	}
	
	delay_ms(30);
	//  THIS COMMAND DISABLES 1hz GLL
	uint8_t setGLL[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B};
	for(int j=0;j<3;j++)
	{
		for(int i=0; i<sizeof(setGLL)/sizeof(uint8_t); i++)
		{
			printf("%c",setGLL[i]);
		}
		printf("\r\n");
		delay_ms(30);
	}
	
	delay_ms(30);
	//  THIS COMMAND DISABLES 1hz GSV
	uint8_t setGSV[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};
	for(int j=0;j<3;j++)
	{
		for(int i=0; i<sizeof(setGSV)/sizeof(uint8_t); i++)
		{
			printf("%c",setGSV[i]);
		}
		printf("\r\n");
		delay_ms(30);
	}
	
	delay_ms(30);
	//  THIS COMMAND ENABLES 1hz GST
	uint8_t setGST[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x07, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x0C, 0x69};
	for(int j=0;j<3;j++)
	{
		for(int i=0; i<sizeof(setGST)/sizeof(uint8_t); i++)
		{
			printf("%c",setGST[i]);
		}
		printf("\r\n");
		delay_ms(30);
	}
	

	delay_ms(30);
	//  THIS COMMAND ENABLES 1hz UBX04
	uint8_t setUBX04[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF1, 0x04, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x0A, 0x5C};
	for(int j=0;j<3;j++)
	{
		for(int i=0; i<sizeof(setUBX04)/sizeof(uint8_t); i++)
		{
			printf("%c",setUBX04[i]);
		}
		printf("\r\n");
		delay_ms(30);
	}
	
	
	/*
	delay_ms(30);
	// Do not use with OpenLog. Causes Instabilities.
	//  THIS COMMAND ENABLES 1hz TIM-TP
	uint8_t setTIMTPP[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x0D, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x23, 0x27};
	for(int j=0;j<3;j++)
	{
		for(int i=0; i<sizeof(setTIMTPP)/sizeof(uint8_t); i++)
		{
			printf("%c",setTIMTPP[i]);
		}
		printf("\r\n");
	}
	*/
	
	delay_ms(30);
	
	uart_sd_init();
	GPSCounter = 0;
	while(ring_pop(&gps_ring, &gps_data)==0) { // returns 0 on success so keep going while we have data
		printf("%c", gps_data);
		GPSCounter++;
		delay_ms(1);
		if (GPSCounter>238)
		{
			GPSLock = 1;
			}else{
			GPSLock = 0;
		}
	}
	printf("\n\n");
		
	delay_ms(30);
	
	
	// Pressure
 	//static_sensor.ss_pin = IOPORT_CREATE_PIN(PORTC, 4);
 	//Initialize the SPI interface
 	//spi_init(&SPIC, &PORTC);
 	//ioport_configure_pin(static_sensor.ss_pin, IOPORT_DIR_OUTPUT || IOPORT_INIT_HIGH);
 	//pressure_init(&static_sensor);
	
	
	
	
	//IMU initialization
	twi_options_t m_options = {
		.speed = 400000,
		.speed_reg = TWI_BAUD(32000000, 400000),
	};
	sysclk_enable_peripheral_clock(&TWIE);
		
	twi_master_options_t opt = {
		.speed = 50000,
		.chip  = 0x50
	};

	twi_master_setup(&TWIE, &opt);
	twi_master_init(&TWIE, &m_options);
	twi_master_enable(&TWIE);
		
	mpu9250_t imu_e =
	{
		.twi = &TWIE,
	};
		
	init_imu(imu_e);
		
		
	// ************** MAIN LOOP ************** //
	imu_data_t imu_data;
	uint32_t packetNumber = 0;
	
	PORTB.DIR = 0b00000111; // On Board LED
	PORTB.OUT = 0b00000111; // Turn LED On
	
	printf("\nEND OF STARTUP\n");
	printf("\n\n\n\n\n\n\n\n");
	

	printf("Packet #,  Pitch,   Roll,    Yaw, AccX, AccY, AccZ,GyroX,GyroY,GyroZ, MagX, MagY, MagZ,IMUTp,HPSTp,EXTTp,BATTp,PMTTp\n");





//Neutron Test

//Blink for 2 minutes
// for (int j = 0;j<3;j++){
// 	for (int i = 0;i<120*2;i++)
// 	{
// 		PORTB.OUT = 0b00000111; // Turn LED On
// 		delay_ms(250*5/8);
// 		PORTB.OUT = 0b00000000; // Turn LED Off
// 		delay_ms(250*5/8);
// 	}
// 
// 	//On for one minute
// 	PORTB.OUT = 0b00000111; // Turn LED On
// 	for (int i = 0;i<60;i++)
// 	{
// 		delay_ms(1000*5/8);
// 	}
// }
// PORTB.OUT = 0b00000000; // Turn LED Off










/*
while(1){
	uart_gps_init();
	usart_serial_read_packet(&USARTC0, *gpsPacketString, 10);
	uart_sd_init();
	printf("%s", *gpsPacketString);
}
*/











// while(1){
// 	nmeaProcess();
// 	uart_sd_init();
// 	printf("%s", USARTBuffer);
// 	uart_gps_init();
// }












// int c = 0;
// size_t len;
// while(1){
// 	
// 	c=1;
// 	PORTB.OUT = 0b00000111; // Turn LED On
// 	while(c<4){
// 		
// 		uart_gps_init();
// 		usart_serial_read_packet(&USARTC0, *gpsPacketString, 10);
// 		
// 		uart_sd_init();
// 		
// 		printf("%s", *gpsPacketString);
// 		
// 		len = strlen(gpsPacketString);
// 		if(strchr(*gpsPacketString, 42) || strchr(*gpsPacketString, 10)) {
// 			c=c+1;
// 		}
// 	}
// 	PORTB.OUT = 0b00000000; // Turn LED Off
// 	delay_ms(10);
// 	printf("\nHello!\n");
// 	
// 	
// 	
// 	
// 	
// }


		


	uart_sd_init();
	while(1){
		
		PORTB.OUT = 0b00000111; // Turn LED On
		delay_ms(80);
		if (!GPSLock){
			PORTB.OUT = 0b00000000; // Turn LED Off
		}
		imu_data = imu_update(imu_e); // Reading IMU
		for (int i = 0; i < 4; i++)// Reading temp values
		{
			adc_set_pin(i+1);
			delay_ms(5);
			adcValues[i] = adc_read();
		}
		
		
		uart_sd_init();
		printf("%8lu,%3d.%03d,%3d.%03d,%3d.%03d,%5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d\n",
			packetNumber,
			(int16_t)imu_data.pitch, (int16_t)abs(((imu_data.pitch-(int16_t)imu_data.pitch)*1000)),
			(int16_t)imu_data.roll, (int16_t)abs(((imu_data.roll-(int16_t)imu_data.roll)*1000)),
			(int16_t)imu_data.yaw, (int16_t)abs(((imu_data.yaw-(int16_t)imu_data.yaw)*1000)),
			imu_data.data.acc_x, imu_data.data.acc_y, imu_data.data.acc_z,
			imu_data.data.gyro_x, imu_data.data.gyro_y, imu_data.data.gyro_z,
			imu_data.data.mag_x, imu_data.data.mag_y, imu_data.data.mag_z,
			imu_data.data.imu_temperature,
			adcValues[0], adcValues[1], adcValues[2], adcValues[3]);
		
		// Wait for new GPS packet
		while(ring_pop(&gps_ring, &gps_data)!=0){}
		printf("%c", gps_data);
		delay_ms(10);
		
		GPSCounter = 0;
		// While the packet is coming in 
		while(ring_pop(&gps_ring, &gps_data)==0) { // returns 0 on success so keep going while we have data
			printf("%c", gps_data);
			GPSCounter++;
			delay_us(850);
			if (GPSCounter>335)
			{
				GPSLock = 1;
			}else{
				GPSLock = 0;
			}
		}
		printf("\n\n");
		
		// turn sd off, gps on
		uart_gps_init();
		
		
		packetNumber = packetNumber + 1;
	}



// 	while(1){
// 		
// 		uart_gps_init();
// 		//parseGPS();
// 		
// 		//alt = readMS5607(&static_sensor);
// 		
// 		delay_ms(500);
// 		uart_sd_init();
// 
// 		imu_data = imu_update(imu_e); // Reading IMU
// 		//PORTC.OUT = 0b00000001; // Turn Board LED On
// 		PORTB.OUT = 0b00000111; // Turn Status LED On
// 		
// 		printf("%10lu, %6d.%03d, %6d.%03d, %6d.%03d, %6d, %6d, %6d, %6d, %6d, %6d, %6d, %6d, %6d, %6d, %s, %s, %s, %s, %i\n", 
// 						packetNumber, 
// 						(int16_t)imu_data.pitch, (int16_t)abs(((imu_data.pitch-(int16_t)imu_data.pitch)*1000)), 
// 						(int16_t)imu_data.roll, (int16_t)abs(((imu_data.roll-(int16_t)imu_data.roll)*1000)), 
// 						(int16_t)imu_data.yaw, (int16_t)abs(((imu_data.yaw-(int16_t)imu_data.yaw)*1000)),
// 						imu_data.data.acc_x, imu_data.data.acc_y, imu_data.data.acc_z,
// 						imu_data.data.gyro_x, imu_data.data.gyro_y, imu_data.data.gyro_z,
// 						imu_data.data.mag_x, imu_data.data.mag_y, imu_data.data.mag_z,
// 						imu_data.data.imu_temperature,
// 						latitude,longitude,readAlt,satNumber,gpsSpeed);
// 		
// 		if (atoi(latitude) == 0 && atoi(longitude) == 0 && atoi(readAlt) == 0 && atoi(satNumber) == 0 && gpsSpeed == 0){
// 			//PORTC.OUT = 0b00000000; // Turn LED Off
// 			PORTB.OUT = 0b00000000; // Turn LED Off
// 		}else{
// 			//PORTC.OUT = 0b00000001; // Turn LED On
// 			PORTB.OUT = 0b00000111; // Turn LED On
// 		}
// 		
// 		packetNumber = packetNumber + 1;
// 		delay_ms(3);
// 	}
}

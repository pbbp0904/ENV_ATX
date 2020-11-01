/*
 * imu.c
 *
 * Created: 5/23/2017 5:35:00 PM
 *  Author: trb0023
 */ 

#include "Drivers/imu.h"
static double mapdouble(double x, double in_min, double in_max, double out_min, double out_max);
static void update_max_min_mag(void);
static void calculate_offset_mag(void);

static int16_t mag_x_max =-32768;
static int16_t mag_x_min = 32767;
static int16_t mag_y_max =-32768;
static int16_t mag_y_min = 32767;
static int16_t mag_z_max =-32768;
static int16_t mag_z_min = 32767;

// int16_t MAG_X_OFF = 123;
// int16_t MAG_Y_OFF = 123;
// int16_t MAG_Z_OFF = 123;

int16_t MAG_X_OFF = -319;
int16_t MAG_Y_OFF = 404;
int16_t MAG_Z_OFF = 2627;

int16_t ACC_X_OFF = -292;
int16_t ACC_Y_OFF = -242;
int16_t ACC_Z_OFF = -740;

int16_t GYR_X_OFF = 6;
int16_t GYR_Y_OFF = 0;
int16_t GYR_Z_OFF = 0;

mpu9250_data_t data; //he names the box that will be filled with data "data"

void init_imu(mpu9250_t imu) //this is the function for initializing imu
{
	init_mpu9250(imu, 100); //to initialize imu, use the init_mpu9250 function with the box "imu" and odr (output data rate) of 100 hz
}

bool imu_is_data_ready(void) //true or false: is data ready
{
	return (bool)is_data_rdy(); //returns true or false off of 0 or 1 checking if IMU data is ready based on the function is_data_ready
}

imu_data_t imu_update(mpu9250_t imu) //returns an imu data box based on the update. IMU data box is neater
{
	data = read_mpu9250(imu); //tells the mcu to read the mpu thats located on the pins defined in the imu box and put that data into the box with a ton of little boxes
	
	//filters data
	MahonyAHRSupdate(	mapdouble((double)data.gyro_x+(GYR_X_OFF), -32768.0, 32767.0, -2000, 2000),
						mapdouble((double)data.gyro_y+(GYR_Y_OFF), -32768.0, 32767.0, -2000.0, 2000.0),
						mapdouble((double)data.gyro_z+(GYR_Z_OFF), -32768.0, 32767.0, -2000.0, 2000.0),
						(double)data.acc_x+(ACC_X_OFF), (double)data.acc_y+(ACC_Y_OFF), (double)data.acc_z+(ACC_Z_OFF),
						(double)data.mag_y+(MAG_Y_OFF), (double)data.mag_x+(MAG_X_OFF), -((double)data.mag_z+(MAG_Z_OFF)));

	
	//converts all the little boxes from the mpu into more digestable boxes of angles
	imu_data_t imudata;					
	imudata.yaw   = atan2(2.0 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3); //gets yaw angle in radians
	imudata.pitch = -asin(2.0 * (q1 * q3 - q0 * q2)); //gets pitch angle in radians MAY NOT CARE ABOUT
	imudata.roll  = atan2(2.0 * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3); //gets roll angle in radians
	imudata.pitch *= 180.0 / PI; //converts pitch angle to degrees
	imudata.yaw   *= 180.0 / PI; //converts yaw angle to degrees	
	imudata.roll  *= 180.0 / PI;  //converts roll to degrees
	imudata.data = data;
	
	return imudata;
}

static double mapdouble(double x, double in_min, double in_max, double out_min, double out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; //converting scale from ugly numbers to pretty numbers (look in mahony update above for the numbers).
}


static void update_max_min_mag(void)
{
	if(data.mag_x > mag_x_max)
	{
		mag_x_max = data.mag_x;
	}
	if (data.mag_x < mag_x_min)
	{
		mag_x_min = data.mag_x;
	}
	if(data.mag_y > mag_y_max)
	{
		mag_y_max = data.mag_y;
	}
	if (data.mag_y < mag_y_min)
	{
		mag_y_min = data.mag_y;
	}
	if(data.mag_z > mag_z_max)
	{
		mag_z_max = data.mag_z;
	}
	if (data.mag_z < mag_z_min)
	{
		mag_z_min = data.mag_z;
	}
} //filter to make it read out maxes if actual numbers exceed maxes

static void calculate_offset_mag(void)
{
	MAG_X_OFF = -((mag_x_max + mag_x_min)/2); //x offset
	MAG_Y_OFF = -((mag_y_max + mag_y_min)/2);// y offset
	MAG_Z_OFF = -((mag_z_max + mag_z_min)/2); //z offset
}

void imu_calibrate_mag(mpu9250_t imu) //CALIBRATION FUNCTION (WOO!)
{
	printf("\nCalibrating magnetometer! Swing the electronics around randomly!");

	delay_ms(2000);

	mag_x_max =-32768;
	mag_x_min = 32767;
	mag_y_max =-32768;
	mag_y_min = 32767;
	mag_z_max =-32768;
	mag_z_min = 32767;

	for (uint16_t i = 0; i < MAG_CALIBRATION_SAMPLES;)
	{
		
		if(is_data_rdy())
		{
			i++;

			data = read_mpu9250(imu);

			update_max_min_mag();

			if(i%30 == 0)
			{
				
				printf("\n%.1f%%\n	X %i %i\n	Y %i %i\n	Z %i %i", ((double)i*100.0/(double)MAG_CALIBRATION_SAMPLES), mag_x_min, mag_x_max, mag_y_min, mag_y_max, mag_z_min, mag_z_max);
			}

		}

		
	}

	calculate_offset_mag();

	printf("\nCalibration Complete!\nNew Offsets:\nX: %i  Y: %i  Z %i\n", MAG_X_OFF, MAG_Y_OFF, MAG_Z_OFF);

	delay_ms(5000);
} //Finds calibration values

imu_mag_cal_t imu_get_mag_calibration(void)
{
	imu_mag_cal_t cal;
	cal.x_off = MAG_X_OFF;
	cal.y_off = MAG_Y_OFF;
	cal.z_off = MAG_Z_OFF;

	return cal;
} //returns calibration values

void imu_set_mag_calibration(imu_mag_cal_t calibration)
{
	MAG_X_OFF = calibration.x_off;
	MAG_Y_OFF = calibration.y_off;
	MAG_Z_OFF = calibration.z_off;
} // sets values
/*
 * imu.h
 *
 * Created: 5/23/2017 5:35:14 PM
 *  Author: trb0023
 */ 

#include <math.h>
#include <asf.h>
#include "Drivers/mpu9250.h"
#include "Drivers/MahonyAHRS.h"


#ifndef IMU_H_
#define IMU_H_

#define MPU_UPDATE_DIV 3

#define MAG_CALIBRATION_SAMPLES 5000

typedef struct  
{
	double pitch;
	double roll;
	double yaw;
	mpu9250_data_t data;
}imu_data_t;

typedef struct
{
	int16_t x_off;
	int16_t y_off;
	int16_t z_off;
}imu_mag_cal_t; //creates box that holds 3 boxes that will hold the offsets define in calibration function later

void init_imu(mpu9250_t imu); //initialize IMU

bool imu_is_data_ready(void); //true or false: is there data?

imu_data_t imu_update(mpu9250_t imu); // when this function runs, it will return an imu_data_t (box with 3 boxes with the actual stuff in it:))

void imu_calibrate_mag(mpu9250_t imu); //function that calibrates the magnetometer

imu_mag_cal_t imu_get_mag_calibration(void); //puts the stuff into a calibration box
void imu_set_mag_calibration(imu_mag_cal_t calibration); //names a box "calibration" that will be used in later functions

#endif /* IMU_H_ */
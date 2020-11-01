/*
 * mpu9250.h
 *
 * Created: 2/12/2015 7:18:21 PM
 *  Author: aeldemire, with edits from trb0023
 */ 

#ifndef MPU9250_H
#define MPU9250_H

#include "asf.h"



//MPU9250 Pin for Interrupt. Attached to the "INT" pin on the breakout board
#define MPU9250_INT_PIN_NUM		0
#define MPU9250_INT_PIN_PORT	PORTD
#define MPU9250_INT		IOPORT_CREATE_PIN(PORTD, 0)	//Interrupt Pin for IMU, Used as clock source
#define MPU9250_INT_LVL		PMIC_HILVLEN_bm //	PORT_INT0LVL_HI_gc //making this a high level interrupt, which means it can interrupt everything
#define MPU9250_INT_vect	PORTD_INT0_vect //vector is where in the memory the interrupt service routine is. Which is the procedure the interupt follows when its interrupting
#define IMU_SAMPLE_RATE		100 //100 hz

#define MPU9250_ACC_SENSITIVTY		1//32.8/10 // divided by 100 makes it convert to 0.1 deg/s
#define MPU9250_GYRO_SENSITIVITY	1//4096/1000 //divided by 1000 makes it convert to mG (0.001 gee)
#define MPU9250_TEMP_SENSITIVITY	1 //still trying to find sensitivity for temp sensor; maybe it's just straight degrees? (seems really unlikely, but I can't find any data)

//Full Scale Range Settings 
//Gyro
#define MPU9250_GYRO_FS_250DPS	0x00	//Gyro full scale range to +/-250 deg/sec
#define MPU9250_GYRO_FS_500DPS	0x01	//Gyro full scale range to +/-500 deg/sec
#define MPU9250_GYRO_FS_1000DPS	0x02	//Gyro full scale range to +/-1000 deg/sec
#define MPU9250_GYRO_FS_2000DPS	0x03	//Gyro full scale range to +/-2000 deg/sec
//Accelerometer
#define MPU9250_ACC_FS_2G	0x00	//Accelerometer full scale range to +/-2G
#define MPU9250_ACC_FS_4G	0x01	//Accelerometer full scale range to +/-4G
#define MPU9250_ACC_FS_8G	0x02	//Accelerometer full scale range to +/-8G
#define MPU9250_ACC_FS_16G	0x03	//Accelerometer full scale range to +/-16G

//These are the Full Scale ranges of the IMU
#define MPU9250_GYRO_SCALE	MPU9250_GYRO_FS_2000DPS	//This is the full scale range of the Gyro
#define MPU9250_ACC_SCALE	MPU9250_ACC_FS_16G		//This is the full scale range of the Accelerometer


//Slave address of this chip
#define MPU9250_TWI_SLAVE_ADD	0b01101000 //binary, what this is. address of imu
#define AK8963_TWI_SLAVE_ADD	0x0C //magnetometer has it's own house  (address)
#define AK8963_DEVICE_ID        0x48  //id of magnetometer
#define MPU9250_I2C_READ_FLAG	0x80	//used when reading in I2C master mode
//Offset of raw temperature reading
//#define MPU9250_TEMP_OFFSET 13200
//Baseline reading level of MPU9250 temperature
//#define MPU9250_TEMP_BASE 35

//Calibration offsets of each axis (ADC counts), observed per chip
//#define MPU9250_GYRO_X_OFFSET 207
//#define MPU9250_GYRO_Y_OFFSET -11 //off by 0.0625
//#define MPU9250_GYRO_Z_OFFSET 77 //off by 0.1875



//Devices
typedef struct //struct is like a list, but only accepts certain types of variables. tells which kinds of variables below
{
	TWI_t *twi;			//the TWI port that this sensor is connected to. (Pointer)
	uint8_t int_pin; //A configurable interrupt output pin on this sensor. (uint8_t = telling what type of variable it is, int_pin = variable)
	uint8_t fsync;	//Frame synchronization pin
}mpu9250_t; //name of struct. creates structure that hold the por, pin, anf frme pin of the MP9250. Creates an empty big box of 3 boxes that will be filled with info later. Can have multiple different big boxes with 3 little boxes.

typedef struct
{	
	uint8_t int_status;
	
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
	
	int16_t imu_temperature; //Temperature of sensor in 0.01 degrees Celsius/bit
	
	int16_t gyro_x; //Rate of rotation around chip's x axis in 0.1 degrees/second/bit
	int16_t gyro_y; //Rate of rotation around chip's y axis in 0.1 degrees/second/bit
	int16_t	gyro_z; //Rate of rotation around chip's z axis in 0.1 degrees/second/bit
	
	int16_t mag_x; 
	int16_t mag_y; 
	int16_t	mag_z; 
}mpu9250_data_t; //creates the structure that holds the data from the IMU. Creates lots of boxes in a bigger box. 

//*****************API FUNCTIONS******************
//***Caller should use these to use this sensor***
//************************************************
void init_mpu9250(mpu9250_t chip, uint8_t odr);	//Initializes the IMU
mpu9250_data_t read_mpu9250(mpu9250_t chip);	//Reads the Gyroscope Data and returns the results

//TODO: enable self test features	
void mpu9250_gyro_init(mpu9250_t chip);		//Initializes the Gyroscope
void mpu9250_acc_init(mpu9250_t chip);		//Initializes the Accelerometer
void mpu9250_mag_init(mpu9250_t chip);		//Initializes the Magnetometer
void mpu9250_gyro_cal(mpu9250_t chip);		//calibrates the Gyroscope
void mpu9250_acc_cal(mpu9250_t chip);		//calibrates the Accelerometer
void mpu9250_mag_cal(mpu9250_t chip);		//calibrates the Magnetometer
void mpu9250_dmp_init(mpu9250_t chip);		//Initializes the Digital Motion Processor
void mpu9250_int_init(mpu9250_t chip);		//Configures the mpu9250 Interrupt
void mpu9250_ODR_init(mpu9250_t chip, uint8_t sample_rate);	//Configures the mpu9250 output data rate

uint8_t who_am_i(mpu9250_t chip);
bool is_data_rdy(void);
//returns the time in counts
uint32_t what_time_is_it(void);

//reset mission time
void reset_mission_time(void);

//returns true if the timer interrupt flag is set (use for time keeping)
//sets the timer flag to 0 if returning a true
uint8_t is_sample_timer_flag_set(void);

//registers
#define MPU9250_SELF_TEST_X_GYRO_REG	0	//The value in this register indicates the self test output generated during manufacturing tests. This value is to be used to check against subsequent self test outputs performed by the end user.
#define MPU9250_SELF_TEST_Y_GYRO_REG	1	//The value in this register indicates the self test output generated during manufacturing tests. This value is to be used to check against subsequent self test outputs performed by the end user.
#define MPU9250_SELF_TEST_Z_GYRO_REG	2	//The value in this register indicates the self test output generated during manufacturing tests. This value is to be used to check against subsequent self test outputs performed by the end user.

#define MPU9250_SELF_TEST_X_ACC_REG	13	//The value in this register indicates the self test output generated during manufacturing tests. This value is to be used to check against subsequent self test outputs performed by the end user.
#define MPU9250_SELF_TEST_Y_ACC_REG	14	//The value in this register indicates the self test output generated during manufacturing tests. This value is to be used to check against subsequent self test outputs performed by the end user.
#define MPU9250_SELF_TEST_Z_ACC_REG	15	//The value in this register indicates the self test output generated during manufacturing tests. This value is to be used to check against subsequent self test outputs performed by the end user.

#define MPU9250_XH_OFFS_USR_REG	19	//OffsetLSB= X_OFFS_USR * 4 / 2^FS_SEL
#define MPU9250_XL_OFFS_USR_REG	20	//OffsetDPS= X_OFFS_USR * 4 / 2^FS_SEL / Gyro_Sensitivity
#define MPU9250_YH_OFFS_USR_REG	21	//OffsetLSB= X_OFFS_USR * 4 / 2^FS_SEL
#define MPU9250_YL_OFFS_USR_REG	22	//OffsetDPS= X_OFFS_USR * 4 / 2^FS_SEL / Gyro_Sensitivity
#define MPU9250_ZH_OFFS_USR_REG 23	//OffsetLSB= X_OFFS_USR * 4 / 2^FS_SEL
#define MPU9250_ZL_OFFS_USR_REG	24	//OffsetDPS= X_OFFS_USR * 4 / 2^FS_SEL / Gyro_Sensitivity

#define MPU9250_SMPLRT_DIV_REG		25	//SAMPLE_RATE=Internal_Sample_Rate / (1 + SMPLRT_DIV)
#define MPU9250_CONFIG_REG			26	//includes sample rate base, bandwidth, digital low pass filter see register map document
#define MPU9250_GYRO_CONFIG_REG		27
#define MPU9250_ACC_CONFIG_REG		28
#define MPU9250_ACC_CONFIG_2_REG	29	//handles overriding the LPF - not used as of now
#define MPU9250_LP_ACC_ODR_CTRL_REG	30	//Sets the frequency of waking up the chip to take a sample of accel data – the low power accel Output Data Rate.
#define MPU9250_WOM_THRESHOLD_REG	31	//for setting up the wake on motion threshold
#define MPU9250_FIFO_ENABLE_REG		35	//using the buffer
#define MPU9250_I2C_MSTR_CTRL_REG	36	//I2C Master Control	TODO: LOOK Into that

#define MPU9250_I2C_SLV0_ADDR	37
#define MPU9250_I2C_SLV0_REG	38
#define MPU9250_I2C_SLV0_CTRL	39
#define MPU9250_I2C_SLV1_ADDR	40
#define MPU9250_I2C_SLV1_REG	41
#define MPU9250_I2C_SLV1_CTRL	42
#define MPU9250_I2C_SLV2_ADDR	43
#define MPU9250_I2C_SLV2_REG	44
#define MPU9250_I2C_SLV2_CTRL	45

#define MPU9250_I2C_MSTR_STAT_REG	54
#define MPU9250_INT_PIN_CNFG_REG	55
#define MPU9250_INT_EN_REG			56

#define MPU9250_INT_STATUS_REG		58

#define MPU9250_ACC_XOUT_H_REG		59
#define MPU9250_ACC_XOUT_L_REG		60
#define MPU9250_ACC_YOUT_H_REG		61
#define MPU9250_ACC_YOUT_L_REG		62
#define MPU9250_ACC_ZOUT_H_REG		63
#define MPU9250_ACC_ZOUT_L_REG		64

#define MPU9250_TEMP_OUT_H_REG		65
#define MPU9250_TEMP_OUT_L_REG		66

#define MPU9250_GYRO_XOUT_H_REG		67
#define MPU9250_GYRO_XOUT_L_REG		68
#define MPU9250_GYRO_YOUT_H_REG		69
#define MPU9250_GYRO_YOUT_L_REG		70
#define MPU9250_GYRO_ZOUT_H_REG		71
#define MPU9250_GYRO_ZOUT_L_REG		72
#define MPU9250_EXT_SENS_DATA_00	73
#define MPU9250_EXT_SENS_DATA_01	74
#define MPU9250_EXT_SENS_DATA_02	75
#define MPU9250_EXT_SENS_DATA_03	76
#define MPU9250_EXT_SENS_DATA_04	77
#define MPU9250_EXT_SENS_DATA_05	78
#define MPU9250_EXT_SENS_DATA_06	79
#define MPU9250_EXT_SENS_DATA_07	80
#define MPU9250_EXT_SENS_DATA_08	81
#define MPU9250_EXT_SENS_DATA_09	82
#define MPU9250_EXT_SENS_DATA_10	83
#define MPU9250_EXT_SENS_DATA_11	84
#define MPU9250_EXT_SENS_DATA_12	85
#define MPU9250_EXT_SENS_DATA_13	86
#define MPU9250_EXT_SENS_DATA_14	87
#define MPU9250_EXT_SENS_DATA_15	88
#define MPU9250_EXT_SENS_DATA_16	89
#define MPU9250_EXT_SENS_DATA_17	90
#define MPU9250_EXT_SENS_DATA_18	91
#define MPU9250_EXT_SENS_DATA_19	92
#define MPU9250_EXT_SENS_DATA_20	93
#define MPU9250_EXT_SENS_DATA_21	94
#define MPU9250_EXT_SENS_DATA_22	95
#define MPU9250_EXT_SENS_DATA_23	96
#define MPU9250_I2C_SLV0_DO_REG	99
#define MPU9250_I2C_SLV1_DO_REG	100
#define MPU9250_I2C_SLV2_DO_REG	101
#define MPU9250_I2C_SLV3_DO_REG	102
#define MPU9250_I2C_MSTR_DELAY_CTRL_REG	103
#define MPU9250_SIGNAL_PATH_RST_REG		104
#define MPU9250_ACC_INT_CTRL_REG		105

#define MPU9250_USER_CTRL_REG		106
#define MPU9250_PWR_MGMT_1_REG		107
#define MPU9250_PWR_MGMT_2_REG		108
#define MPU9250_WHO_AM_I_REG		117 //defaults to 0x71

#define AK8963_WIA_REG		0x00
#define AK8963_INFO_REG		0x01
#define AK8963_ST1_REG		0x02
#define AK8963_HXL_REG		0x03
#define AK8963_HXH_REG		0x04
#define AK8963_HYL_REG		0x05
#define AK8963_HYH_REG		0x06
#define AK8963_HZL_REG		0x07
#define AK8963_HZH_REG		0x08
#define AK8963_ST2_REG		0x09
#define AK8963_CNTL_REG		0x0A
#define AK8963_RSV_REG		0x0B
#define AK8963_ASTC_REG		0x0C
#define AK8963_TS1_REG		0x0D
#define AK8963_TS2_REG		0x0E
#define AK8963_I2CDIS_REG	0x0F
#define AK8963_ASAX_REG		0x10
#define AK8963_ASAY_REG		0x11
#define AK8963_ASAZ_REG		0x12



#endif /* MPU9250_H_ */
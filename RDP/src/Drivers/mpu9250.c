/*
 * mpu9250.c
 *
 * Created: 2/12/2015 7:22:03 PM
 *  Author: aeldemire, with edits from trb0023
 */ 
//https://github.com/kriswiner/MPU-9250/blob/master/STM32F401/MPU9250.h

//interrupts?
//self test
//scaling and calibration
//
#include "mpu9250.h"

static uint8_t mpu9250_twi_data[256];
static uint8_t data_rdy_flag = 0;
static uint8_t timerflag = 0;
static uint32_t time = 0;

static volatile float ASAX = 0;
static volatile float ASAY = 0;
static volatile float ASAZ = 0;

static volatile float GYRO_SENSITIVITY = 1.0;
static volatile float ACC_SENSITIVITY = 2.048;

static twi_package_t readmpu9250 = {
	.addr[0]	  = 0x04,
	.addr_length = 1,
	.chip        = MPU9250_TWI_SLAVE_ADD,
	.buffer      = mpu9250_twi_data,
	.length      = 1,
	.no_wait     = false
};

static twi_package_t readak8963 = {
	.addr[0]	  = 0x04,
	.addr_length = 1,
	.chip        = AK8963_TWI_SLAVE_ADD,
	.buffer      = mpu9250_twi_data,
	.length      = 1,
	.no_wait     = false
};

inline static void ak8963_twi_write_reg(mpu9250_t chip, uint8_t regi,uint8_t val)
{
	readmpu9250.chip=AK8963_TWI_SLAVE_ADD;
	readmpu9250.addr[0]	  = regi;
	readmpu9250.length=1;
	mpu9250_twi_data[0]=val;
	twi_master_write(chip.twi, &readak8963);
}

inline static void ak8963_twi_read_multi(mpu9250_t chip, uint8_t regi, uint8_t length)
{
	readmpu9250.chip=AK8963_TWI_SLAVE_ADD;
	readmpu9250.addr[0]	  = regi;
	readmpu9250.length=length;
	twi_master_read(chip.twi, &readak8963);
}

inline static void mpu9250_twi_write_reg(mpu9250_t chip, uint8_t regi,uint8_t val)
{
	readmpu9250.chip=MPU9250_TWI_SLAVE_ADD;
	readmpu9250.addr[0]	  = regi;
	readmpu9250.length=1;
	mpu9250_twi_data[0]=val;
	twi_master_write(chip.twi, &readmpu9250);
}

inline static void mpu9250_twi_read_multi(mpu9250_t chip, uint8_t regi, uint8_t length)
{
	readmpu9250.chip=MPU9250_TWI_SLAVE_ADD;
	readmpu9250.addr[0]	  = regi;
	readmpu9250.length=length;
	twi_master_read(chip.twi, &readmpu9250);
}

//Configures the mpu9250 output data rate where sample rate is in Hz
void mpu9250_ODR_init(mpu9250_t chip, uint8_t sample_rate)
{
	sample_rate = ((uint8_t)((1000/((uint16_t)sample_rate)) - 1));
	mpu9250_twi_write_reg(chip,MPU9250_SMPLRT_DIV_REG,sample_rate);
}	

//Initializes the Gyroscope
void mpu9250_gyro_init(mpu9250_t chip)
{
	//Gyro Configurations
	//FIFO_MODE		= 0
	//EXT_SYNC_SET	= 0
	//DLPF_CFG		= 001
	//FCHOICE_B		= 00
	//GYRO_FS_SEL	= 11
	
	//This sets up the gyro in the following mode
	//fifo is overwritten when full
	//FSYNC is not used
	//Gyro Digital LPF BW	= 92Hz
	//Gyro Delay			= 3.9ms
	//Temp Digital LPF BW	= 98Hz
	//Temp Delay			= 2.8ms
	//Gyro Full Scale		= +/-2000dps
	
	mpu9250_twi_write_reg(chip,MPU9250_CONFIG_REG,0b00000010);
	mpu9250_twi_write_reg(chip,MPU9250_GYRO_CONFIG_REG,(MPU9250_GYRO_SCALE<<3));
	
// 	switch(MPU9250_GYRO_SCALE)
// 	{
// 		case MPU9250_GYRO_FS_250DPS:
// 			GYRO_SENSITIVITY = 131/10;
// 		break;
// 		case MPU9250_GYRO_FS_500DPS:
// 			GYRO_SENSITIVITY = 65.5/10;
// 		break;
// 		case MPU9250_GYRO_FS_1000DPS:
// 			GYRO_SENSITIVITY = 32.8/10;
// 		break;
// 		case MPU9250_GYRO_FS_2000DPS:
// 			GYRO_SENSITIVITY = 16.4/10;
// 		break;
// 	}
}

//Initializes the Accelerometer
void mpu9250_acc_init(mpu9250_t chip)
{
	//Accelerometer Configurations
	//ACCEL_FS_SEL	= 10 (+/- 8g)
	//FCHOICE_B		= 0
	//A_DLPFCFG		= 1
	
	//This sets up the Accelerometer in the following mode
	//Acc Digital LPF BW			= 92Hz
	//Acc Delay						= 7.80ms
	//Acc Noise Density (ug/rtHz)	= 250
	//Acc Full Scale				= +/- 8g
	mpu9250_twi_write_reg(chip,MPU9250_ACC_CONFIG_REG,(MPU9250_ACC_SCALE<<3));
	mpu9250_twi_write_reg(chip,MPU9250_ACC_CONFIG_2_REG,0b0000010);
	
	//SENSITIVITY in mG
// 	switch(MPU9250_ACC_SCALE)
// 	{
// 		case MPU9250_ACC_FS_2G:
// 			ACC_SENSITIVITY = 16384/1000;
// 		break;
// 		case MPU9250_ACC_FS_4G:
// 			ACC_SENSITIVITY = 8192/1000;
// 		break;
// 		case MPU9250_ACC_FS_8G:
// 			ACC_SENSITIVITY = 4096/1000;
// 		break;
// 		case MPU9250_ACC_FS_16G:
// 			ACC_SENSITIVITY = 2048/1000;
// 		break;
// 	}
}

//Initializes the Magnetometer
void mpu9250_mag_init(mpu9250_t chip)
{
	//SHAMELESSLY COPIED FROM http://www.emlid.com/navio-mpu9250-tutorial-cpp/
	//Initializes the Magnetometer
	mpu9250_twi_write_reg(chip,MPU9250_I2C_MSTR_CTRL_REG,0xCD);				//  I2C configuration multi-master  IIC 400KHz
	mpu9250_twi_write_reg(chip,MPU9250_I2C_MSTR_DELAY_CTRL_REG,0x81);		//Delays shadowing of external sensor data until all data is received from the magnetometer 
	mpu9250_twi_write_reg(chip,MPU9250_USER_CTRL_REG,0x20);					//enables I2C Master mode
	
	//resets the AK8963
	mpu9250_twi_write_reg(chip,MPU9250_I2C_SLV0_ADDR,AK8963_TWI_SLAVE_ADD);	//Set the I2C slave address of AK8963 and set for write.
	mpu9250_twi_write_reg(chip,MPU9250_I2C_SLV0_REG,AK8963_RSV_REG);		//I2C slave 0 register address from where to begin data transfer
	mpu9250_twi_write_reg(chip,MPU9250_I2C_SLV0_DO_REG,0x01);				// Reset AK8963
	mpu9250_twi_write_reg(chip,MPU9250_I2C_SLV0_CTRL, 0x81);				//Enable I2C and set 1 byte
	delay_ms(10);
	
	mpu9250_mag_cal(chip);
	
	//sets to 16-bit output and continuous mode 2 100Hz sample rate
	mpu9250_twi_write_reg(chip,MPU9250_I2C_SLV0_REG, AK8963_CNTL_REG);	//I2C slave 0 register address from where to begin data transfer
	mpu9250_twi_write_reg(chip,MPU9250_I2C_SLV0_DO_REG, 0x16);			// Register value to continuous measurement in 16bit
	mpu9250_twi_write_reg(chip,MPU9250_I2C_SLV0_CTRL, 0x81);	//Enable I2C and set 1 byte
 	delay_ms(10);
	
	//sets up the master mode to read the magnetometer sensor data when it reads the rest of the data
	mpu9250_twi_write_reg(chip,MPU9250_I2C_SLV0_ADDR,AK8963_TWI_SLAVE_ADD|MPU9250_I2C_READ_FLAG);	//Set the I2C slave address of AK8963 and set for write.
	mpu9250_twi_write_reg(chip,MPU9250_I2C_SLV0_REG,AK8963_HXL_REG);		//I2C slave 0 register address from where to begin data transfer
	mpu9250_twi_write_reg(chip,MPU9250_I2C_SLV0_CTRL,0x87);					//must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.
}	

//calibrates the Gyroscope
void mpu9250_gyro_cal(mpu9250_t chip)
{
	
}

//calibrates the Accelerometer		
void mpu9250_acc_cal(mpu9250_t chip)
{
	
}

//calibrates the Magnetometer
void mpu9250_mag_cal(mpu9250_t chip)
{
	//SHAMELESSLY COPIED FROM http://www.emlid.com/navio-mpu9250-tutorial-cpp/
	
	//sets the AK8963 in Fure ROM access mode
	mpu9250_twi_write_reg(chip,MPU9250_I2C_SLV0_ADDR,AK8963_TWI_SLAVE_ADD);	//Set the I2C slave address of AK8963 and set for write.
	mpu9250_twi_write_reg(chip,MPU9250_I2C_SLV0_REG, AK8963_CNTL_REG);	//I2C slave 0 register address from where to begin data transfer
	mpu9250_twi_write_reg(chip,MPU9250_I2C_SLV0_DO_REG, 0x1F);			// Register value to continuous measurement in 16bit
	mpu9250_twi_write_reg(chip,MPU9250_I2C_SLV0_CTRL, 0x81);	//Enable I2C and set 1 byte
	delay_ms(20);
	
	//reads the rom
	mpu9250_twi_write_reg(chip,MPU9250_I2C_SLV0_ADDR,AK8963_TWI_SLAVE_ADD|MPU9250_I2C_READ_FLAG);	//Set the I2C slave address of AK8963 and set for write.
	mpu9250_twi_write_reg(chip,MPU9250_I2C_SLV0_REG,AK8963_ASAX_REG);		//I2C slave 0 register address from where to begin data transfer
	mpu9250_twi_write_reg(chip,MPU9250_I2C_SLV0_CTRL,0x83);					//Read 3 bytes from the magnetometer
	delay_ms(20);	//waits for the data to be transfered to the EXT_SENS_DATA registers 
	mpu9250_twi_read_multi(chip,MPU9250_EXT_SENS_DATA_00,4);	//reads the calibration data from the magnetometer
	ASAX = ((((float)(mpu9250_twi_data[0]))-128)/256)+1;
	ASAY = ((((float)(mpu9250_twi_data[1]))-128)/256)+1;
	ASAZ = ((((float)(mpu9250_twi_data[2]))-128)/256)+1;
	
	//sets the AK8963 back to power down mode
	mpu9250_twi_write_reg(chip,MPU9250_I2C_SLV0_ADDR,AK8963_TWI_SLAVE_ADD);	//Set the I2C slave address of AK8963 and set for write.
	mpu9250_twi_write_reg(chip,MPU9250_I2C_SLV0_REG, AK8963_CNTL_REG);	//I2C slave 0 register address from where to begin data transfer
	mpu9250_twi_write_reg(chip,MPU9250_I2C_SLV0_DO_REG, 0X10);			// Register value to continuous measurement in 16bit
	mpu9250_twi_write_reg(chip,MPU9250_I2C_SLV0_CTRL, 0x81);	//Enable I2C and set 1 byte
	delay_ms(20);		
}

//Initializes the Digital Motion Process0r
void mpu9250_dmp_init(mpu9250_t chip)
{
	//TODO: Do cool Digital Motion Processing stuff
}

//Configures the mpu9250 Interrupt 
void mpu9250_int_init(mpu9250_t chip)
{
	//Interrupt Pin Configurations
	//ACTL			= 0 (The logic level for INT pin is active high)
	//OPEN			= 0 (INT pin is configured as push-pull)
	//LATCH_INT_EN	= 1 (INT pin level held until interrupt status is cleared)
	//INT_ANYRD_2CLEAR	= 1 (Interrupt status is cleared only by reading )
	//ACTL_FSYNC	= 0 (The logic level for the FSYNC pin as an interrupt is active high)
	//FSYNC_INT_MODE_EN	= 0 (This disables the FSYNC pin from causing an interrupt)
	//BYPASS_EN		= 1 
	
	//WOM_EN			= 0 (function is disabled)
	//FIFO_OVERFLOW_EN	= 0 (function is disabled)
	//FSYNC_INT_EN		= 0 (function is disabled)
	//RAW_RDY_EN		= 1 (Enable Raw Sensor Data Ready interrupt to propagate to interrupt pin. The timing of the interrupt can vary depending on the setting in register 36 I2C_MST_CTRL, bit [6] WAIT_FOR_ES.)
	mpu9250_twi_write_reg(chip,MPU9250_INT_PIN_CNFG_REG,0x10);
	mpu9250_twi_write_reg(chip,MPU9250_INT_EN_REG,0x01);
	
// 	//Sets up the external interrupt stuff
// 	MPU9250_INT_PIN_PORT.INT0MASK = 1<<MPU9250_INT_PIN_NUM;
// 	
// 	MPU9250_INT_PIN_PORT.INTCTRL = MPU9250_INT_LVL;
	
}

//odr is the output data rate in Hz
void init_mpu9250(mpu9250_t chip,uint8_t odr)
{
	mpu9250_twi_write_reg(chip,MPU9250_PWR_MGMT_1_REG,0b10000000);//Resets the IMU
	delay_ms(100);
	mpu9250_twi_write_reg(chip,MPU9250_PWR_MGMT_1_REG,0x01);	//Clock Source
	mpu9250_twi_write_reg(chip,MPU9250_PWR_MGMT_2_REG,0x00);	//enables the Acc & Gyro
	
	mpu9250_gyro_init(chip);
	mpu9250_acc_init(chip);
	mpu9250_ODR_init(chip,odr);
	mpu9250_int_init(chip);
	
	
	mpu9250_mag_init(chip);
	mpu9250_dmp_init(chip);
	
	MPU9250_INT_PIN_PORT.PIN0CTRL =  PORT_OPC_PULLDOWN_gc|PORT_ISC_RISING_gc;
	MPU9250_INT_PIN_PORT.INT0MASK = (1<<MPU9250_INT_PIN_NUM);
	MPU9250_INT_PIN_PORT.INTCTRL = 0b11; //MPU9250_INT_LVL;
}

mpu9250_data_t read_mpu9250(mpu9250_t chip)			//read data from chip
{
	mpu9250_data_t data;
	
	//mpu9250_twi_read_multi(chip, mpu9250_INT_STATUS_REG, 2);//call the int status register to clear the system and read instant
	//while (gpio_pin_is_low(chip.int_pin));//wait for something to read (interrupt low = not interrupting, no data)
	mpu9250_twi_read_multi(chip, MPU9250_INT_STATUS_REG,22); //21 = 20 regs + 1 because of atmel bullshit
	data.int_status = mpu9250_twi_data[0];
	data.acc_x = ((int16_t)mpu9250_twi_data[1])<<8;//gyro x high
	data.acc_x += (int16_t)mpu9250_twi_data[2];
	
	data.acc_y = ((int16_t)mpu9250_twi_data[3])<<8;
	data.acc_y += (int16_t)mpu9250_twi_data[4];
	
	data.acc_z = ((int16_t)mpu9250_twi_data[5])<<8;
	data.acc_z += (int16_t)mpu9250_twi_data[6];
	
	data.imu_temperature = ((int16_t)mpu9250_twi_data[7])<<8;
	data.imu_temperature += (int16_t)mpu9250_twi_data[8];
	
	data.gyro_x = ((int16_t)mpu9250_twi_data[9])<<8;
	data.gyro_x += (int16_t)mpu9250_twi_data[10];
	
	data.gyro_y = ((int16_t)mpu9250_twi_data[11])<<8;
	data.gyro_y += (int16_t)mpu9250_twi_data[12];
	
	data.gyro_z = ((int16_t)mpu9250_twi_data[13])<<8;
	data.gyro_z += (int16_t)mpu9250_twi_data[14];
	
	data.mag_x = ((int16_t)mpu9250_twi_data[16])<<8;
	data.mag_x += (int16_t)mpu9250_twi_data[15];
		
	data.mag_y = ((int16_t)mpu9250_twi_data[18])<<8;
	data.mag_y += (int16_t)mpu9250_twi_data[17];
		
	data.mag_z = ((int16_t)mpu9250_twi_data[20])<<8;
	data.mag_z += (int16_t)mpu9250_twi_data[19];
		
	//calibration (observed per chip) [[should be done on-chip according to IMU datasheet; we'll see]]
	//TODO: Manual calibration of all sensors implement self test features
	data.acc_x = (int16_t)(((float)data.acc_x)/ACC_SENSITIVITY);
	data.acc_y = (int16_t)(((float)data.acc_y)/ACC_SENSITIVITY);
	data.acc_z = (int16_t)(((float)data.acc_z)/ACC_SENSITIVITY);
	
	data.gyro_x = (int16_t)(((float)data.gyro_x)/GYRO_SENSITIVITY);
	data.gyro_y = (int16_t)(((float)data.gyro_y)/GYRO_SENSITIVITY);
	data.gyro_z = (int16_t)(((float)data.gyro_z)/GYRO_SENSITIVITY);
	
	data.mag_x = (int16_t)(((float)data.mag_x)*ASAX*6);
	data.mag_y = (int16_t)(((float)data.mag_y)*ASAY*6);
	data.mag_z = (int16_t)(((float)data.mag_z)*ASAZ*6);
	
	data.imu_temperature = (int16_t)(((float)data.imu_temperature)/3.3387)+2100;
	
	return data;
}

bool is_data_rdy(void)
{
	if(data_rdy_flag)
	{
		data_rdy_flag = 0;
		return true;
	}
	else
	{
		return false;
	}
}

ISR(MPU9250_INT_vect)
{
	//printf("jkeavrf\n");
	data_rdy_flag = 1;
	timerflag = 1;
	time++;
}

uint8_t who_am_i(mpu9250_t chip)
{
	mpu9250_twi_read_multi(chip,MPU9250_WHO_AM_I_REG, 2);
	return  mpu9250_twi_data[0];
}


//reset mission time
void reset_mission_time(void)
{
	time = 0;
}

//returns the time in counts
uint32_t what_time_is_it(void)
{
	return time;
}

//returns true if the timer interrupt flag is set (use for time keeping)
uint8_t is_sample_timer_flag_set(void)
{
	if(timerflag)
	{
		timerflag = 0;
		return 1;
	}
	else
	{
		return 0;
	}
}
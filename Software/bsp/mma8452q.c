/******************************************************************************
 
This is a C-version of the SparkFun Arduino Library: https://github.com/sparkfun/SparkFun_MMA8452Q_Arduino_Library

Original Header:
================
SparkFun_MMA8452Q.c
SparkFun_MMA8452Q Library Header File
Jim Lindblom and Andrea DeVore @ SparkFun Electronics
Original Creation Date: June 3, 2014
https://github.com/sparkfun/MMA8452_Accelerometer

This file prototypes the MMA8452Q class, implemented in SFE_MMA8452Q.cpp. In
addition, it defines every register in the MMA8452Q.

Development environment specifics:
	IDE: Arduino 1.0.5
	Hardware Platform: Arduino Uno

	**Updated for Arduino 1.8.5 2/2019**

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "mma8452q.h"

/**
 * @brief SET STANDBY MODE
 *	Sets the MMA8452 to standby mode. It must be in standby to change most register settings 
 */
void standby(mma8452_t* acc);

/**
 * @brief SET ACTIVE MODE
 *	Sets the MMA8452 to active mode. Needs to be in this mode to output data 
 */
void active(mma8452_t* acc);

/**
 * @brief CHECK STATE (ACTIVE or STANDBY)
 *	Returns true if in Active State, otherwise return false
 * 
 * @return true if in Active State.
 * @return false if in Standby state.
 */
bool isActive(mma8452_t* acc);

/**
 * @brief SET UP PORTRAIT/LANDSCAPE DETECTION
 *	This function sets up portrait and landscape detection.
 */
void setupPL(mma8452_t* acc);

/**
 * @brief SET UP TAP DETECTION
 *	This function can set up tap detection on the x, y, and/or z axes.
 *	The xThs, yThs, and zThs parameters serve two functions:
 *		1. Enable tap detection on an axis. If the 7th bit is SET (0x80)
 *			tap detection on that axis will be DISABLED.
 *		2. Set tap g's threshold. The lower 7 bits will set the tap threshold
 *			on that axis. 
 * @param xThs Bit 7 disables tap. Lower bits set tap detection threshold for X-axis.
 * @param yThs Bit 7 disables tap. Lower bits set tap detection threshold for Y-axis.
 * @param zThs Bit 7 disables tap. Lower bits set tap detection threshold for Z-axis.
 */
void setupTap(mma8452_t* acc, uint8_t xThs, uint8_t yThs, uint8_t zThs);

/**
 * @brief WRITE A SINGLE REGISTER
 * 	Write a single byte of data to a register in the MMA8452Q.
 * 
 * @param reg Register to write to.
 * @param data Data to write to the register.
 */
void writeRegister(mma8452_t* acc, uint8_t reg, uint8_t data);

/**
 * @brief READ A SINGLE REGISTER
 *	Read a byte from the MMA8452Q register "reg".
 * 
 * @param reg Register to read.
 * @return uint8_t Read register content.
 */
uint8_t readRegister(mma8452_t* acc, uint8_t reg);

/**
 * @brief READ MULTIPLE REGISTERS
 *	Read "len" bytes from the MMA8452Q, starting at register "reg". Bytes are stored
 *	in "buffer" on exit.
 * 
 * @param reg Register address to read from.
 * @param buffer Destination buffer for the read data.
 * @param len Number of bytes to read.
 */
void readRegisters(mma8452_t* acc, uint8_t reg, uint8_t *buffer, uint8_t len);

/**
 * @brief Converts a two's complement value to a signed integer.
 * @param val The 12-bit value to convert.
 * @return The signed integer.
 */
int16_t twos_comp_to_int16(uint16_t val);

/*************************************************************************************/

bool mma8452q_init(mma8452_t* acc) {

    if (acc->address == 0x00) {
        acc->address = MMA8452Q_DEFAULT_ADDRESS;
    }
    uint8_t c = readRegister(acc, MMA8452Q_WHO_AM_I); /* Read WHO_AM_I register. */

	if (c != 0x2A) /* WHO_AM_I should always be 0x2A. */
	{
		return false;
	}

	acc->scale = SCALE_2G;
	acc->odr = ODR_800;

	mma8452q_setScale(acc, acc->scale);   /* Set up accelerometer scale. */
	mma8452q_setDataRate(acc, acc->odr);  /* Set up output data rate. */
	setupPL(acc);		            /* Set up portrait/landscape detection. */

	/* Multiply parameter by 0.0625g to calculate threshold. */
	setupTap(acc, 0x80, 0x80, 0x08); /* Disable x, y, set z to 0.5g. */

    active(acc); /* Set to active to start reading. */

	return true;
}
/*-----------------------------------------------------------*/

void mma8452q_read(mma8452_t* acc) {
    uint8_t rawData[6]; /* x/y/z accel register data stored here. */

	readRegisters(acc, MMA8452Q_OUT_X_MSB, rawData, 6); /* Read the six raw data registers into data array. */

	acc->x = ((uint16_t)(rawData[0] << 8 | rawData[1])) >> 4;
	acc->y = ((uint16_t)(rawData[2] << 8 | rawData[3])) >> 4;
	acc->z = ((uint16_t)(rawData[4] << 8 | rawData[5])) >> 4;

	int16_t tmp = twos_comp_to_int16(acc->x);
	float factor = (float)(1 << 11) / (float)(acc->scale);
	acc->cx = tmp / factor;

	tmp = twos_comp_to_int16(acc->y);
	acc->cy = tmp / factor;

	tmp = twos_comp_to_int16(acc->z);
	acc->cz = tmp / factor;
}
/*-----------------------------------------------------------*/

uint8_t mma8452q_available(mma8452_t* acc) {
    return (readRegister(acc, MMA8452Q_F_STATUS) & 0x08) >> 3;
}
/*-----------------------------------------------------------*/

uint8_t mma8452q_readTap(mma8452_t* acc) {
    uint8_t tapStat = readRegister(acc, MMA8452Q_PULSE_SRC);
	if (tapStat & 0x80) /* Read EA bit to check if a interrupt was generated. */
	{
		return tapStat & 0x7F;
	}
	else
		return 0;
}
/*-----------------------------------------------------------*/

uint8_t mma8452q_readPL(mma8452_t* acc) {

    uint8_t plStat = readRegister(acc, MMA8452Q_PL_STATUS);

	if (plStat & 0x40) /* Z-tilt lockout. */
		return MMA8452Q_LOCKOUT;
	else /* Otherwise return LAPO status. */
		return (plStat & 0x6) >> 1;
}
/*-----------------------------------------------------------*/

uint8_t mma8452q_readID(mma8452_t* acc) {
    return readRegister(acc, MMA8452Q_WHO_AM_I);
}
/*-----------------------------------------------------------*/

uint16_t mma8452q_getX(mma8452_t* acc) {
    uint8_t rawData[2];
	readRegisters(acc, MMA8452Q_OUT_X_MSB, rawData, 2); /* Read the X data into a data array. */
	return ((short)(rawData[0] << 8 | rawData[1])) >> 4;
}
/*-----------------------------------------------------------*/

uint16_t mma8452q_getY(mma8452_t* acc) {
    uint8_t rawData[2];
	readRegisters(acc, MMA8452Q_OUT_Y_MSB, rawData, 2); /* Read the X data into a data array. */
	return ((short)(rawData[0] << 8 | rawData[1])) >> 4;
}
/*-----------------------------------------------------------*/

uint16_t mma8452q_getZ(mma8452_t* acc) {
    uint8_t rawData[2];
	readRegisters(acc, MMA8452Q_OUT_Z_MSB, rawData, 2); /* Read the X data into a data array. */
	return ((short)(rawData[0] << 8 | rawData[1])) >> 4;
}
/*-----------------------------------------------------------*/

float mma8452q_getCalculatedX(mma8452_t* acc) {
    acc->x = mma8452q_getX(acc);

	int16_t tmp = twos_comp_to_int16(acc->x);
	float factor = (float)(1 << 11) / (float)(acc->scale);

	return (float) tmp / factor;
}
/*-----------------------------------------------------------*/

float mma8452q_getCalculatedY(mma8452_t* acc) {
    acc->y = mma8452q_getY(acc);

	int16_t tmp = twos_comp_to_int16(acc->y);
	float factor = (float)(1 << 11) / (float)(acc->scale);

	return (float) tmp / factor;
}
/*-----------------------------------------------------------*/

float mma8452q_getCalculatedZ(mma8452_t* acc) {
    acc->z = mma8452q_getZ(acc);

	int16_t tmp = twos_comp_to_int16(acc->z);
	float factor = (float)(1 << 11) / (float)(acc->scale);

	return (float) tmp / factor;
}
/*-----------------------------------------------------------*/

bool mma8452q_isRight(mma8452_t* acc) {
   if (mma8452q_readPL(acc) == MMA8452Q_LANDSCAPE_R)
		return true;
	return false;
}
/*-----------------------------------------------------------*/

bool mma8452q_isLeft(mma8452_t* acc) {
    if (mma8452q_readPL(acc) == MMA8452Q_LANDSCAPE_L)
		return true;
	return false;
}
/*-----------------------------------------------------------*/

bool mma8452q_isUp(mma8452_t* acc) {
    if (mma8452q_readPL(acc) == MMA8452Q_PORTRAIT_U)
		return true;
	return false;
}
/*-----------------------------------------------------------*/

bool mma8452q_isDown(mma8452_t* acc) {
    if (mma8452q_readPL(acc) == MMA8452Q_PORTRAIT_D)
		return true;
	return false;
}
/*-----------------------------------------------------------*/

bool mma8452q_isFlat(mma8452_t* acc) {
    if (mma8452q_readPL(acc) == MMA8452Q_LOCKOUT)
		return true;
	return false;
}
/*-----------------------------------------------------------*/


void mma8452q_setScale(mma8452_t* acc, MMA8452Q_Scale_t fsr) {
    /* Must be in standby mode to make changes!!!
	   Change to standby if currently in active state. */
	if (isActive(acc) == true)
		standby(acc);

	uint8_t cfg = readRegister(acc, MMA8452Q_XYZ_DATA_CFG);
	cfg &= 0xFC;	   /* Mask out scale bits. */
	cfg |= (fsr >> 2); /* Neat trick, see page 22. 00 = 2G, 01 = 4A, 10 = 8G. */
	writeRegister(acc, MMA8452Q_XYZ_DATA_CFG, cfg);

	/* Return to active state when done.
	   Must be in active state to read data. */
	active(acc);
}
/*-----------------------------------------------------------*/

void mma8452q_setDataRate(mma8452_t* acc, MMA8452Q_ODR_t odr) {
    /* Must be in standby mode to make changes!!!
	   Change to standby if currently in active state. */
	if (isActive(acc) == true)
		standby(acc);

	uint8_t ctrl = readRegister(acc, MMA8452Q_CTRL_REG1);
	ctrl &= 0xC7; /* Mask out data rate bits. */
	ctrl |= (odr << 3);
	writeRegister(acc, MMA8452Q_CTRL_REG1, ctrl);

	/* Return to active state when done.
	   Must be in active state to read data. */
	active(acc);
}
/*-----------------------------------------------------------*/

void standby(mma8452_t* acc) {
    uint8_t c = readRegister(acc, MMA8452Q_CTRL_REG1);
	writeRegister(acc, MMA8452Q_CTRL_REG1, c & ~(0x01)); /* Clear the active bit to go into standby. */
}
/*-----------------------------------------------------------*/

void active(mma8452_t* acc) {
    uint8_t c = readRegister(acc, MMA8452Q_CTRL_REG1);
	writeRegister(acc, MMA8452Q_CTRL_REG1, c | 0x01); /* Set the active bit to begin detection. */
}
/*-----------------------------------------------------------*/

bool isActive(mma8452_t* acc) {
    uint8_t currentState = readRegister(acc, MMA8452Q_SYSMOD);
	currentState &= 0b00000011;

	/* Wake and Sleep are both active SYSMOD states (pg. 10 datasheet). */
	if (currentState == MMA8452Q_SYSMOD_STANDBY)
		return false;
	return true;
}
/*-----------------------------------------------------------*/

void setupPL(mma8452_t* acc) {
    /* Must be in standby mode to make changes!!!
	   Change to standby if currently in active state. */
	if (isActive(acc) == true)
		standby(acc);

	/* For more info check out this app note:
	  	http://cache.freescale.com/files/sensors/doc/app_note/AN4068.pdf
	   1. Enable P/L writeRegister(acc, MMA8452Q_PL_CFG, readRegister(acc, MMA8452Q_PL_CFG) | 0x40); 
	      Set PL_EN (enable).
	   2. Set the debounce rate writeRegister(acc, MMA8452Q_PL_COUNT, 0x50); 
	      Debounce counter at 100ms (at 800 hz). */

	/* Return to active state when done.
	   Must be in active state to read data. */
	active(acc);
}
/*-----------------------------------------------------------*/

void setupTap(mma8452_t* acc, uint8_t xThs, uint8_t yThs, uint8_t zThs) {
    /* Must be in standby mode to make changes!!!
	   Change to standby if currently in active state. */
	if (isActive(acc) == true)
		standby(acc);

	/* Set up single and double tap - 5 steps:
	   for more info check out this app note:
	   http://cache.freescale.com/files/sensors/doc/app_note/AN4072.pdf
	   Set the threshold - minimum required acceleration to cause a tap. */
	uint8_t temp = 0;
	if (!(xThs & 0x80)) /* If top bit ISN'T set. */
	{
		temp |= 0x3;					 				/* Enable taps on x. */
		writeRegister(acc, MMA8452Q_PULSE_THSX, xThs); 	/* x thresh. */
	}
	if (!(yThs & 0x80))
	{
		temp |= 0xC;					 				/* Enable taps on y. */
		writeRegister(acc, MMA8452Q_PULSE_THSY, yThs);  /* y thresh. */
	}
	if (!(zThs & 0x80))
	{
		temp |= 0x30;					 				/* Enable taps on z. */
		writeRegister(acc, MMA8452Q_PULSE_THSZ, zThs); 	/* z thresh. */
	}
	
	/* Set up single and/or double tap detection on each axis individually. */
	writeRegister(acc, MMA8452Q_PULSE_CFG, temp | 0x40);
	
	/* Set the time limit - the maximum time that a tap can be above the thresh. */
	writeRegister(acc, MMA8452Q_PULSE_TMLT, 0x30); /* 30ms time limit at 800Hz odr. */

	/* Set the pulse latency - the minimum required time between pulses. */
	writeRegister(acc, MMA8452Q_PULSE_LTCY, 0xA0); /* 200ms (at 800Hz odr) between taps min. */

	/* Set the second pulse window - maximum allowed time between end of
	   latency and start of second pulse. */
	writeRegister(acc, MMA8452Q_PULSE_WIND, 0xFF); /* 5. 318ms (max value) between taps max. */

	/* Return to active state when done.
	   Must be in active state to read data. */
	active(acc);
}
/*-----------------------------------------------------------*/

void writeRegister(mma8452_t* acc, uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = data;
    
    int ret = i2c_write_blocking(I2C_PORT, acc->address, buf, 2, false);
}

/*-----------------------------------------------------------*/

uint8_t readRegister(mma8452_t* acc, uint8_t reg) {
    uint8_t data;

    readRegisters(acc, reg, &data, 1);
    
    return data;
}
/*-----------------------------------------------------------*/

void readRegisters(mma8452_t* acc, uint8_t reg, uint8_t *buffer, uint8_t len) {
    /* First send (device address + write)
       then send register address
       first tell accelerometer which address to read from. */
    i2c_write_blocking(I2C_PORT, acc->address, &reg, 1, true);
    
    /* Then read from accelerometer. */
    i2c_read_blocking(I2C_PORT, acc->address, buffer, len, false); /* false stop bit. */
    
}
/*-----------------------------------------------------------*/

int16_t twos_comp_to_int16(uint16_t val) {
    
    /* Mask to keep only the lowest 12 bits. */
    val &= 0x0FFF;

    /* Check if sign bit (bit 11) is set. */
    if (val & 0x0800) {
        /* If sign bit is set, extend the sign to 16 bits. */
        val |= 0xF000;  /* Set upper 4 bits to 1 for sign extension. */
    }

    return (int16_t)val;
}
/*-----------------------------------------------------------*/
/******************************************************************************
 
This is a C-version of the SparkFun Arduino Library: https://github.com/sparkfun/SparkFun_MMA8452Q_Arduino_Library

Original Header:
================
SparkFun_MMA8452Q.h
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

#ifndef MMA8452Q_NEW_H
#define MMA8452Q_NEW_H

/**
 * @brief Default I2C port to use.
 */
#ifndef I2C_PORT
#define I2C_PORT    i2c0
#endif

/**
 * @brief Default I2C address.
 */
#define MMA8452Q_DEFAULT_ADDRESS 0x1C

/**
 * @brief MMA8452Q Accelerometer register addresses
 */
#define MMA8452Q_F_STATUS 0x00
#define MMA8452Q_OUT_X_MSB 0x01 
#define MMA8452Q_OUT_X_LSB 0x02
#define MMA8452Q_OUT_Y_MSB 0x03
#define MMA8452Q_OUT_Y_LSB 0x04
#define MMA8452Q_OUT_Z_MSB 0x05
#define MMA8452Q_OUT_Z_LSB 0x06
#define MMA8452Q_SYSMOD 0x0B
#define MMA8452Q_INT_SOURCE 0x0C
#define MMA8452Q_WHO_AM_I 0x0D
#define MMA8452Q_XYZ_DATA_CFG 0x0E
#define MMA8452Q_HP_FILTER_CUTOFF 0x0F
#define MMA8452Q_PL_STATUS 0x10
#define MMA8452Q_PL_CFG 0x11
#define MMA8452Q_PL_COUNT 0x12
#define MMA8452Q_PL_BF_ZCOMP 0x13
#define MMA8452Q_P_L_THS_REG 0x14
#define MMA8452Q_FF_MT_CFG 0x15
#define MMA8452Q_FF_MT_SRC 0x16
#define MMA8452Q_FF_MT_THS 0x17
#define MMA8452Q_FF_MT_COUNT 0x18
#define MMA8452Q_TRANSIENT_CFG 0x1D
#define MMA8452Q_TRANSIENT_SRC 0x1E
#define MMA8452Q_TRANSIENT_THS 0x1F
#define MMA8452Q_TRANSIENT_COUNT 0x20
#define MMA8452Q_PULSE_CFG 0x21
#define MMA8452Q_PULSE_SRC 0x22
#define MMA8452Q_PULSE_THSX 0x23
#define MMA8452Q_PULSE_THSY 0x24
#define MMA8452Q_PULSE_THSZ 0x25
#define MMA8452Q_PULSE_TMLT 0x26
#define MMA8452Q_PULSE_LTCY 0x27
#define MMA8452Q_PULSE_WIND 0x28
#define MMA8452Q_ALSP_COUNT 0x29
#define MMA8452Q_CTRL_REG1 0x2A
#define MMA8452Q_CTRL_REG2 0x2B
#define MMA8452Q_CTRL_REG3 0x2C
#define MMA8452Q_CTRL_REG4 0x2D
#define MMA8452Q_CTRL_REG5 0x2E
#define MMA8452Q_OFF_X 0x2F
#define MMA8452Q_OFF_Y 0x30
#define MMA8452Q_OFF_Z 0x31

/**
 * @brief Possible acceleration range configurations of the sensor.
 */
#define ACCEL_RANGE_2G      0b00
#define ACCEL_RANGE_4G      0b01
#define ACCEL_RANGE_8G      0b10

#define CURRENT_ACCEL_RANGE ACCEL_RANGE_2G

/**
 * @brief Possible portrait/landscape settings
 */
#define MMA8452Q_PORTRAIT_U 0
#define MMA8452Q_PORTRAIT_D 1
#define MMA8452Q_LANDSCAPE_R 2
#define MMA8452Q_LANDSCAPE_L 3
#define MMA8452Q_LOCKOUT 0x40

/**
 * @brief Posible SYSMOD (system mode) States
 */
#define MMA8452Q_SYSMOD_STANDBY 0b00
#define MMA8452Q_SYSMOD_WAKE 0b01
#define MMA8452Q_SYSMOD_SLEEP 0b10

/**
 * @brief Possible scale values of the sensor.
 * 
 */
typedef enum {
	SCALE_2G = 2,
	SCALE_4G = 4,
	SCALE_8G = 8
} MMA8452Q_Scale_t;

/**
 * @brief `Possible data rates of the sensor
 */
typedef enum {
	ODR_800,
	ODR_400,
	ODR_200,
	ODR_100,
	ODR_50,
	ODR_12,
	ODR_6,
	ODR_1
} MMA8452Q_ODR_t; 

/**
 * @brief Describes a specific sensor instance.
 */
typedef struct {
    uint8_t address;
    MMA8452Q_Scale_t scale;
    MMA8452Q_ODR_t odr;
    uint16_t x;
    uint16_t y;
    uint16_t z;
    float cx;
    float cy;
    float cz;
} mma8452_t;

bool mma8452q_init(mma8452_t* acc);

/**
 * @brief READ ACCELERATION DATA
 *  This function will read the acceleration values from the MMA8452Q. After
 *	reading, it will update two triplets of variables:
 *		- int's x, y, and z will store the signed 12-bit values read out
 *		  of the acceleromter.
 *		- floats cx, cy, and cz will store the calculated acceleration from
 *		  those 12-bit values. These variables are in units of g's.
 * 
 * @param acc Sensor instance.
 */
void mma8452q_read(mma8452_t* acc);

/**
 * @brief CHECK IF NEW DATA IS AVAILABLE
 *	This function checks the status of the MMA8452Q to see if new data is availble.
 *	returns 0 if no new data is present, or a 1 if new data is available.
 * 
 * @param acc Sensor instance.
 * @return uint8_t 0 if no new data is present, or a 1 if new data is available.
 */
uint8_t mma8452q_available(mma8452_t* acc);

/**
 * @brief READ TAP STATUS
 *	This function returns any taps read by the MMA8452Q. 
 * 
 * @param acc Sensor instance.
 * @return uint8_t If the function returns 0, no new taps were detected. Otherwise the function will return the lower 7 bits of the PULSE_SRC register.
 */
uint8_t mma8452q_readTap(mma8452_t* acc);

/**
 * @brief READ PORTRAIT/LANDSCAPE STATUS
 *	This function reads the portrait/landscape status register of the MMA8452Q.
 *	It will return either PORTRAIT_U, PORTRAIT_D, LANDSCAPE_R, LANDSCAPE_L,
 *	or LOCKOUT. LOCKOUT indicates that the sensor is in neither p or ls.
 * 
 * @param acc Sensor instance.
 * @return uint8_t Returns the orientation.
 */
uint8_t mma8452q_readPL(mma8452_t* acc);

/**
 * @brief Read the device ID.
 * 
 * @param acc Sensor instance. 
 * @return uint8_t 
 */
uint8_t mma8452q_readID(mma8452_t* acc);

/**
 * @brief GET FUNCTIONS FOR RAW ACCELERATION DATA
 * Returns raw X acceleration data
 * 
 * @param acc Sensor instance. 
 * @return uint16_t Raw X acceleration data
 */
uint16_t mma8452q_getX(mma8452_t* acc);

/**
 * @brief GET FUNCTIONS FOR RAW ACCELERATION DATA
 * Returns raw Y acceleration data
 * 
 * @param acc Sensor instance. 
 * @return uint16_t Raw Y acceleration data
 */
uint16_t mma8452q_getY(mma8452_t* acc);

/**
 * @brief GET FUNCTIONS FOR RAW ACCELERATION DATA
 * Returns raw Z acceleration data
 * 
 * @param acc Sensor instance. 
 * @return uint16_t Raw Z acceleration data
 */
uint16_t mma8452q_getZ(mma8452_t* acc);

/**
 * @brief GET FUNCTIONS FOR CALCULATED ACCELERATION DATA
 * Returns calculated X acceleration data
 * 
 * @param acc Sensor instance. 
 * @return float Calculated X acceleration data.
 */
float mma8452q_getCalculatedX(mma8452_t* acc);

/**
 * @brief GET FUNCTIONS FOR CALCULATED ACCELERATION DATA
 * Returns calculated Y acceleration data
 * 
 * @param acc Sensor instance. 
 * @return float Calculated Y acceleration data.
 */
float mma8452q_getCalculatedY(mma8452_t* acc);

/**
 * @brief GET FUNCTIONS FOR CALCULATED ACCELERATION DATA
 * Returns calculated Z acceleration data
 * 
 * @param acc Sensor instance. 
 * @return float Calculated Z acceleration data.
 */
float mma8452q_getCalculatedZ(mma8452_t* acc);

/**
 * @brief Check if device in right orientation.
 * 
 * @param acc Sensor instance. 
 * @return true 
 * @return false 
 */
bool mma8452q_isRight(mma8452_t* acc);

/**
 * @brief Check if device in left orientation.
 * 
 * @param acc Sensor instance. 
 * @return true 
 * @return false 
 */
bool mma8452q_isLeft(mma8452_t* acc);

/**
 * @brief Check if device in upright orientation.
 * 
 * @param acc Sensor instance. 
 * @return true 
 * @return false 
 */
bool mma8452q_isUp(mma8452_t* acc);

/**
 * @brief Check if device in down orientation.
 * 
 * @param acc Sensor instance. 
 * @return true 
 * @return false 
 */
bool mma8452q_isDown(mma8452_t* acc);

/**
 * @brief Check if device in flat orientation.
 * 
 * @param acc Sensor instance. 
 * @return true 
 * @return false 
 */
bool mma8452q_isFlat(mma8452_t* acc);

/**
 * @brief SET FULL-SCALE RANGE
 *	This function sets the full-scale range of the x, y, and z axis accelerometers.
 *	Possible values for the fsr variable are SCALE_2G, SCALE_4G, or SCALE_8G.
 * 
 * @param acc Sensor instance. 
 * @param fsr Possible values for the fsr variable are SCALE_2G, SCALE_4G, or SCALE_8G.
 */
void mma8452q_setScale(mma8452_t* acc, MMA8452Q_Scale_t fsr);

/**
 * @brief SET THE OUTPUT DATA RATE
 *	This function sets the output data rate of the MMA8452Q.
 *	Possible values for the odr parameter are: ODR_800, ODR_400, ODR_200,
 *	ODR_100, ODR_50, ODR_12, ODR_6, or ODR_1
 * 
 * @param acc Sensor instance. 
 * @param odr Output data rate.
 */
void mma8452q_setDataRate(mma8452_t* acc, MMA8452Q_ODR_t odr);

#endif /* MMA8452Q_NEW_H */

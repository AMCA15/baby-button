/******************************************************************************
SFE_LSM9DS1.h
SFE_LSM9DS1 Library Header File
Jim Lindblom @ SparkFun Electronics
Original Creation Date: February 27, 2015
https://github.com/sparkfun/LSM9DS1_Breakout

This file prototypes the LSM9DS1 class, implemented in SFE_LSM9DS1.cpp. In
addition, it defines every register in the LSM9DS1 (both the Gyro and Accel/
Magnetometer registers).

Development environment specifics:
	IDE: Arduino 1.6.0
	Hardware Platform: Arduino Uno
	LSM9DS1 Breakout Version: 1.0

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
------------------------------------------------------------------------------
- Migrated from C++ to C and ported to NRF52 devices - 10/4/2020
  Anderson Contreras <acontreras@aimonkey.info>
******************************************************************************/

#ifndef __SparkFunLSM9DS1_H__
#define __SparkFunLSM9DS1_H__

#include "LSM9DS1_Registers.h"
#include "LSM9DS1_Types.h"

#include <stdint.h>
#include <nrfx_twim.h>

#define LSM9DS1_AG_ADDR(sa0)	((sa0) == 0 ? 0x6A : 0x6B)
#define LSM9DS1_M_ADDR(sa1)		((sa1) == 0 ? 0x1C : 0x1E)

typedef enum{
	X_AXIS,
	Y_AXIS,
	Z_AXIS,
	ALL_AXIS
} lsm9ds1_axis;


typedef struct {
	struct IMUSettings settings;
	// We'll store the gyro, accel, and magnetometer readings in a series of
	// public class variables. Each sensor gets three variables -- one for each
	// axis. Call lsm9ds1_readGyro(), lsm9ds1_readAccel(), and lsm9ds1_readMag() first, before using
	// these variables!
	// These values are the RAW signed 16-bit readings from the sensors.
	int16_t gx, gy, gz;     // x, y, and z axis readings of the gyroscope
	int16_t ax, ay, az;     // x, y, and z axis readings of the accelerometer
	int16_t mx, my, mz;     // x, y, and z axis readings of the magnetometer
    int16_t temperature;    // Chip temperature
	float gBias[3], aBias[3], mBias[3];
	int16_t gBiasRaw[3], aBiasRaw[3], mBiasRaw[3];
	
	// x_mAddress and gAddress store the I2C address or SPI chip select pin
	// for each sensor.
	uint8_t _mAddress, _xgAddress;
	
	// gRes, aRes, and mRes store the current resolution for each sensor. 
	// Units of these values would be DPS (or g's or Gs's) per ADC tick.
	// This value is calculated as (sensor scale) / (2^15).
	float gRes, aRes, mRes;
	
	// _autoCalc keeps track of whether we're automatically subtracting off
	// accelerometer and gyroscope bias calculated in lsm9ds1_calibrate().
	bool _autoCalc;
} lsm9ds1_t;


// lsm9ds1_begin() and beginSPI() -- Initialize the gyro, accelerometer, and magnetometer.
// This will set up the scale and output rate of each sensor. The values set
// in the IMUSettings struct will take effect after calling this function.
// INPUTS:
// - agAddress - Sets either the I2C address of the accel/gyro or SPI chip 
//   select pin connected to the CS_XG pin.
// - mAddress - Sets either the I2C address of the magnetometer or SPI chip 
//   select pin connected to the CS_M pin.
// - i2C port (Note, only on "lsm9ds1_begin()" funtion, for use with I2C com interface)
//   defaults to Wire, but if hardware supports it, can use other TwoWire ports.
//   **For SPI use "beginSPI()", and only send first two address arguments.
uint16_t lsm9ds1_begin(lsm9ds1_t *lsm9ds1, nrfx_twim_t *wirePort, uint8_t agAddress, uint8_t mAddress);

void lsm9ds1_calibrate(lsm9ds1_t *lsm9ds1, bool autoCalc);
void lsm9ds1_calibrateMag(lsm9ds1_t *lsm9ds1, bool loadIn);
void lsm9ds1_magOffset(lsm9ds1_t *lsm9ds1, uint8_t axis, int16_t offset);

// lsm9ds1_accelAvailable() -- Polls the accelerometer status register to check
// if new data is available.
// Output:	1 - New data available
//			0 - No new data available
uint8_t lsm9ds1_accelAvailable(lsm9ds1_t *lsm9ds1);

// lsm9ds1_gyroAvailable() -- Polls the gyroscope status register to check
// if new data is available.
// Output:	1 - New data available
//			0 - No new data available
uint8_t lsm9ds1_gyroAvailable(lsm9ds1_t *lsm9ds1);

// lsm9ds1_tempAvailable() -- Polls the temperature status register to check
// if new data is available.
// Output:	1 - New data available
//			0 - No new data available
uint8_t lsm9ds1_tempAvailable(lsm9ds1_t *lsm9ds1);

// lsm9ds1_magAvailable() -- Polls the accelerometer status register to check
// if new data is available.
// Input:
//	- axis can be either X_AXIS, Y_AXIS, Z_AXIS, to check for new data
//	  on one specific axis. Or ALL_AXIS (default) to check for new data
//	  on all axes.
// Output:	1 - New data available
//			0 - No new data available
uint8_t lsm9ds1_magAvailable(lsm9ds1_t *lsm9ds1, lsm9ds1_axis axis);

// lsm9ds1_readGyro() -- Read the gyroscope output registers.
// This function will read all six gyroscope output registers.
// The readings are stored in the class' gx, gy, and gz variables. Read
// those _after_ calling lsm9ds1_readGyro().
void lsm9ds1_readGyro(lsm9ds1_t *lsm9ds1);

// int16_t lsm9ds1_readGyro(axis) -- Read a specific axis of the gyroscope.
// [axis] can be any of X_AXIS, Y_AXIS, or Z_AXIS.
// Input:
//	- axis: can be either X_AXIS, Y_AXIS, or Z_AXIS.
// Output:
//	A 16-bit signed integer with sensor data on requested axis.
int16_t lsm9ds1_readGyro_ax(lsm9ds1_t *lsm9ds1, lsm9ds1_axis axis);

// lsm9ds1_readAccel() -- Read the accelerometer output registers.
// This function will read all six accelerometer output registers.
// The readings are stored in the class' ax, ay, and az variables. Read
// those _after_ calling lsm9ds1_readAccel().
void lsm9ds1_readAccel(lsm9ds1_t *lsm9ds1);

// int16_t lsm9ds1_readAccel(axis) -- Read a specific axis of the accelerometer.
// [axis] can be any of X_AXIS, Y_AXIS, or Z_AXIS.
// Input:
//	- axis: can be either X_AXIS, Y_AXIS, or Z_AXIS.
// Output:
//	A 16-bit signed integer with sensor data on requested axis.
int16_t lsm9ds1_readAccel_ax(lsm9ds1_t *lsm9ds1, lsm9ds1_axis axis);

// lsm9ds1_readMag() -- Read the magnetometer output registers.
// This function will read all six magnetometer output registers.
// The readings are stored in the class' mx, my, and mz variables. Read
// those _after_ calling lsm9ds1_readMag().
void lsm9ds1_readMag(lsm9ds1_t *lsm9ds1);

// int16_t lsm9ds1_readMag(axis) -- Read a specific axis of the magnetometer.
// [axis] can be any of X_AXIS, Y_AXIS, or Z_AXIS.
// Input:
//	- axis: can be either X_AXIS, Y_AXIS, or Z_AXIS.
// Output:
//	A 16-bit signed integer with sensor data on requested axis.
int16_t lsm9ds1_readMag_ax(lsm9ds1_t *lsm9ds1, lsm9ds1_axis axis);

// lsm9ds1_readTemp() -- Read the temperature output register.
// This function will read two temperature output registers.
// The combined readings are stored in the class' temperature variables. Read
// those _after_ calling lsm9ds1_readTemp().
void lsm9ds1_readTemp(lsm9ds1_t *lsm9ds1);

// lsm9ds1_readAccelGyro_burst() -- Read the accelerometer and magnetometer output registers in burst.
// This function will read all six magnetometer output registers n (length) times.
// The readings are stored in the buffer variable. only_accel: set this option when the gyroscope is inactive
void lsm9ds1_readAccelGyro_burst(lsm9ds1_t *lsm9ds1, uint16_t *buffer, uint8_t length, uint8_t only_accel);

// lsm9ds1_calcGyro() -- Convert from RAW signed 16-bit value to degrees per second
// This function reads in a signed 16-bit value and returns the scaled
// DPS. This function relies on gScale and gRes being correct.
// Input:
//	- gyro = A signed 16-bit raw reading from the gyroscope.
float lsm9ds1_calcGyro(lsm9ds1_t *lsm9ds1, int16_t gyro);

// lsm9ds1_calcAccel() -- Convert from RAW signed 16-bit value to gravity (g's).
// This function reads in a signed 16-bit value and returns the scaled
// g's. This function relies on aScale and aRes being correct.
// Input:
//	- accel = A signed 16-bit raw reading from the accelerometer.
float lsm9ds1_calcAccel(lsm9ds1_t *lsm9ds1, int16_t accel);

// lsm9ds1_calcMag() -- Convert from RAW signed 16-bit value to Gauss (Gs)
// This function reads in a signed 16-bit value and returns the scaled
// Gs. This function relies on mScale and mRes being correct.
// Input:
//	- mag = A signed 16-bit raw reading from the magnetometer.
float lsm9ds1_calcMag(lsm9ds1_t *lsm9ds1, int16_t mag);

// lsm9ds1_setGyroScale() -- Set the full-scale range of the gyroscope.
// This function can be called to set the scale of the gyroscope to 
// 245, 500, or 200 degrees per second.
// Input:
// 	- gScl = The desired gyroscope scale. Must be one of three possible
//		values from the gyro_scale.
void lsm9ds1_setGyroScale(lsm9ds1_t *lsm9ds1, uint16_t gScl);

// lsm9ds1_setAccelScale() -- Set the full-scale range of the accelerometer.
// This function can be called to set the scale of the accelerometer to
// 2, 4, 6, 8, or 16 g's.
// Input:
// 	- aScl = The desired accelerometer scale. Must be one of five possible
//		values from the accel_scale.
void lsm9ds1_setAccelScale(lsm9ds1_t *lsm9ds1, uint8_t aScl);

// lsm9ds1_setMagScale() -- Set the full-scale range of the magnetometer.
// This function can be called to set the scale of the magnetometer to
// 2, 4, 8, or 12 Gs.
// Input:
// 	- mScl = The desired magnetometer scale. Must be one of four possible
//		values from the mag_scale.
void lsm9ds1_setMagScale(lsm9ds1_t *lsm9ds1, uint8_t mScl);

// lsm9ds1_setGyroODR() -- Set the output data rate and bandwidth of the gyroscope
// Input:
//	- gRate = The desired output rate and cutoff frequency of the gyro.
void lsm9ds1_setGyroODR(lsm9ds1_t *lsm9ds1, uint8_t gRate);

// lsm9ds1_setAccelODR() -- Set the output data rate of the accelerometer
// Input:
//	- aRate = The desired output rate of the accel.
void lsm9ds1_setAccelODR(lsm9ds1_t *lsm9ds1, uint8_t aRate); 	

// lsm9ds1_setMagODR() -- Set the output data rate of the magnetometer
// Input:
//	- mRate = The desired output rate of the mag.
void lsm9ds1_setMagODR(lsm9ds1_t *lsm9ds1, uint8_t mRate);

// lsm9ds1_configInactivity() -- Configure inactivity interrupt parameters
// Input:
//	- duration = Inactivity duration - actual value depends on gyro ODR
//	- threshold = Activity Threshold
//	- sleepOn = Gyroscope operating mode during inactivity.
//	  true: gyroscope in sleep mode
//	  false: gyroscope in power-down
void lsm9ds1_configInactivity(lsm9ds1_t *lsm9ds1, uint8_t duration, uint8_t threshold, bool sleepOn);

// lsm9ds1_configAccelInt() -- Configure Accelerometer Interrupt Generator
// Input:
//	- generator = Interrupt axis/high-low events
//	  Any OR'd combination of ZHIE_XL, ZLIE_XL, YHIE_XL, YLIE_XL, XHIE_XL, XLIE_XL
//	- andInterrupts = AND/OR combination of interrupt events
//	  true: AND combination
//	  false: OR combination
void lsm9ds1_configAccelInt(lsm9ds1_t *lsm9ds1, uint8_t generator, bool andInterrupts);

// lsm9ds1_configAccelThs() -- Configure the threshold of an accelereomter axis
// Input:
//	- threshold = Interrupt threshold. Possible values: 0-255.
//	  Multiply by 128 to get the actual raw accel value.
//	- axis = Axis to be configured. Either X_AXIS, Y_AXIS, or Z_AXIS
//	- duration = Duration value must be above or below threshold to trigger interrupt
//	- wait = Wait function on duration counter
//	  true: Wait for duration samples before exiting interrupt
//	  false: Wait function off
void lsm9ds1_configAccelThs(lsm9ds1_t *lsm9ds1, uint8_t threshold, lsm9ds1_axis axis, uint8_t duration, bool wait);

// lsm9ds1_configGyroInt() -- Configure Gyroscope Interrupt Generator
// Input:
//	- generator = Interrupt axis/high-low events
//	  Any OR'd combination of ZHIE_G, ZLIE_G, YHIE_G, YLIE_G, XHIE_G, XLIE_G
//	- aoi = AND/OR combination of interrupt events
//	  true: AND combination
//	  false: OR combination
//	- latch: latch gyroscope interrupt request.
void lsm9ds1_configGyroInt(lsm9ds1_t *lsm9ds1, uint8_t generator, bool aoi, bool latch);

// lsm9ds1_configGyroThs() -- Configure the threshold of a gyroscope axis
// Input:
//	- threshold = Interrupt threshold. Possible values: 0-0x7FF.
//	  Value is equivalent to raw gyroscope value.
//	- axis = Axis to be configured. Either X_AXIS, Y_AXIS, or Z_AXIS
//	- duration = Duration value must be above or below threshold to trigger interrupt
//	- wait = Wait function on duration counter
//	  true: Wait for duration samples before exiting interrupt
//	  false: Wait function off
void lsm9ds1_configGyroThs(lsm9ds1_t *lsm9ds1, int16_t threshold, lsm9ds1_axis axis, uint8_t duration, bool wait);

// lsm9ds1_configInt() -- Configure INT1 or INT2 (Gyro and Accel Interrupts only)
// Input:
//	- interrupt = Select INT1 or INT2
//	  Possible values: XG_INT1 or XG_INT2
//	- generator = Or'd combination of interrupt generators.
//	  Possible values: INT_DRDY_XL, INT_DRDY_G, INT1_BOOT (INT1 only), INT2_DRDY_TEMP (INT2 only)
//	  INT_FTH, INT_OVR, INT_FSS5, INT_IG_XL (INT1 only), INT1_IG_G (INT1 only), INT2_INACT (INT2 only)
//	- activeLow = Interrupt active configuration
//	  Can be either INT_ACTIVE_HIGH or INT_ACTIVE_LOW
//	- pushPull =  Push-pull or open drain interrupt configuration
//	  Can be either INT_PUSH_PULL or INT_OPEN_DRAIN
void lsm9ds1_configInt(lsm9ds1_t *lsm9ds1, interrupt_select interupt, uint8_t generator,
				h_lactive activeLow, pp_od pushPull);
				
// lsm9ds1_configMagInt() -- Configure Magnetometer Interrupt Generator
// Input:
//	- generator = Interrupt axis/high-low events
//	  Any OR'd combination of ZIEN, YIEN, XIEN
//	- activeLow = Interrupt active configuration
//	  Can be either INT_ACTIVE_HIGH or INT_ACTIVE_LOW
//	- latch: latch gyroscope interrupt request.
void lsm9ds1_configMagInt(lsm9ds1_t *lsm9ds1, uint8_t generator, h_lactive activeLow, bool latch);

// lsm9ds1_configMagThs() -- Configure the threshold of a gyroscope axis
// Input:
//	- threshold = Interrupt threshold. Possible values: 0-0x7FF.
//	  Value is equivalent to raw magnetometer value.
void lsm9ds1_configMagThs(lsm9ds1_t *lsm9ds1, uint16_t threshold);

// lsm9ds1_getGyroIntSrc() -- Get contents of Gyroscope interrupt source register
uint8_t lsm9ds1_getGyroIntSrc(lsm9ds1_t *lsm9ds1);

// lsm9ds1_getAccelIntSrc() -- Get contents of accelerometer interrupt source register
uint8_t lsm9ds1_getAccelIntSrc(lsm9ds1_t *lsm9ds1);

// lsm9ds1_getMagIntSrc() -- Get contents of magnetometer interrupt source register
uint8_t lsm9ds1_getMagIntSrc(lsm9ds1_t *lsm9ds1);

// lsm9ds1_getInactivity() -- Get status of inactivity interrupt
uint8_t lsm9ds1_getInactivity(lsm9ds1_t *lsm9ds1);

// lsm9ds1_sleepGyro() -- Sleep or wake the gyroscope
// Input:
//	- enable: True = sleep gyro. False = wake gyro.
void lsm9ds1_sleepGyro(lsm9ds1_t *lsm9ds1, bool enable);

// lsm9ds1_enableFIFO() - Enable or disable the FIFO
// Input:
//	- enable: true = enable, false = disable.
void lsm9ds1_enableFIFO(lsm9ds1_t *lsm9ds1, bool enable);

// lsm9ds1_setFIFO() - Configure FIFO mode and Threshold
// Input:
//	- fifoMode: Set FIFO mode to off, FIFO (stop when full), continuous, bypass
//	  Possible inputs: FIFO_OFF, FIFO_THS, FIFO_CONT_TRIGGER, FIFO_OFF_TRIGGER, FIFO_CONT
//	- fifoThs: FIFO threshold level setting
//	  Any value from 0-0x1F is acceptable.
void lsm9ds1_setFIFO(lsm9ds1_t *lsm9ds1, fifoMode_type fifoMode, uint8_t fifoThs);

// lsm9ds1_getFIFOSamples() - Get number of FIFO samples
uint8_t lsm9ds1_getFIFOSamples(lsm9ds1_t *lsm9ds1);
	

// lsm9ds1_init() -- Sets up gyro, accel, and mag settings to default.
// to set com interface and/or addresses see lsm9ds1_begin() and beginSPI().
void lsm9ds1_init();

// lsm9ds1_initGyro() -- Sets up the gyroscope to lsm9ds1_begin reading.
// This function steps through all five gyroscope control registers.
// Upon exit, the following parameters will be set:
//	- CTRL_REG1_G = 0x0F: Normal operation mode, all axes enabled. 
//		95 Hz ODR, 12.5 Hz cutoff frequency.
//	- CTRL_REG2_G = 0x00: HPF set to normal mode, cutoff frequency
//		set to 7.2 Hz (depends on ODR).
//	- CTRL_REG3_G = 0x88: Interrupt enabled on INT_G (set to push-pull and
//		active high). Data-ready output enabled on DRDY_G.
//	- CTRL_REG4_G = 0x00: Continuous update mode. Data LSB stored in lower
//		address. Scale set to 245 DPS. SPI mode set to 4-wire.
//	- CTRL_REG5_G = 0x00: FIFO disabled. HPF disabled.
void lsm9ds1_initGyro(lsm9ds1_t *lsm9ds1);

// lsm9ds1_initAccel() -- Sets up the accelerometer to lsm9ds1_begin reading.
// This function steps through all accelerometer related control registers.
// Upon exit these registers will be set as:
//	- CTRL_REG0_XM = 0x00: FIFO disabled. HPF bypassed. Normal mode.
//	- CTRL_REG1_XM = 0x57: 100 Hz data rate. Continuous update.
//		all axes enabled.
//	- CTRL_REG2_XM = 0x00:  2g scale. 773 Hz anti-alias filter BW.
//	- CTRL_REG3_XM = 0x04: Accel data ready signal on INT1_XM pin.
void lsm9ds1_initAccel(lsm9ds1_t *lsm9ds1);

// lsm9ds1_initMag() -- Sets up the magnetometer to lsm9ds1_begin reading.
// This function steps through all magnetometer-related control registers.
// Upon exit these registers will be set as:
//	- CTRL_REG4_XM = 0x04: Mag data ready signal on INT2_XM pin.
//	- CTRL_REG5_XM = 0x14: 100 Hz update rate. Low resolution. Interrupt
//		requests don't latch. Temperature sensor disabled.
//	- CTRL_REG6_XM = 0x00:  2 Gs scale.
//	- CTRL_REG7_XM = 0x00: Continuous conversion mode. Normal HPF mode.
//	- INT_CTRL_REG_M = 0x09: Interrupt active-high. Enable interrupts.
void lsm9ds1_initMag(lsm9ds1_t *lsm9ds1);

// gReadByte() -- Reads a byte from a specified gyroscope register.
// Input:
// 	- subAddress = Register to be read from.
// Output:
// 	- An 8-bit value read from the requested address.
uint8_t lsm9ds1_mReadByte(lsm9ds1_t *lsm9ds1, uint8_t subAddress);

// gReadBytes() -- Reads a number of bytes -- beginning at an address
// and incrementing from there -- from the gyroscope.
// Input:
// 	- subAddress = Register to be read from.
// 	- * dest = A pointer to an array of uint8_t's. Values read will be
//		stored in here on return.
//	- count = The number of bytes to be read.
// Output: No value is returned, but the `dest` array will store
// 	the data read upon exit.
uint8_t lsm9ds1_mReadBytes(lsm9ds1_t *lsm9ds1, uint8_t subAddress, uint8_t * dest, uint8_t count);

// gWriteByte() -- Write a byte to a register in the gyroscope.
// Input:
//	- subAddress = Register to be written to.
//	- data = data to be written to the register.
void lsm9ds1_mWriteByte(lsm9ds1_t *lsm9ds1, uint8_t subAddress, uint8_t data);

// xmReadByte() -- Read a byte from a register in the accel/mag sensor
// Input:
//	- subAddress = Register to be read from.
// Output:
//	- An 8-bit value read from the requested register.
uint8_t lsm9ds1_xgReadByte(lsm9ds1_t *lsm9ds1, uint8_t subAddress);

// xmReadBytes() -- Reads a number of bytes -- beginning at an address
// and incrementing from there -- from the accelerometer/magnetometer.
// Input:
// 	- subAddress = Register to be read from.
// 	- * dest = A pointer to an array of uint8_t's. Values read will be
//		stored in here on return.
//	- count = The number of bytes to be read.
// Output: No value is returned, but the `dest` array will store
// 	the data read upon exit.
uint8_t lsm9ds1_xgReadBytes(lsm9ds1_t *lsm9ds1, uint8_t subAddress, uint8_t * dest, uint8_t count);

// xmWriteByte() -- Write a byte to a register in the accel/mag sensor.
// Input:
//	- subAddress = Register to be written to.
//	- data = data to be written to the register.
void lsm9ds1_xgWriteByte(lsm9ds1_t *lsm9ds1, uint8_t subAddress, uint8_t data);

// lsm9ds1_calcgRes() -- Calculate the resolution of the gyroscope.
// This function will set the value of the gRes variable. gScale must
// be set prior to calling this function.
void lsm9ds1_calcgRes(lsm9ds1_t *lsm9ds1);

// lsm9ds1_calcmRes() -- Calculate the resolution of the magnetometer.
// This function will set the value of the mRes variable. mScale must
// be set prior to calling this function.
void lsm9ds1_calcmRes(lsm9ds1_t *lsm9ds1);

// lsm9ds1_calcaRes() -- Calculate the resolution of the accelerometer.
// This function will set the value of the aRes variable. aScale must
// be set prior to calling this function.
void lsm9ds1_calcaRes(lsm9ds1_t *lsm9ds1);

//////////////////////
// Helper Functions //
//////////////////////
void lsm9ds1_constrainScales(lsm9ds1_t *lsm9ds1);
	
///////////////////
// I2C Functions //
///////////////////

// I2CwriteByte() -- Write a byte out of I2C to a register in the device
// Input:
//	- address = The 7-bit I2C address of the slave device.
//	- subAddress = The register to be written to.
//	- data = Byte to be written to the register.
void I2CwriteByte(nrfx_twim_t *twim, uint8_t address, uint8_t subAddress, uint8_t data);

// I2CreadByte() -- Read a single byte from a register over I2C.
// Input:
//	- address = The 7-bit I2C address of the slave device.
//	- subAddress = The register to be read from.
// Output:
//	- The byte read from the requested address.
uint8_t I2CreadByte(nrfx_twim_t *twim, uint8_t address, uint8_t subAddress);

// I2CreadBytes() -- Read a series of bytes, starting at a register via SPI
// Input:
//	- address = The 7-bit I2C address of the slave device.
//	- subAddress = The register to lsm9ds1_begin reading.
// 	- * dest = Pointer to an array where we'll store the readings.
//	- count = Number of registers to be read.
// Output: No value is returned by the function, but the registers read are
// 		all stored in the *dest array given.
uint8_t I2CreadBytes(nrfx_twim_t *twim, uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count);


#endif // SFE_LSM9DS1_H //

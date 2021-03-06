/******************************************************************************
SFE_LSM9DS1.cpp
SFE_LSM9DS1 Library Source File
Jim Lindblom @ SparkFun Electronics
Original Creation Date: February 27, 2015
https://github.com/sparkfun/LSM9DS1_Breakout

This file implements all functions of the LSM9DS1 class. Functions here range
from higher level stuff, like reading/writing LSM9DS1 registers to low-level,
hardware reads and writes. Both SPI and I2C handler functions can be found
towards the bottom of this file.

Development environment specifics:
	IDE: Arduino 1.6
	Hardware Platform: Arduino Uno
	LSM9DS1 Breakout Version: 1.0

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
------------------------------------------------------------------------------
- Migrated from C++ to C and ported to NRF52 devices - 10/4/2020
  Anderson Contreras <acontreras@aimonkey.info>
******************************************************************************/

#include "SparkFunLSM9DS1.h"
#include "LSM9DS1_Registers.h"
#include "LSM9DS1_Types.h"

#include <nrfx_twim.h>
#include <nrf_log.h>
#include <app_error.h>

// Sensor Sensitivity Constants
// Values set according to the typical specifications provided in
// table 3 of the LSM9DS1 datasheet. (pg 12)
#define SENSITIVITY_ACCELEROMETER_2  0.000061
#define SENSITIVITY_ACCELEROMETER_4  0.000122
#define SENSITIVITY_ACCELEROMETER_8  0.000244
#define SENSITIVITY_ACCELEROMETER_16 0.000732
#define SENSITIVITY_GYROSCOPE_245    0.00875
#define SENSITIVITY_GYROSCOPE_500    0.0175
#define SENSITIVITY_GYROSCOPE_2000   0.07
#define SENSITIVITY_MAGNETOMETER_4   0.00014
#define SENSITIVITY_MAGNETOMETER_8   0.00029
#define SENSITIVITY_MAGNETOMETER_12  0.00043
#define SENSITIVITY_MAGNETOMETER_16  0.00058
#define SENSITIVITY_TEMPERATURE      0.0625

void lsm9ds1_init(lsm9ds1_t *lsm9ds1)
{
	lsm9ds1->settings.gyro.enabled = true;
	lsm9ds1->settings.gyro.enableX = true;
	lsm9ds1->settings.gyro.enableY = true;
	lsm9ds1->settings.gyro.enableZ = true;
	// gyro scale can be 245, 500, or 2000
	lsm9ds1->settings.gyro.scale = 245;
	// gyro sample rate: value between 1-6
	// 1 = 14.9    4 = 238
	// 2 = 59.5    5 = 476
	// 3 = 119     6 = 952
	lsm9ds1->settings.gyro.sampleRate = 1;
	// gyro cutoff frequency: value between 0-3
	// Actual value of cutoff frequency depends
	// on sample rate.
	lsm9ds1->settings.gyro.bandwidth = 0;
	lsm9ds1->settings.gyro.lowPowerEnable = true;
	lsm9ds1->settings.gyro.HPFEnable = false;
	// Gyro HPF cutoff frequency: value between 0-9
	// Actual value depends on sample rate. Only applies
	// if gyroHPFEnable is true.
	lsm9ds1->settings.gyro.HPFCutoff = 0;
	lsm9ds1->settings.gyro.flipX = false;
	lsm9ds1->settings.gyro.flipY = false;
	lsm9ds1->settings.gyro.flipZ = false;
	lsm9ds1->settings.gyro.orientation = 0;
	lsm9ds1->settings.gyro.latchInterrupt = true;

	lsm9ds1->settings.accel.enabled = true;
	lsm9ds1->settings.accel.enableX = true;
	lsm9ds1->settings.accel.enableY = true;
	lsm9ds1->settings.accel.enableZ = true;
	// accel scale can be 2, 4, 8, or 16
	lsm9ds1->settings.accel.scale = 2;
	// accel sample rate can be 1-6
	// 1 = 10 Hz    4 = 238 Hz
	// 2 = 50 Hz    5 = 476 Hz
	// 3 = 119 Hz   6 = 952 Hz
	lsm9ds1->settings.accel.sampleRate = 1;
	// Accel cutoff frequency can be any value between -1 - 3. 
	// -1 = bandwidth determined by sample rate
	// 0 = 408 Hz   2 = 105 Hz
	// 1 = 211 Hz   3 = 50 Hz
	lsm9ds1->settings.accel.bandwidth = -1;
	lsm9ds1->settings.accel.highResEnable = false;
	// accelHighResBandwidth can be any value between 0-3
	// LP cutoff is set to a factor of sample rate
	// 0 = ODR/50    2 = ODR/9
	// 1 = ODR/100   3 = ODR/400
	lsm9ds1->settings.accel.highResBandwidth = 0;

	lsm9ds1->settings.mag.enabled = true;
	// mag scale can be 4, 8, 12, or 16
	lsm9ds1->settings.mag.scale = 4;
	// mag data rate can be 0-7
	// 0 = 0.625 Hz  4 = 10 Hz
	// 1 = 1.25 Hz   5 = 20 Hz
	// 2 = 2.5 Hz    6 = 40 Hz
	// 3 = 5 Hz      7 = 80 Hz
	lsm9ds1->settings.mag.sampleRate = 4;
	lsm9ds1->settings.mag.tempCompensationEnable = false;
	// magPerformance can be any value between 0-3
	// 0 = Low power mode      2 = high performance
	// 1 = medium performance  3 = ultra-high performance
	lsm9ds1->settings.mag.XYPerformance = 3;
	lsm9ds1->settings.mag.ZPerformance = 3;
	lsm9ds1->settings.mag.lowPowerEnable = false;
	// magOperatingMode can be 0-2
	// 0 = continuous conversion
	// 1 = single-conversion
	// 2 = power down
	lsm9ds1->settings.mag.operatingMode = 0;

	lsm9ds1->settings.temp.enabled = true;
	for (int i=0; i<3; i++)
	{
		lsm9ds1->gBias[i] = 0;
		lsm9ds1->aBias[i] = 0;
		lsm9ds1->mBias[i] = 0;
		lsm9ds1->gBiasRaw[i] = 0;
		lsm9ds1->aBiasRaw[i] = 0;
		lsm9ds1->mBiasRaw[i] = 0;
	}
	lsm9ds1->_autoCalc = false;
}


uint16_t lsm9ds1_begin(lsm9ds1_t *lsm9ds1, nrfx_twim_t *twim, uint8_t agAddress, uint8_t mAddress)
{
	// Set device settings, they are used in many other places
	lsm9ds1->settings.device.commInterface = IMU_MODE_I2C;
	lsm9ds1->settings.device.agAddress = agAddress;
	lsm9ds1->settings.device.mAddress = mAddress;
	lsm9ds1->settings.device.i2c = twim;
	
	//! Todo: don't use _xgAddress or _mAddress, duplicating memory
	lsm9ds1->_xgAddress = lsm9ds1->settings.device.agAddress;
	lsm9ds1->_mAddress = lsm9ds1->settings.device.mAddress;
	
	lsm9ds1_init(lsm9ds1);
	
	lsm9ds1_constrainScales(lsm9ds1);
	// Once we have the scale values, we can calculate the resolution
	// of each sensor. That's what these functions are for. One for each sensor
	lsm9ds1_calcgRes(lsm9ds1); // Calculate DPS / ADC tick, stored in gRes variable
	lsm9ds1_calcmRes(lsm9ds1); // Calculate Gs / ADC tick, stored in mRes variable
	lsm9ds1_calcaRes(lsm9ds1); // Calculate g / ADC tick, stored in aRes variable
	
	// We expect caller to lsm9ds1_begin their I2C port, with the speed of their choice external to the library
	// But if they forget, we could start the hardware here.
	// settings.device.i2c->lsm9ds1_begin();	// Initialize I2C library
		
	// To verify communication, we can read from the WHO_AM_I register of
	// each device. Store those in a variable so we can return them.
	uint8_t mTest = lsm9ds1_mReadByte(lsm9ds1, WHO_AM_I_M);		// Read the gyro WHO_AM_I
	uint8_t xgTest = lsm9ds1_xgReadByte(lsm9ds1, WHO_AM_I_XG);	// Read the accel/mag WHO_AM_I
	uint16_t whoAmICombined = (xgTest << 8) | mTest;
	
	if (whoAmICombined != ((WHO_AM_I_AG_RSP << 8) | WHO_AM_I_M_RSP))
		return 0;
	
	// Gyro initialization stuff:
	lsm9ds1_initGyro(lsm9ds1);	// This will "turn on" the gyro. Setting up interrupts, etc.
	
	// Accelerometer initialization stuff:
	lsm9ds1_initAccel(lsm9ds1); // "Turn on" all axes of the accel. Set up interrupts, etc.
	
	// Magnetometer initialization stuff:
	lsm9ds1_initMag(lsm9ds1); // "Turn on" all axes of the mag. Set up interrupts, etc.

	// Once everything is initialized, return the WHO_AM_I registers we read:
	return whoAmICombined;
}


void lsm9ds1_initGyro(lsm9ds1_t *lsm9ds1)
{
	uint8_t tempRegValue = 0;
	
	// CTRL_REG1_G (Default value: 0x00)
	// [ODR_G2][ODR_G1][ODR_G0][FS_G1][FS_G0][0][BW_G1][BW_G0]
	// ODR_G[2:0] - Output data rate selection
	// FS_G[1:0] - Gyroscope full-scale selection
	// BW_G[1:0] - Gyroscope bandwidth selection
	
	// To disable gyro, set sample rate bits to 0. We'll only set sample
	// rate if the gyro is enabled.
	if (lsm9ds1->settings.gyro.enabled)
	{
		tempRegValue = (lsm9ds1->settings.gyro.sampleRate & 0x07) << 5;
	}
	switch (lsm9ds1->settings.gyro.scale)
	{
		case 500:
			tempRegValue |= (0x1 << 3);
			break;
		case 2000:
			tempRegValue |= (0x3 << 3);
			break;
		// Otherwise we'll set it to 245 dps (0x0 << 4)
	}
	tempRegValue |= (lsm9ds1->settings.gyro.bandwidth & 0x3);
	lsm9ds1_xgWriteByte(lsm9ds1, CTRL_REG1_G, tempRegValue);
	
	// CTRL_REG2_G (Default value: 0x00)
	// [0][0][0][0][INT_SEL1][INT_SEL0][OUT_SEL1][OUT_SEL0]
	// INT_SEL[1:0] - INT selection configuration
	// OUT_SEL[1:0] - Out selection configuration
	lsm9ds1_xgWriteByte(lsm9ds1, CTRL_REG2_G, 0x00);	
	
	// CTRL_REG3_G (Default value: 0x00)
	// [LP_mode][HP_EN][0][0][HPCF3_G][HPCF2_G][HPCF1_G][HPCF0_G]
	// LP_mode - Low-power mode enable (0: disabled, 1: enabled)
	// HP_EN - HPF enable (0:disabled, 1: enabled)
	// HPCF_G[3:0] - HPF cutoff frequency
	tempRegValue = lsm9ds1->settings.gyro.lowPowerEnable ? (1<<7) : 0;
	if (lsm9ds1->settings.gyro.HPFEnable)
	{
		tempRegValue |= (1<<6) | (lsm9ds1->settings.gyro.HPFCutoff & 0x0F);
	}
	lsm9ds1_xgWriteByte(lsm9ds1, CTRL_REG3_G, tempRegValue);
	
	// CTRL_REG4 (Default value: 0x38)
	// [0][0][Zen_G][Yen_G][Xen_G][0][LIR_XL1][4D_XL1]
	// Zen_G - Z-axis output enable (0:disable, 1:enable)
	// Yen_G - Y-axis output enable (0:disable, 1:enable)
	// Xen_G - X-axis output enable (0:disable, 1:enable)
	// LIR_XL1 - Latched interrupt (0:not latched, 1:latched)
	// 4D_XL1 - 4D option on interrupt (0:6D used, 1:4D used)
	tempRegValue = 0;
	if (lsm9ds1->settings.gyro.enableZ) tempRegValue |= (1<<5);
	if (lsm9ds1->settings.gyro.enableY) tempRegValue |= (1<<4);
	if (lsm9ds1->settings.gyro.enableX) tempRegValue |= (1<<3);
	if (lsm9ds1->settings.gyro.latchInterrupt) tempRegValue |= (1<<1);
	lsm9ds1_xgWriteByte(lsm9ds1, CTRL_REG4, tempRegValue);
	
	// ORIENT_CFG_G (Default value: 0x00)
	// [0][0][SignX_G][SignY_G][SignZ_G][Orient_2][Orient_1][Orient_0]
	// SignX_G - Pitch axis (X) angular rate sign (0: positive, 1: negative)
	// Orient [2:0] - Directional user orientation selection
	tempRegValue = 0;
	if (lsm9ds1->settings.gyro.flipX) tempRegValue |= (1<<5);
	if (lsm9ds1->settings.gyro.flipY) tempRegValue |= (1<<4);
	if (lsm9ds1->settings.gyro.flipZ) tempRegValue |= (1<<3);
	lsm9ds1_xgWriteByte(lsm9ds1, ORIENT_CFG_G, tempRegValue);
}

void lsm9ds1_initAccel(lsm9ds1_t *lsm9ds1)
{
	uint8_t tempRegValue = 0;
	
	//	CTRL_REG5_XL (0x1F) (Default value: 0x38)
	//	[DEC_1][DEC_0][Zen_XL][Yen_XL][Zen_XL][0][0][0]
	//	DEC[0:1] - Decimation of accel data on OUT REG and FIFO.
	//		00: None, 01: 2 samples, 10: 4 samples 11: 8 samples
	//	Zen_XL - Z-axis output enabled
	//	Yen_XL - Y-axis output enabled
	//	Xen_XL - X-axis output enabled
	if (lsm9ds1->settings.accel.enableZ) tempRegValue |= (1<<5);
	if (lsm9ds1->settings.accel.enableY) tempRegValue |= (1<<4);
	if (lsm9ds1->settings.accel.enableX) tempRegValue |= (1<<3);
	
	lsm9ds1_xgWriteByte(lsm9ds1, CTRL_REG5_XL, tempRegValue);
	
	// CTRL_REG6_XL (0x20) (Default value: 0x00)
	// [ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_SCAL_ODR][BW_XL1][BW_XL0]
	// ODR_XL[2:0] - Output data rate & power mode selection
	// FS_XL[1:0] - Full-scale selection
	// BW_SCAL_ODR - Bandwidth selection
	// BW_XL[1:0] - Anti-aliasing filter bandwidth selection
	tempRegValue = 0;
	// To disable the accel, set the sampleRate bits to 0.
	if (lsm9ds1->settings.accel.enabled)
	{
		tempRegValue |= (lsm9ds1->settings.accel.sampleRate & 0x07) << 5;
	}
	switch (lsm9ds1->settings.accel.scale)
	{
		case 4:
			tempRegValue |= (0x2 << 3);
			break;
		case 8:
			tempRegValue |= (0x3 << 3);
			break;
		case 16:
			tempRegValue |= (0x1 << 3);
			break;
		// Otherwise it'll be set to 2g (0x0 << 3)
	}
	if (lsm9ds1->settings.accel.bandwidth >= 0)
	{
		tempRegValue |= (1<<2); // Set BW_SCAL_ODR
		tempRegValue |= (lsm9ds1->settings.accel.bandwidth & 0x03);
	}
	lsm9ds1_xgWriteByte(lsm9ds1, CTRL_REG6_XL, tempRegValue);
	
	// CTRL_REG7_XL (0x21) (Default value: 0x00)
	// [HR][DCF1][DCF0][0][0][FDS][0][HPIS1]
	// HR - High resolution mode (0: disable, 1: enable)
	// DCF[1:0] - Digital filter cutoff frequency
	// FDS - Filtered data selection
	// HPIS1 - HPF enabled for interrupt function
	tempRegValue = 0;
	if (lsm9ds1->settings.accel.highResEnable)
	{
		tempRegValue |= (1<<7); // Set HR bit
		tempRegValue |= (lsm9ds1->settings.accel.highResBandwidth & 0x3) << 5;
	}
	lsm9ds1_xgWriteByte(lsm9ds1, CTRL_REG7_XL, tempRegValue);
}

// This is a function that uses the FIFO to accumulate sample of accelerometer and gyro data, average
// them, scales them to  gs and deg/s, respectively, and then passes the biases to the main sketch
// for subtraction from all subsequent data. There are no gyro and accelerometer bias registers to store
// the data as there are in the ADXL345, a precursor to the LSM9DS0, or the MPU-9150, so we have to
// subtract the biases ourselves. This results in a more accurate measurement in general and can
// remove errors due to imprecise or varying initial placement. Calibration of sensor data in this manner
// is good practice.
void lsm9ds1_calibrate(lsm9ds1_t *lsm9ds1, bool autoCalc)
{
	uint8_t samples = 0;
	int ii;
	int32_t aBiasRawTemp[3] = {0, 0, 0};
	int32_t gBiasRawTemp[3] = {0, 0, 0};
	
	// Turn on FIFO and set threshold to 32 samples
	lsm9ds1_enableFIFO(lsm9ds1, true);
	lsm9ds1_setFIFO(lsm9ds1, FIFO_THS, 0x1F);
	while (samples < 0x1F)
	{
		samples = (lsm9ds1_xgReadByte(lsm9ds1, FIFO_SRC) & 0x3F); // Read number of stored samples
	}
	for(ii = 0; ii < samples ; ii++) 
	{	// Read the gyro data stored in the FIFO
		lsm9ds1_readGyro(lsm9ds1);
		gBiasRawTemp[0] += lsm9ds1->gx;
		gBiasRawTemp[1] += lsm9ds1->gy;
		gBiasRawTemp[2] += lsm9ds1->gz;
		lsm9ds1_readAccel(lsm9ds1);
		aBiasRawTemp[0] += lsm9ds1->ax;
		aBiasRawTemp[1] += lsm9ds1->ay;
		aBiasRawTemp[2] += lsm9ds1->az - (int16_t)(1./lsm9ds1->aRes); // Assumes sensor facing up!
	}  
	for (ii = 0; ii < 3; ii++)
	{
		lsm9ds1->gBiasRaw[ii] = gBiasRawTemp[ii] / samples;
		lsm9ds1->gBias[ii] = lsm9ds1_calcGyro(lsm9ds1, lsm9ds1->gBiasRaw[ii]);
		lsm9ds1->aBiasRaw[ii] = aBiasRawTemp[ii] / samples;
		lsm9ds1->aBias[ii] = lsm9ds1_calcAccel(lsm9ds1, lsm9ds1->aBiasRaw[ii]);
	}
	
	lsm9ds1_enableFIFO(lsm9ds1, false);
	lsm9ds1_setFIFO(lsm9ds1, FIFO_OFF, 0x00);
	
	if (autoCalc) lsm9ds1->_autoCalc = true;
}

void lsm9ds1_calibrateMag(lsm9ds1_t *lsm9ds1, bool loadIn)
{
	int i, j;
	int16_t magMin[3] = {0, 0, 0};
	int16_t magMax[3] = {0, 0, 0}; // The road warrior
	
	for (i=0; i<128; i++)
	{
		while (!lsm9ds1_magAvailable(lsm9ds1, ALL_AXIS))
			;
		lsm9ds1_readMag(lsm9ds1);
		int16_t magTemp[3] = {0, 0, 0};
		magTemp[0] = lsm9ds1->mx;		
		magTemp[1] = lsm9ds1->my;
		magTemp[2] = lsm9ds1->mz;
		for (j = 0; j < 3; j++)
		{
			if (magTemp[j] > magMax[j]) magMax[j] = magTemp[j];
			if (magTemp[j] < magMin[j]) magMin[j] = magTemp[j];
		}
	}
	for (j = 0; j < 3; j++)
	{
		lsm9ds1->mBiasRaw[j] = (magMax[j] + magMin[j]) / 2;
		lsm9ds1->mBias[j] = lsm9ds1_calcMag(lsm9ds1, lsm9ds1->mBiasRaw[j]);
		if (loadIn)
			lsm9ds1_magOffset(lsm9ds1, j, lsm9ds1->mBiasRaw[j]);
	}
	
}
void lsm9ds1_magOffset(lsm9ds1_t *lsm9ds1, uint8_t axis, int16_t offset)
{
	if (axis > 2)
		return;
	uint8_t msb, lsb;
	msb = (offset & 0xFF00) >> 8;
	lsb = offset & 0x00FF;
	lsm9ds1_mWriteByte(lsm9ds1, OFFSET_X_REG_L_M + (2 * axis), lsb);
	lsm9ds1_mWriteByte(lsm9ds1, OFFSET_X_REG_H_M + (2 * axis), msb);
}

void lsm9ds1_initMag(lsm9ds1_t *lsm9ds1)
{
	uint8_t tempRegValue = 0;
	
	// CTRL_REG1_M (Default value: 0x10)
	// [TEMP_COMP][OM1][OM0][DO2][DO1][DO0][0][ST]
	// TEMP_COMP - Temperature compensation
	// OM[1:0] - X & Y axes op mode selection
	//	00:low-power, 01:medium performance
	//	10: high performance, 11:ultra-high performance
	// DO[2:0] - Output data rate selection
	// ST - Self-test enable
	if (lsm9ds1->settings.mag.tempCompensationEnable) tempRegValue |= (1<<7);
	tempRegValue |= (lsm9ds1->settings.mag.XYPerformance & 0x3) << 5;
	tempRegValue |= (lsm9ds1->settings.mag.sampleRate & 0x7) << 2;
	lsm9ds1_mWriteByte(lsm9ds1, CTRL_REG1_M, tempRegValue);
	
	// CTRL_REG2_M (Default value 0x00)
	// [0][FS1][FS0][0][REBOOT][SOFT_RST][0][0]
	// FS[1:0] - Full-scale configuration
	// REBOOT - Reboot memory content (0:normal, 1:reboot)
	// SOFT_RST - Reset config and user registers (0:default, 1:reset)
	tempRegValue = 0;
	switch (lsm9ds1->settings.mag.scale)
	{
	case 8:
		tempRegValue |= (0x1 << 5);
		break;
	case 12:
		tempRegValue |= (0x2 << 5);
		break;
	case 16:
		tempRegValue |= (0x3 << 5);
		break;
	// Otherwise we'll default to 4 gauss (00)
	}
	lsm9ds1_mWriteByte(lsm9ds1, CTRL_REG2_M, tempRegValue); // +/-4Gauss
	
	// CTRL_REG3_M (Default value: 0x03)
	// [I2C_DISABLE][0][LP][0][0][SIM][MD1][MD0]
	// I2C_DISABLE - Disable I2C interace (0:enable, 1:disable)
	// LP - Low-power mode cofiguration (1:enable)
	// SIM - SPI mode selection (0:write-only, 1:read/write enable)
	// MD[1:0] - Operating mode
	//	00:continuous conversion, 01:single-conversion,
	//  10,11: Power-down
	tempRegValue = 0;
	if (lsm9ds1->settings.mag.lowPowerEnable) tempRegValue |= (1<<5);
	tempRegValue |= (lsm9ds1->settings.mag.operatingMode & 0x3);
	lsm9ds1_mWriteByte(lsm9ds1, CTRL_REG3_M, tempRegValue); // Continuous conversion mode
	
	// CTRL_REG4_M (Default value: 0x00)
	// [0][0][0][0][OMZ1][OMZ0][BLE][0]
	// OMZ[1:0] - Z-axis operative mode selection
	//	00:low-power mode, 01:medium performance
	//	10:high performance, 10:ultra-high performance
	// BLE - Big/little endian data
	tempRegValue = 0;
	tempRegValue = (lsm9ds1->settings.mag.ZPerformance & 0x3) << 2;
	lsm9ds1_mWriteByte(lsm9ds1, CTRL_REG4_M, tempRegValue);
	
	// CTRL_REG5_M (Default value: 0x00)
	// [0][BDU][0][0][0][0][0][0]
	// BDU - Block data update for magnetic data
	//	0:continuous, 1:not updated until MSB/LSB are read
	tempRegValue = 0;
	lsm9ds1_mWriteByte(lsm9ds1, CTRL_REG5_M, tempRegValue);
}

uint8_t lsm9ds1_accelAvailable(lsm9ds1_t *lsm9ds1)
{
	uint8_t status = lsm9ds1_xgReadByte(lsm9ds1, STATUS_REG_1);
	
	return (status & (1<<0));
}

uint8_t lsm9ds1_gyroAvailable(lsm9ds1_t *lsm9ds1)
{
	uint8_t status = lsm9ds1_xgReadByte(lsm9ds1, STATUS_REG_1);
	
	return ((status & (1<<1)) >> 1);
}

uint8_t lsm9ds1_tempAvailable(lsm9ds1_t *lsm9ds1)
{
	uint8_t status = lsm9ds1_xgReadByte(lsm9ds1, STATUS_REG_1);
	
	return ((status & (1<<2)) >> 2);
}

uint8_t lsm9ds1_magAvailable(lsm9ds1_t *lsm9ds1, lsm9ds1_axis axis)
{
	uint8_t status;
	status = lsm9ds1_mReadByte(lsm9ds1, STATUS_REG_M);
	
	return ((status & (1<<axis)) >> axis);
}

void lsm9ds1_readAccel(lsm9ds1_t *lsm9ds1)
{
	uint8_t temp[6]; // We'll read six bytes from the accelerometer into temp	
	if ( lsm9ds1_xgReadBytes(lsm9ds1, OUT_X_L_XL, temp, 6) == 6 ) // Read 6 bytes, beginning at OUT_X_L_XL
	{
		lsm9ds1->ax = (temp[1] << 8) | temp[0]; // Store x-axis values into ax
		lsm9ds1->ay = (temp[3] << 8) | temp[2]; // Store y-axis values into ay
		lsm9ds1->az = (temp[5] << 8) | temp[4]; // Store z-axis values into az
		if (lsm9ds1->_autoCalc)
		{
			lsm9ds1->ax -= lsm9ds1->aBiasRaw[X_AXIS];
			lsm9ds1->ay -= lsm9ds1->aBiasRaw[Y_AXIS];
			lsm9ds1->az -= lsm9ds1->aBiasRaw[Z_AXIS];
		}
	}
}

int16_t lsm9ds1_readAccel_ax(lsm9ds1_t *lsm9ds1, lsm9ds1_axis axis)
{
	uint8_t temp[2];
	int16_t value = 0;
	if ( lsm9ds1_xgReadBytes(lsm9ds1, OUT_X_L_XL + (2 * axis), temp, 2) == 2)
	{
		value = (temp[1] << 8) | temp[0];
		
		if (lsm9ds1->_autoCalc)
			value -= lsm9ds1->aBiasRaw[axis];
		
		return value;
	}
	return value;
}

void lsm9ds1_readMag(lsm9ds1_t *lsm9ds1)
{
	uint8_t temp[6]; // We'll read six bytes from the mag into temp	
	if ( lsm9ds1_mReadBytes(lsm9ds1, OUT_X_L_M, temp, 6) == 6) // Read 6 bytes, beginning at OUT_X_L_M
	{
		lsm9ds1->mx = (temp[1] << 8) | temp[0]; // Store x-axis values into mx
		lsm9ds1->my = (temp[3] << 8) | temp[2]; // Store y-axis values into my
		lsm9ds1->mz = (temp[5] << 8) | temp[4]; // Store z-axis values into mz
	}
}

int16_t lsm9ds1_readMag_ax(lsm9ds1_t *lsm9ds1, lsm9ds1_axis axis)
{
	uint8_t temp[2];
	if ( lsm9ds1_mReadBytes(lsm9ds1, OUT_X_L_M + (2 * axis), temp, 2) == 2)
	{
		return (temp[1] << 8) | temp[0];
	}
	return 0;
}

void lsm9ds1_readTemp(lsm9ds1_t *lsm9ds1)
{
	uint8_t temp[2]; // We'll read two bytes from the temperature sensor into temp	
	if ( lsm9ds1_xgReadBytes(lsm9ds1, OUT_TEMP_L, temp, 2) == 2 ) // Read 2 bytes, beginning at OUT_TEMP_L
	{
		int16_t offset = 25;  // Per datasheet sensor outputs 0 typically @ 25 degrees centigrade
		// Temperature is scaled by 10 to avoid working with floating-point
		lsm9ds1->temperature = 10*(offset + SENSITIVITY_TEMPERATURE*(int16_t)((temp[1] << 8) | temp[0]));
	}
}

void lsm9ds1_readGyro(lsm9ds1_t *lsm9ds1)
{
	uint8_t temp[6]; // We'll read six bytes from the gyro into temp
	if ( lsm9ds1_xgReadBytes(lsm9ds1, OUT_X_L_G, temp, 6) == 6) // Read 6 bytes, beginning at OUT_X_L_G
	{
		lsm9ds1->gx = (temp[1] << 8) | temp[0]; // Store x-axis values into gx
		lsm9ds1->gy = (temp[3] << 8) | temp[2]; // Store y-axis values into gy
		lsm9ds1->gz = (temp[5] << 8) | temp[4]; // Store z-axis values into gz
		if (lsm9ds1->_autoCalc)
		{
			lsm9ds1->gx -= lsm9ds1->gBiasRaw[X_AXIS];
			lsm9ds1->gy -= lsm9ds1->gBiasRaw[Y_AXIS];
			lsm9ds1->gz -= lsm9ds1->gBiasRaw[Z_AXIS];
		}
	}
}

int16_t lsm9ds1_readGyro_ax(lsm9ds1_t *lsm9ds1, lsm9ds1_axis axis)
{
	uint8_t temp[2];
	int16_t value = 0;
	
	if ( lsm9ds1_xgReadBytes(lsm9ds1, OUT_X_L_G + (2 * axis), temp, 2) == 2)
	{
		value = (temp[1] << 8) | temp[0];
		
		if (lsm9ds1->_autoCalc)
			value -= lsm9ds1->gBiasRaw[axis];
		
		return value;
	}
	return value;
}

void lsm9ds1_readAccelGyro_burst(lsm9ds1_t *lsm9ds1, uint16_t *buffer, uint8_t length, uint8_t only_accel) {
	
	uint8_t temp[12*length];

	if(only_accel) {
		if ( lsm9ds1_xgReadBytes(lsm9ds1, OUT_X_L_G, temp, 12*length) == 12*length )
		{
			for(uint8_t i = 0, j = 0, k=0; i < length; i++, j+=6, k+=12) {
				buffer[ j ] = 0x7fff;							// Gyr_X
				buffer[j+1] = 0x7fff;  							// Gyr_Y
				buffer[j+2] = 0x7fff;  							// Gyr_Z
				buffer[j+3] = (temp[k+7]  << 8) | temp[k+6];  	// Acc_X
				buffer[j+4] = (temp[k+9]  << 8) | temp[k+8];  	// Acc_Y
				buffer[j+5] = (temp[k+11] << 8) | temp[k+10];	// Acc_Z
			}
		}
	}
	else {
		if ( lsm9ds1_xgReadBytes(lsm9ds1, OUT_X_L_G, temp, 12*length) == 12*length )
		{
			for(uint8_t i = 0, j = 0, k=0; i < length; i++, j+=6, k+=12) {
				buffer[ j ] = (temp[k+1]  << 8) | temp[ k ];	// Gyr_X
				buffer[j+1] = (temp[k+3]  << 8) | temp[k+2];  	// Gyr_Y
				buffer[j+2] = (temp[k+5]  << 8) | temp[k+4];  	// Gyr_Z
				buffer[j+3] = (temp[k+7]  << 8) | temp[k+6];  	// Acc_X
				buffer[j+4] = (temp[k+9]  << 8) | temp[k+8];  	// Acc_Y
				buffer[j+5] = (temp[k+11] << 8) | temp[k+10];	// Acc_Z
			}
		}
	}

}

float lsm9ds1_calcGyro(lsm9ds1_t *lsm9ds1, int16_t gyro)
{
	// Return the gyro raw reading times our pre-calculated DPS / (ADC tick):
	return lsm9ds1->gRes * gyro; 
}

float lsm9ds1_calcAccel(lsm9ds1_t *lsm9ds1, int16_t accel)
{
	// Return the accel raw reading times our pre-calculated g's / (ADC tick):
	return lsm9ds1->aRes * accel;
}

float lsm9ds1_calcMag(lsm9ds1_t *lsm9ds1, int16_t mag)
{
	// Return the mag raw reading times our pre-calculated Gs / (ADC tick):
	return lsm9ds1->mRes * mag;
}

void lsm9ds1_setGyroScale(lsm9ds1_t *lsm9ds1, uint16_t gScl)
{
	// Read current value of CTRL_REG1_G:
	uint8_t ctrl1RegValue = lsm9ds1_xgReadByte(lsm9ds1, CTRL_REG1_G);
	// Mask out scale bits (3 & 4):
	ctrl1RegValue &= 0xE7;
	switch (gScl)
	{
		case 500:
			ctrl1RegValue |= (0x1 << 3);
			lsm9ds1->settings.gyro.scale = 500;
			break;
		case 2000:
			ctrl1RegValue |= (0x3 << 3);
			lsm9ds1->settings.gyro.scale = 2000;
			break;
		default: // Otherwise we'll set it to 245 dps (0x0 << 4)
			lsm9ds1->settings.gyro.scale = 245;
			break;
	}
	lsm9ds1_xgWriteByte(lsm9ds1, CTRL_REG1_G, ctrl1RegValue);
	
	lsm9ds1_calcgRes(lsm9ds1);	
}

void lsm9ds1_setAccelScale(lsm9ds1_t *lsm9ds1, uint8_t aScl)
{
	// We need to preserve the other bytes in CTRL_REG6_XL. So, first read it:
	uint8_t tempRegValue = lsm9ds1_xgReadByte(lsm9ds1, CTRL_REG6_XL);
	// Mask out accel scale bits:
	tempRegValue &= 0xE7;
	
	switch (aScl)
	{
		case 4:
			tempRegValue |= (0x2 << 3);
			lsm9ds1->settings.accel.scale = 4;
			break;
		case 8:
			tempRegValue |= (0x3 << 3);
			lsm9ds1->settings.accel.scale = 8;
			break;
		case 16:
			tempRegValue |= (0x1 << 3);
			lsm9ds1->settings.accel.scale = 16;
			break;
		default: // Otherwise it'll be set to 2g (0x0 << 3)
			lsm9ds1->settings.accel.scale = 2;
			break;
	}
	lsm9ds1_xgWriteByte(lsm9ds1, CTRL_REG6_XL, tempRegValue);
	
	// Then calculate a new aRes, which relies on aScale being set correctly:
	lsm9ds1_calcaRes(lsm9ds1);
}

void lsm9ds1_setMagScale(lsm9ds1_t *lsm9ds1, uint8_t mScl)
{
	// We need to preserve the other bytes in CTRL_REG6_XM. So, first read it:
	uint8_t temp = lsm9ds1_mReadByte(lsm9ds1, CTRL_REG2_M);
	// Then mask out the mag scale bits:
	temp &= 0xFF^(0x3 << 5);
	
	switch (mScl)
	{
	case 8:
		temp |= (0x1 << 5);
		lsm9ds1->settings.mag.scale = 8;
		break;
	case 12:
		temp |= (0x2 << 5);
		lsm9ds1->settings.mag.scale = 12;
		break;
	case 16:
		temp |= (0x3 << 5);
		lsm9ds1->settings.mag.scale = 16;
		break;
	default: // Otherwise we'll default to 4 gauss (00)
		lsm9ds1->settings.mag.scale = 4;
		break;
	}	
	
	// And write the new register value back into CTRL_REG6_XM:
	lsm9ds1_mWriteByte(lsm9ds1, CTRL_REG2_M, temp);
	
	// We've updated the sensor, but we also need to update our class variables
	// First update mScale:
	//mScale = mScl;
	// Then calculate a new mRes, which relies on mScale being set correctly:
	lsm9ds1_calcmRes(lsm9ds1);
}

void lsm9ds1_setGyroODR(lsm9ds1_t *lsm9ds1, uint8_t gRate)
{
	// Only do this if gRate is not 0 (which would disable the gyro)
	if ((gRate & 0x07) != 0)
	{
		// We need to preserve the other bytes in CTRL_REG1_G. So, first read it:
		uint8_t temp = lsm9ds1_xgReadByte(lsm9ds1, CTRL_REG1_G);
		// Then mask out the gyro ODR bits:
		temp &= 0xFF^(0x7 << 5);
		temp |= (gRate & 0x07) << 5;
		// Update our settings struct
		lsm9ds1->settings.gyro.sampleRate = gRate & 0x07;
		// And write the new register value back into CTRL_REG1_G:
		lsm9ds1_xgWriteByte(lsm9ds1, CTRL_REG1_G, temp);
	}
}

void lsm9ds1_setAccelODR(lsm9ds1_t *lsm9ds1, uint8_t aRate)
{
	// Only do this if aRate is not 0 (which would disable the accel)
	if ((aRate & 0x07) != 0)
	{
		// We need to preserve the other bytes in CTRL_REG1_XM. So, first read it:
		uint8_t temp = lsm9ds1_xgReadByte(lsm9ds1, CTRL_REG6_XL);
		// Then mask out the accel ODR bits:
		temp &= 0x1F;
		// Then shift in our new ODR bits:
		temp |= ((aRate & 0x07) << 5);
		lsm9ds1->settings.accel.sampleRate = aRate & 0x07;
		// And write the new register value back into CTRL_REG1_XM:
		lsm9ds1_xgWriteByte(lsm9ds1, CTRL_REG6_XL, temp);
	}
}

void lsm9ds1_setMagODR(lsm9ds1_t *lsm9ds1, uint8_t mRate)
{
	// We need to preserve the other bytes in CTRL_REG5_XM. So, first read it:
	uint8_t temp = lsm9ds1_mReadByte(lsm9ds1, CTRL_REG1_M);
	// Then mask out the mag ODR bits:
	temp &= 0xFF^(0x7 << 2);
	// Then shift in our new ODR bits:
	temp |= ((mRate & 0x07) << 2);
	lsm9ds1->settings.mag.sampleRate = mRate & 0x07;
	// And write the new register value back into CTRL_REG5_XM:
	lsm9ds1_mWriteByte(lsm9ds1, CTRL_REG1_M, temp);
}

void lsm9ds1_calcgRes(lsm9ds1_t *lsm9ds1)
{
	switch (lsm9ds1->settings.gyro.scale)
	{
	case 245:
		lsm9ds1->gRes = SENSITIVITY_GYROSCOPE_245;
		break;
	case 500:
		lsm9ds1->gRes = SENSITIVITY_GYROSCOPE_500;
		break;
	case 2000:
		lsm9ds1->gRes = SENSITIVITY_GYROSCOPE_2000;
		break;
	default:
		break;
	}
}

void lsm9ds1_calcaRes(lsm9ds1_t *lsm9ds1)
{
	switch (lsm9ds1->settings.accel.scale)
	{
	case 2:
		lsm9ds1->aRes = SENSITIVITY_ACCELEROMETER_2;
		break;
	case 4:
		lsm9ds1->aRes = SENSITIVITY_ACCELEROMETER_4;
		break;
	case 8:
		lsm9ds1->aRes = SENSITIVITY_ACCELEROMETER_8;
		break;
	case 16:
		lsm9ds1->aRes = SENSITIVITY_ACCELEROMETER_16;
		break;
	default:
		break;
	}
}

void lsm9ds1_calcmRes(lsm9ds1_t *lsm9ds1)
{
	switch (lsm9ds1->settings.mag.scale)
	{
	case 4:
		lsm9ds1->mRes = SENSITIVITY_MAGNETOMETER_4;
		break;
	case 8:
		lsm9ds1->mRes = SENSITIVITY_MAGNETOMETER_8;
		break;
	case 12:
		lsm9ds1->mRes = SENSITIVITY_MAGNETOMETER_12;
		break;
	case 16:
		lsm9ds1->mRes = SENSITIVITY_MAGNETOMETER_16;
		break;
	}	
}

void lsm9ds1_configInt(lsm9ds1_t *lsm9ds1, interrupt_select interrupt, uint8_t generator,
	                     h_lactive activeLow, pp_od pushPull)
{
	// Write to INT1_CTRL or INT2_CTRL. [interupt] should already be one of
	// those two values.
	// [generator] should be an OR'd list of values from the interrupt_generators enum
	lsm9ds1_xgWriteByte(lsm9ds1, interrupt, generator);
	
	// Configure CTRL_REG8
	uint8_t temp;
	temp = lsm9ds1_xgReadByte(lsm9ds1, CTRL_REG8);
	
	if (activeLow) temp |= (1<<5);
	else temp &= ~(1<<5);
	
	if (!pushPull) temp &= ~(1<<4);
	else temp |= (1<<4);
	
	lsm9ds1_xgWriteByte(lsm9ds1, CTRL_REG8, temp);
}

void lsm9ds1_configInactivity(lsm9ds1_t *lsm9ds1, uint8_t duration, uint8_t threshold, bool sleepOn)
{
	uint8_t temp = 0;
	
	temp = threshold & 0x7F;
	if (sleepOn) temp |= (1<<7);
	lsm9ds1_xgWriteByte(lsm9ds1, ACT_THS, temp);
	
	lsm9ds1_xgWriteByte(lsm9ds1, ACT_DUR, duration);
}

uint8_t lsm9ds1_getInactivity(lsm9ds1_t *lsm9ds1)
{
	uint8_t temp = lsm9ds1_xgReadByte(lsm9ds1, STATUS_REG_0);
	// temp &= (0x10);
	temp = (temp & 0x10) == 0x10;
	return temp;
}

void lsm9ds1_configAccelInt(lsm9ds1_t *lsm9ds1, uint8_t generator, bool andInterrupts)
{
	// Use variables from accel_interrupt_generator, OR'd together to create
	// the [generator]value.
	uint8_t temp = generator;
	if (andInterrupts) temp |= 0x80;
	lsm9ds1_xgWriteByte(lsm9ds1, INT_GEN_CFG_XL, temp);
}

void lsm9ds1_configAccelThs(lsm9ds1_t *lsm9ds1, uint8_t threshold, lsm9ds1_axis axis, uint8_t duration, bool wait)
{
	// Write threshold value to INT_GEN_THS_?_XL.
	// axis will be 0, 1, or 2 (x, y, z respectively)
	lsm9ds1_xgWriteByte(lsm9ds1, INT_GEN_THS_X_XL + axis, threshold);
	
	// Write duration and wait to INT_GEN_DUR_XL
	uint8_t temp;
	temp = (duration & 0x7F);
	if (wait) temp |= 0x80;
	lsm9ds1_xgWriteByte(lsm9ds1, INT_GEN_DUR_XL, temp);
}

uint8_t lsm9ds1_getAccelIntSrc(lsm9ds1_t *lsm9ds1)
{
	uint8_t intSrc = lsm9ds1_xgReadByte(lsm9ds1, INT_GEN_SRC_XL);
	
	// Check if the IA_XL (interrupt active) bit is set
	if (intSrc & (1<<6))
	{
		return (intSrc & 0x3F);
	}
	
	return 0;
}

void lsm9ds1_configGyroInt(lsm9ds1_t *lsm9ds1, uint8_t generator, bool aoi, bool latch)
{
	// Use variables from accel_interrupt_generator, OR'd together to create
	// the [generator]value.
	uint8_t temp = generator;
	if (aoi) temp |= 0x80;
	if (latch) temp |= 0x40;
	lsm9ds1_xgWriteByte(lsm9ds1, INT_GEN_CFG_G, temp);
}

void lsm9ds1_configGyroThs(lsm9ds1_t *lsm9ds1, int16_t threshold, lsm9ds1_axis axis, uint8_t duration, bool wait)
{
	uint8_t buffer[2];
	buffer[0] = (threshold & 0x7F00) >> 8;
	buffer[1] = (threshold & 0x00FF);
	// Write threshold value to INT_GEN_THS_?H_G and  INT_GEN_THS_?L_G.
	// axis will be 0, 1, or 2 (x, y, z respectively)
	lsm9ds1_xgWriteByte(lsm9ds1, INT_GEN_THS_XH_G + (axis * 2), buffer[0]);
	lsm9ds1_xgWriteByte(lsm9ds1, INT_GEN_THS_XH_G + 1 + (axis * 2), buffer[1]);
	
	// Write duration and wait to INT_GEN_DUR_XL
	uint8_t temp;
	temp = (duration & 0x7F);
	if (wait) temp |= 0x80;
	lsm9ds1_xgWriteByte(lsm9ds1, INT_GEN_DUR_G, temp);
}

uint8_t lsm9ds1_getGyroIntSrc(lsm9ds1_t *lsm9ds1)
{
	uint8_t intSrc = lsm9ds1_xgReadByte(lsm9ds1, INT_GEN_SRC_G);
	
	// Check if the IA_G (interrupt active) bit is set
	if (intSrc & (1<<6))
	{
		return (intSrc & 0x3F);
	}
	
	return 0;
}

void lsm9ds1_configMagInt(lsm9ds1_t *lsm9ds1, uint8_t generator, h_lactive activeLow, bool latch)
{
	// Mask out non-generator bits (0-4)
	uint8_t config = (generator & 0xE0);	
	// IEA bit is 0 for active-low, 1 for active-high.
	if (activeLow == INT_ACTIVE_HIGH) config |= (1<<2);
	// IEL bit is 0 for latched, 1 for not-latched
	if (!latch) config |= (1<<1);
	// As long as we have at least 1 generator, enable the interrupt
	if (generator != 0) config |= (1<<0);
	
	lsm9ds1_mWriteByte(lsm9ds1, INT_CFG_M, config);
}

void lsm9ds1_configMagThs(lsm9ds1_t *lsm9ds1, uint16_t threshold)
{
	// Write high eight bits of [threshold] to INT_THS_H_M
	lsm9ds1_mWriteByte(lsm9ds1, INT_THS_H_M, (uint8_t)((threshold & 0x7F00) >> 8));
	// Write low eight bits of [threshold] to INT_THS_L_M
	lsm9ds1_mWriteByte(lsm9ds1, INT_THS_L_M, (uint8_t)(threshold & 0x00FF));
}

uint8_t lsm9ds1_getMagIntSrc(lsm9ds1_t *lsm9ds1)
{
	uint8_t intSrc = lsm9ds1_mReadByte(lsm9ds1, INT_SRC_M);
	
	// Check if the INT (interrupt active) bit is set
	if (intSrc & (1<<0))
	{
		return (intSrc & 0xFE);
	}
	
	return 0;
}

void lsm9ds1_sleepGyro(lsm9ds1_t *lsm9ds1, bool enable)
{
	uint8_t temp = lsm9ds1_xgReadByte(lsm9ds1, CTRL_REG9);
	if (enable) temp |= (1<<6);
	else temp &= ~(1<<6);
	lsm9ds1_xgWriteByte(lsm9ds1, CTRL_REG9, temp);
}

void lsm9ds1_enableFIFO(lsm9ds1_t *lsm9ds1, bool enable)
{
	uint8_t temp = lsm9ds1_xgReadByte(lsm9ds1, CTRL_REG9);
	if (enable) temp |= (1<<1);
	else temp &= ~(1<<1);
	lsm9ds1_xgWriteByte(lsm9ds1, CTRL_REG9, temp);
}

void lsm9ds1_setFIFO(lsm9ds1_t *lsm9ds1, fifoMode_type fifoMode, uint8_t fifoThs)
{
	// Limit threshold - 0x1F (31) is the maximum. If more than that was asked
	// limit it to the maximum.
	uint8_t threshold = fifoThs <= 0x1F ? fifoThs : 0x1F;
	lsm9ds1_xgWriteByte(lsm9ds1, FIFO_CTRL, ((fifoMode & 0x7) << 5) | (threshold & 0x1F));
}

uint8_t lsm9ds1_getFIFOSamples(lsm9ds1_t *lsm9ds1)
{
	return (lsm9ds1_xgReadByte(lsm9ds1, FIFO_SRC) & 0x3F);
}

void lsm9ds1_constrainScales(lsm9ds1_t *lsm9ds1)
{
	if ((lsm9ds1->settings.gyro.scale != 245) && (lsm9ds1->settings.gyro.scale != 500) && 
		(lsm9ds1->settings.gyro.scale != 2000))
	{
		lsm9ds1->settings.gyro.scale = 245;
	}
		
	if ((lsm9ds1->settings.accel.scale != 2) && (lsm9ds1->settings.accel.scale != 4) &&
		(lsm9ds1->settings.accel.scale != 8) && (lsm9ds1->settings.accel.scale != 16))
	{
		lsm9ds1->settings.accel.scale = 2;
	}
		
	if ((lsm9ds1->settings.mag.scale != 4) && (lsm9ds1->settings.mag.scale != 8) &&
		(lsm9ds1->settings.mag.scale != 12) && (lsm9ds1->settings.mag.scale != 16))
	{
		lsm9ds1->settings.mag.scale = 4;
	}
}

void lsm9ds1_xgWriteByte(lsm9ds1_t *lsm9ds1, uint8_t subAddress, uint8_t data)
{
	I2CwriteByte(lsm9ds1->settings.device.i2c, lsm9ds1->settings.device.agAddress, subAddress, data);
}

void lsm9ds1_mWriteByte(lsm9ds1_t *lsm9ds1, uint8_t subAddress, uint8_t data)
{
	return I2CwriteByte(lsm9ds1->settings.device.i2c, lsm9ds1->settings.device.mAddress, subAddress, data);
}

uint8_t lsm9ds1_xgReadByte(lsm9ds1_t *lsm9ds1, uint8_t subAddress)
{
	return I2CreadByte(lsm9ds1->settings.device.i2c, lsm9ds1->settings.device.agAddress, subAddress);
}

uint8_t lsm9ds1_xgReadBytes(lsm9ds1_t *lsm9ds1, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	return I2CreadBytes(lsm9ds1->settings.device.i2c, lsm9ds1->settings.device.agAddress, subAddress, dest, count);
}

uint8_t lsm9ds1_mReadByte(lsm9ds1_t *lsm9ds1, uint8_t subAddress)
{
	return I2CreadByte(lsm9ds1->settings.device.i2c, lsm9ds1->settings.device.mAddress, subAddress);
}

uint8_t lsm9ds1_mReadBytes(lsm9ds1_t *lsm9ds1, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	return I2CreadBytes(lsm9ds1->settings.device.i2c, lsm9ds1->settings.device.mAddress, subAddress, dest, count);
}

// Wire.h read and write protocols
void I2CwriteByte(nrfx_twim_t *twim, uint8_t address, uint8_t subAddress, uint8_t data)
{
	nrfx_err_t err_code;
	uint8_t tmp[2] = {subAddress, data};
	nrfx_twim_xfer_desc_t nrfx_twim_xfer_desc = NRFX_TWIM_XFER_DESC_TX(address, tmp, sizeof(tmp));
	err_code = nrfx_twim_xfer(twim, &nrfx_twim_xfer_desc, 0);
	APP_ERROR_CHECK(err_code);
}

uint8_t I2CreadByte(nrfx_twim_t *twim, uint8_t address, uint8_t subAddress)
{
	uint8_t data;                            // 'data' will store the register data
	I2CreadBytes(twim, address, subAddress, &data, 1);
	return data;                             // Return data read from slave register
}

uint8_t I2CreadBytes(nrfx_twim_t *twim, uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	nrfx_err_t err_code;

	while(nrfx_twim_is_busy(twim));
	nrfx_twim_xfer_desc_t nrfx_twim_xfer_desc = NRFX_TWIM_XFER_DESC_TXRX(address, &subAddress, 1, dest, count);
	err_code = nrfx_twim_xfer(twim, &nrfx_twim_xfer_desc, 0);
	APP_ERROR_CHECK(err_code);

	return count;
}

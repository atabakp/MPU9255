#include "MPU9255.h"

/** Default constructor, uses default I2C address.
 * @see MPU9255_ADDRESS
 */
MPU9255::MPU9255()
{
	devAddr = MPU9255_ADDRESS;
}

/** Specific address constructor.
* @param address I2C address
* @see MPU9255_ADDRESS
*/
MPU9255::MPU9255(uint8_t address)
{
	devAddr = address;
}

/** Power on and prepare for general usage.
* This will activate the device and take it out of sleep mode (which must be done
* after start-up). This function also sets both the accelerometer and the gyroscope
* to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
* the clock source to use the X Gyro for reference, which is slightly better than
* the default internal clock source.
*/
void MPU9255::initialize()
{
	setClockSource(CLOCK_INTERNAL);
	setFullScaleGyroRange(GYRO_FS_250);
	setFullScaleAccelRange(ACCEL_FS_2);
	setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!
}

/** Verify the I2C connection.
* Make sure the device is connected and responds as expected.
* @return True if connection is valid, false otherwise
*/
bool MPU9255::testConnection()
{
	return getDeviceID() == 0x73;
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
// Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
void MPU9255::SelfTest(float * destination)
{
	int16_t rawData[6] = { 0, 0, 0, 0, 0, 0 };
	uint8_t selfTest[6];
	int32_t gAvg[3] = { 0 }, aAvg[3] = { 0 }, aSTAvg[3] = { 0 }, gSTAvg[3] = { 0 };
	float factoryTrim[6];
	uint8_t FS = 0;

	setRate(0x00);// Set gyro sample rate to 1 kHz
	setDLPFMode(DLPF_BW_92);// Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	setFullScaleGyroRange(GYRO_FS_250);// Set full scale range for the gyro to 250 dps
	setAccelDLPF(ACCEL_DLPF_BW_92);// Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	setFullScaleAccelRange(ACCEL_FS_2);// Set full scale range for the accelerometer to 2 g

	for (int ii = 0; ii < 200; ii++)
	{  // get average current values of gyro and acclerometer
		getMotion6(&rawData[0], &rawData[1], &rawData[2], &rawData[3], &rawData[4], &rawData[5]);// Read the six raw data registers into data array
		aAvg[0] += rawData[0];
		aAvg[1] += rawData[1];
		aAvg[2] += rawData[2];

		gAvg[0] += rawData[3];
		gAvg[1] += rawData[4];
		gAvg[2] += rawData[5];
	}

	for (int ii = 0; ii < 3; ii++)
	{  // Get average of 200 values and store as average current readings
		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
	}

	// Configure the accelerometer for self-test
	setAccelXSelfTest(true);// Enable self test on all three axes and set accelerometer range to +/- 2 g
	setAccelYSelfTest(true);
	setAccelZSelfTest(true);

	//writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 0xE0);
	setGyroXSelfTest(true);// Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	setGyroYSelfTest(true);
	setGyroZSelfTest(true);
	delay(25);  // Delay a while to let the device stabilize

	for (uint16_t ii = 0; ii < 200; ii++)
	{  // get average self-test values of gyro and acclerometer
		getMotion6(&rawData[0], &rawData[1], &rawData[2], &rawData[3], &rawData[4], &rawData[5]);// Read the six raw data registers into data array
		aSTAvg[0] += rawData[0];
		aSTAvg[1] += rawData[1];
		aSTAvg[2] += rawData[2];

		gSTAvg[0] += rawData[3];
		gSTAvg[1] += rawData[4];
		gSTAvg[2] += rawData[5];
	}

	for (int ii = 0; ii < 3; ii++)
	{  // Get average of 200 values and store as average self-test readings
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}

	// Configure the gyro and accelerometer for normal operation
	setAccelXSelfTest(false);// Disable self test on all three axes and set accelerometer range to +/- 2 g
	setAccelYSelfTest(false);
	setAccelZSelfTest(false);

	setGyroXSelfTest(false);// Disable self test on all three axes and set gyro range to +/- 250 degrees/s
	setGyroYSelfTest(false);
	setGyroZSelfTest(false);
	delay(25);  // Delay a while to let the device stabilize

				// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	selfTest[0] = getAccelXSelfTestFactoryTrim(); // X-axis accel self-test results
	selfTest[1] = getAccelYSelfTestFactoryTrim(); // Y-axis accel self-test results
	selfTest[2] = getAccelZSelfTestFactoryTrim();// Z-axis accel self-test results

	selfTest[3] = getGyroXSelfTestFactoryTrim(); // X-axis gyro self-test results
	selfTest[4] = getGyroYSelfTestFactoryTrim();  // Y-axis gyro self-test results
	selfTest[5] = getGyroZSelfTestFactoryTrim();  // Z-axis gyro self-test results

	//																	// Retrieve factory self-test value from self-test code reads
	factoryTrim[0] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[0] - 1.0))); // FT[Xa] factory trim calculation
	factoryTrim[1] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[1] - 1.0))); // FT[Ya] factory trim calculation
	factoryTrim[2] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[2] - 1.0))); // FT[Za] factory trim calculation
	factoryTrim[3] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[3] - 1.0))); // FT[Xg] factory trim calculation
	factoryTrim[4] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[4] - 1.0))); // FT[Yg] factory trim calculation
	factoryTrim[5] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[5] - 1.0))); // FT[Zg] factory trim calculation

		 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
		// To get percent, must multiply by 100

	for (int i = 0; i < 3; i++)
	{
		destination[i] = 100.0*((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.;   // Report percent differences
		destination[i + 3] = 100.0*((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3] - 100.; // Report percent differences
	}
}

float MPU9255::getGres()
{
	uint8_t FS = getFullScaleGyroRange();
	switch (FS)
	{
		// Possible gyro scales (and their register bit settings) are:
		// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
	case 0:
		return 250.0 / 32768.0;
		break;
	case 1:
		return 500.0 / 32768.0;
		break;
	case 2:
		return 1000.0 / 32768.0;
		break;
	case 3:
		return 2000.0 / 32768.0;
		break;
	}
}

float MPU9255::getAres()
{
	uint8_t FS = getFullScaleAccelRange();
	switch (FS)
	{
		// Possible accelerometer scales (and their register bit settings) are:
		// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
	case 0:
		return 2.0 / 32768.0;
		break;
	case 1:
		return 4.0 / 32768.0;
		break;
	case 2:
		return 8.0 / 32768.0;
		break;
	case 3:
		return 16.0 / 32768.0;
		break;
	}
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void MPU9255::calibrateAccel(float * dest1, float * dest2)
{
	//void accelgyrocalMPU9250(float * dest1, float * dest2)

	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

	// reset device
	reset(); // Write a one to bit 7 reset bit; toggle reset device
	delay(100);

	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready
	// else use the internal oscillator, bits 2:0 = 001
	setClockSource(1);

	setStandbyXAccelEnabled(false);
	setStandbyYAccelEnabled(false);
	setStandbyZAccelEnabled(false);
	setStandbyXGyroEnabled(false);
	setStandbyYGyroEnabled(false);
	setStandbyZGyroEnabled(false);
	delay(200);

	// Configure device for bias calculation
	// Disable all interrupts
	setIntPLLReadyEnabled(false);
	setIntDMPEnabled(false);
	setIntWakeOnMotionEnabled(false);
	setIntFIFOBufferOverflowEnabled(false);
	setIntFSyncEnabled(false);
	setIntDataReadyEnabled(false);

	// Disable FIFO
	setTempFIFOEnabled(false);
	setXGyroFIFOEnabled(false);
	setYGyroFIFOEnabled(false);
	setZGyroFIFOEnabled(false);
	setAccelFIFOEnabled(false);
	setSlave2FIFOEnabled(false);
	setSlave1FIFOEnabled(false);
	setSlave0FIFOEnabled(false);

	setClockSource(0);   // Turn on internal clock source

	// Disable I2C master
	setMultiMasterEnabled(false);
	setWaitForExternalSensorEnabled(false);
	setSlave3FIFOEnabled(false);
	setSlaveReadWriteTransitionEnabled(false);
	setMasterClockSpeed(0);

	// Disable FIFO and I2C master modes

	setFIFOEnabled(false);
	setI2CMasterModeEnabled(false);
	switchSPIEnabled(false);

	// Reset FIFO and DMP
	resetDMP();
	resetFIFO();
	delay(15);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	setDLPFMode(DLPF_BW_184); // Set low-pass filter to 188 Hz
	setRate(0x00)  // Set sample rate to 1 kHz
		writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t  gyrosensitivity = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

										 // Configure FIFO to capture accelerometer and gyro data for bias calculation
	writeByte(MPU9250_ADDRESS, MPU9250_USER_CTRL, 0x40);   // Enable FIFO
	writeByte(MPU9250_ADDRESS, MPU9250_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

			   // At end of sample accumulation, turn off FIFO sensor read
	writeByte(MPU9250_ADDRESS, MPU9250_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	readBytes(MPU9250_ADDRESS, MPU9250_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count / 12;// How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++)
	{
		int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };
		readBytes(MPU9250_ADDRESS, MPU9250_FIFO_R_W, 12, &data[0]); // read data for averaging
		accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
		accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
		gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
		gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
		gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

		accel_bias[0] += (int32_t)accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t)accel_temp[1];
		accel_bias[2] += (int32_t)accel_temp[2];
		gyro_bias[0] += (int32_t)gyro_temp[0];
		gyro_bias[1] += (int32_t)gyro_temp[1];
		gyro_bias[2] += (int32_t)gyro_temp[2];
	}
	accel_bias[0] /= (int32_t)packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t)packet_count;
	accel_bias[2] /= (int32_t)packet_count;
	gyro_bias[0] /= (int32_t)packet_count;
	gyro_bias[1] /= (int32_t)packet_count;
	gyro_bias[2] /= (int32_t)packet_count;

	if (accel_bias[2] > 0L) { accel_bias[2] -= (int32_t)accelsensitivity; }  // Remove gravity from the z-axis accelerometer bias calculation
	else { accel_bias[2] += (int32_t)accelsensitivity; }

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0] / 4) & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
	data[3] = (-gyro_bias[1] / 4) & 0xFF;
	data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
	data[5] = (-gyro_bias[2] / 4) & 0xFF;

	// Push gyro biases to hardware registers
	writeByte(MPU9250_ADDRESS, MPU9250_XG_OFFSET_H, data[0]);
	writeByte(MPU9250_ADDRESS, MPU9250_XG_OFFSET_L, data[1]);
	writeByte(MPU9250_ADDRESS, MPU9250_YG_OFFSET_H, data[2]);
	writeByte(MPU9250_ADDRESS, MPU9250_YG_OFFSET_L, data[3]);
	writeByte(MPU9250_ADDRESS, MPU9250_ZG_OFFSET_H, data[4]);
	writeByte(MPU9250_ADDRESS, MPU9250_ZG_OFFSET_L, data[5]);

	// Output scaled gyro biases for display in the main program
	dest1[0] = (float)gyro_bias[0] / (float)gyrosensitivity;
	dest1[1] = (float)gyro_bias[1] / (float)gyrosensitivity;
	dest1[2] = (float)gyro_bias[2] / (float)gyrosensitivity;

	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	int32_t accel_bias_reg[3] = { 0, 0, 0 }; // A place to hold the factory accelerometer trim biases
	readBytes(MPU9250_ADDRESS, MPU9250_XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
	readBytes(MPU9250_ADDRESS, MPU9250_YA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[1] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
	readBytes(MPU9250_ADDRESS, MPU9250_ZA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[2] = (int32_t)(((int16_t)data[0] << 8) | data[1]);

	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = { 0, 0, 0 }; // Define array to hold mask bit for each accelerometer bias axis

	for (ii = 0; ii < 3; ii++)
	{
		if ((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1] / 8);
	accel_bias_reg[2] -= (accel_bias[2] / 8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0]) & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1]) & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2]) & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

									 // Apparently this is not working for the acceleration biases in the MPU-9250
									 // Are we handling the temperature correction bit properly?
									 // Push accelerometer biases to hardware registers
									 /*  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
									 writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
									 writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
									 writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
									 writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
									 writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
									 */
									 // Output scaled accelerometer biases for display in the main program
	dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
	dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
	dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;
}

// SMPLRT_DIV register

/** Get gyroscope output rate divider.
* The sensor register output, FIFO output, DMP sampling, Motion detection, Zero
* Motion detection, and Free Fall detection are all based on the Sample Rate.
* The Sample Rate is generated by dividing the gyroscope output rate by
* SMPLRT_DIV:
*
* Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
*
* where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or
* 7), and 1kHz when the DLPF is enabled (see Register 26).
*
* Note: The accelerometer output rate is 1kHz. This means that for a Sample
* Rate greater than 1kHz, the same accelerometer sample may be output to the
* FIFO, DMP, and sensor registers more than once.
*
* For a diagram of the gyroscope and accelerometer signal paths, see Section 8
* of the MPU-6000/MPU-6050 Product Specification document.
*
* @return Current sample rate
* @see SMPLRT_DIV
*/
uint8_t MPU9255::getRate()
{
	return I2Cdev::readByte(devAddr, SMPLRT_DIV, buffer);
	return buffer[0];
}
/** Set gyroscope sample rate divider.
* @param rate New sample rate divider
* @see getRate()
* @see SMPLRT_DIV
*/
void MPU9255::setRate(uint8_t rate)
{
	I2Cdev::writeByte(devAddr, SMPLRT_DIV, rate);
}

// CONFIG register

/** Get external FSYNC configuration.
* Configures the external Frame Synchronization (FSYNC) pin sampling. An
* external signal connected to the FSYNC pin can be sampled by configuring
* EXT_SYNC_SET. Signal changes to the FSYNC pin are latched so that short
* strobes may be captured. The latched FSYNC signal will be sampled at the
* Sampling Rate, as defined in register 25. After sampling, the latch will
* reset to the current FSYNC signal state.
*
* The sampled value will be reported in place of the least significant bit in
* a sensor data register determined by the value of EXT_SYNC_SET according to
* the following table.
*
* <pre>
* EXT_SYNC_SET | FSYNC Bit Location
* -------------+-------------------
* 0            | Input disabled
* 1            | TEMP_OUT_L[0]
* 2            | GYRO_XOUT_L[0]
* 3            | GYRO_YOUT_L[0]
* 4            | GYRO_ZOUT_L[0]
* 5            | ACCEL_XOUT_L[0]
* 6            | ACCEL_YOUT_L[0]
* 7            | ACCEL_ZOUT_L[0]
* </pre>
*
* @return FSYNC configuration value
*/
uint8_t MPU9255::getExternalFrameSync()
{
	I2Cdev::readBits(devAddr, CONFIG, CFG_EXT_SYNC_SET_BIT, CFG_EXT_SYNC_SET_LENGTH, buffer);
	return buffer[0];
}
/** Set external FSYNC configuration.
* @see getExternalFrameSync()
* @see CONFIG
* @param sync New FSYNC configuration value
*/
void MPU9255::setExternalFrameSync(uint8_t sync)
{
	I2Cdev::writeBits(devAddr, CONFIG, CFG_EXT_SYNC_SET_BIT, CFG_EXT_SYNC_SET_LENGTH, sync);
}
/** Get digital low-pass filter configuration.
* The DLPF_CFG parameter sets the digital low pass filter configuration. It
* also determines the internal sampling rate used by the device as shown in
* the table below.
*
* Note: The accelerometer output rate is 1kHz. This means that for a Sample
* Rate greater than 1kHz, the same accelerometer sample may be output to the
* FIFO, DMP, and sensor registers more than once.
*
* <pre>
*          |   ACCELEROMETER    |           GYROSCOPE
* DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
* ---------+-----------+--------+-----------+--------+-------------
* 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
* 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
* 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
* 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
* 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
* 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
* 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
* 7        |   -- Reserved --   |   -- Reserved --   | Reserved
* </pre>
*
* @return DLFP configuration
* @see CONFIG
* @see CFG_DLPF_CFG_BIT
* @see CFG_DLPF_CFG_LENGTH
*/
uint8_t MPU9255::getDLPFMode()
{
	//I2Cdev::readBits(devAddr, CONFIG, CFG_DLPF_CFG_BIT, CFG_DLPF_CFG_LENGTH, buffer);
	I2Cdev::readByte(devAddr, CONFIG, buffer, 100);
	return buffer[0];
}
/** Set digital low-pass filter configuration.
* @param mode New DLFP configuration setting
* @see getDLPFBandwidth()
* @see DLPF_BW_256
* @see CONFIG
* @see CFG_DLPF_CFG_BIT
* @see CFG_DLPF_CFG_LENGTH
*/
void MPU9255::setDLPFMode(uint8_t mode)
{
	I2Cdev::writeBits(devAddr, CONFIG, CFG_DLPF_CFG_BIT, CFG_DLPF_CFG_LENGTH, mode);
}

// GYRO_CONFIG register

/** Get self-test enabled setting for gyroscope X axis.
* @return Self-test enabled value
* @see GYRO_CONFIG
*/
bool MPU9255::getGyroXSelfTest()
{
	I2Cdev::readBit(devAddr, GYRO_CONFIG, GCONFIG_XG_ST_BIT, buffer);
	return buffer[0];
}
/** Get self-test enabled setting for gyroscope X axis.
* @param enabled Self-test enabled value
* @see GYRO_CONFIG
*/
void MPU9255::setGyroXSelfTest(bool enabled)
{
	I2Cdev::writeBit(devAddr, GYRO_CONFIG, GCONFIG_XG_ST_BIT, enabled);
}
/** Get self-test enabled value for gyroscope Y axis.
* @return Self-test enabled value
* @see GYRO_CONFIG
*/
bool MPU9255::getGyroYSelfTest()
{
	I2Cdev::readBit(devAddr, GYRO_CONFIG, GCONFIG_YG_ST_BIT, buffer);
	return buffer[0];
}
/** Get self-test enabled value for gyroscope Y axis.
* @param enabled Self-test enabled value
* @see GYRO_CONFIG
*/
void MPU9255::setGyroYSelfTest(bool enabled)
{
	I2Cdev::writeBit(devAddr, GYRO_CONFIG, GCONFIG_YG_ST_BIT, enabled);
}
/** Get self-test enabled value for gyroscope Z axis.
* @return Self-test enabled value
* @see GYRO_CONFIG
*/
bool MPU9255::getGyroZSelfTest()
{
	I2Cdev::readBit(devAddr, GYRO_CONFIG, GCONFIG_ZG_ST_BIT, buffer);
	return buffer[0];
}
/** Set self-test enabled value for gyroscope Z axis.
* @param enabled Self-test enabled value
* @see GYRO_CONFIG
*/
void MPU9255::setGyroZSelfTest(bool enabled)
{
	I2Cdev::writeBit(devAddr, GYRO_CONFIG, GCONFIG_ZG_ST_BIT, enabled);
}

/** Get full-scale gyroscope range.
* The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
* as described in the table below.
*
* <pre>
* 0 = +/- 250 degrees/sec
* 1 = +/- 500 degrees/sec
* 2 = +/- 1000 degrees/sec
* 3 = +/- 2000 degrees/sec
* </pre>
*
* @return Current full-scale gyroscope range setting
* @see GYRO_FS_250
* @see GYRO_CONFIG
* @see GCONFIG_FS_SEL_BIT
* @see GCONFIG_FS_SEL_LENGTH
*/
uint8_t MPU9255::getFullScaleGyroRange()
{
	I2Cdev::readBits(devAddr, GYRO_CONFIG, GCONFIG_FS_SEL_BIT, GCONFIG_FS_SEL_LENGTH, buffer);
	return buffer[0];
}

/** Set full-scale gyroscope range.
* @param range New full-scale gyroscope range value
* @see getFullScaleRange()
* @see GYRO_FS_250
* @see GYRO_CONFIG
* @see GCONFIG_FS_SEL_BIT
* @see GCONFIG_FS_SEL_LENGTH
*/
void MPU9255::setFullScaleGyroRange(uint8_t range)
{
	I2Cdev::writeBits(devAddr, GYRO_CONFIG, GCONFIG_FS_SEL_BIT, GCONFIG_FS_SEL_LENGTH, range);
}

// SELF TEST FACTORY TRIM VALUES

/** Get self-test factory trim value for accelerometer X axis.
* @return factory trim value
* @see SELF_TEST_X_ACCEL
*/
uint8_t MPU9255::getAccelXSelfTestFactoryTrim()
{
	I2Cdev::readByte(devAddr, SELF_TEST_X_ACCEL, &buffer[0]);
	return buffer[0];
}

/** Get self-test factory trim value for accelerometer Y axis.
* @return factory trim value
* @see SELF_TEST_Y_ACCEL
*/
uint8_t MPU9255::getAccelYSelfTestFactoryTrim()
{
	I2Cdev::readByte(devAddr, SELF_TEST_Y_ACCEL, &buffer[0]);
	return buffer[0];
}

/** Get self-test factory trim value for accelerometer Z axis.
* @return factory trim value
* @see SELF_TEST_Z_ACCEL
*/
uint8_t MPU9255::getAccelZSelfTestFactoryTrim()
{
	I2Cdev::readBytes(devAddr, SELF_TEST_Z_ACCEL, 2, buffer);
	return buffer[0];
}

/** Get self-test factory trim value for gyro X axis.
* @return factory trim value
* @see MPU6050_RA_SELF_TEST_X
*/
uint8_t MPU9255::getGyroXSelfTestFactoryTrim()
{
	I2Cdev::readByte(devAddr, SELF_TEST_X_GYRO, buffer);
	return buffer[0];
}

/** Get self-test factory trim value for gyro Y axis.
* @return factory trim value
* @see MPU6050_RA_SELF_TEST_Y
*/
uint8_t MPU9255::getGyroYSelfTestFactoryTrim()
{
	I2Cdev::readByte(devAddr, SELF_TEST_Y_GYRO, buffer);
	return buffer[0];
}

/** Get self-test factory trim value for gyro Z axis.
* @return factory trim value
* @see MPU6050_RA_SELF_TEST_Z
*/
uint8_t MPU9255::getGyroZSelfTestFactoryTrim()
{
	I2Cdev::readByte(devAddr, SELF_TEST_Z_GYRO, buffer);
	return buffer[0];
}

// ACCEL_CONFIG register

/** Get self-test enabled setting for accelerometer X axis.
* @return Self-test enabled value
* @see ACCEL_CONFIG
*/
bool MPU9255::getAccelXSelfTest()
{
	I2Cdev::readBit(devAddr, ACCEL_CONFIG, ACONFIG_XA_ST_BIT, buffer);
	return buffer[0];
}
/** Get self-test enabled setting for accelerometer X axis.
* @param enabled Self-test enabled value
* @see ACCEL_CONFIG
*/
void MPU9255::setAccelXSelfTest(bool enabled)
{
	I2Cdev::writeBit(devAddr, ACCEL_CONFIG, ACONFIG_XA_ST_BIT, enabled);
}
/** Get self-test enabled value for accelerometer Y axis.
* @return Self-test enabled value
* @see ACCEL_CONFIG
*/
bool MPU9255::getAccelYSelfTest()
{
	I2Cdev::readBit(devAddr, ACCEL_CONFIG, ACONFIG_YA_ST_BIT, buffer);
	return buffer[0];
}
/** Get self-test enabled value for accelerometer Y axis.
* @param enabled Self-test enabled value
* @see ACCEL_CONFIG
*/
void MPU9255::setAccelYSelfTest(bool enabled)
{
	I2Cdev::writeBit(devAddr, ACCEL_CONFIG, ACONFIG_YA_ST_BIT, enabled);
}
/** Get self-test enabled value for accelerometer Z axis.
* @return Self-test enabled value
* @see ACCEL_CONFIG
*/
bool MPU9255::getAccelZSelfTest()
{
	I2Cdev::readBit(devAddr, ACCEL_CONFIG, ACONFIG_ZA_ST_BIT, buffer);
	return buffer[0];
}
/** Set self-test enabled value for accelerometer Z axis.
* @param enabled Self-test enabled value
* @see ACCEL_CONFIG
*/
void MPU9255::setAccelZSelfTest(bool enabled)
{
	I2Cdev::writeBit(devAddr, ACCEL_CONFIG, ACONFIG_ZA_ST_BIT, enabled);
}
/** Get full-scale accelerometer range.
* The FS_SEL parameter allows setting the full-scale range of the accelerometer
* sensors, as described in the table below.
*
* <pre>
* 0 = +/- 2g
* 1 = +/- 4g
* 2 = +/- 8g
* 3 = +/- 16g
* </pre>
*
* @return Current full-scale accelerometer range setting
* @see ACCEL_FS_2
* @see ACCEL_CONFIG
* @see ACONFIG_AFS_SEL_BIT
* @see ACONFIG_AFS_SEL_LENGTH
*/
uint8_t MPU9255::getFullScaleAccelRange()
{
	I2Cdev::readBits(devAddr, ACCEL_CONFIG, ACONFIG_AFS_SEL_BIT, ACONFIG_AFS_SEL_LENGTH, buffer);
	return buffer[0];
}
/** Set full-scale accelerometer range.
* @param range New full-scale accelerometer range setting
* @see getFullScaleAccelRange()
*/
void MPU9255::setFullScaleAccelRange(uint8_t range)
{
	I2Cdev::writeBits(devAddr, ACCEL_CONFIG, ACONFIG_AFS_SEL_BIT, ACONFIG_AFS_SEL_LENGTH, range);
}

void MPU9255::setAccelDLPF(uint8_t mode)
{
	I2Cdev::writeBits(devAddr, ACCEL_CONFIG2, ACONFIG2_DLPF_CFG_BIT, ACONFIG2_DLPF_CFG_LENGTH, mode);
}

// MOT_THR register

/** Get motion detection event acceleration threshold.
* This register configures the detection threshold for Motion interrupt
* generation. The unit of MOT_THR is 1LSB = 2mg. Motion is detected when the
* absolute value of any of the accelerometer measurements exceeds this Motion
* detection threshold. This condition increments the Motion detection duration
* counter (Register 32). The Motion detection interrupt is triggered when the
* Motion Detection counter reaches the time count specified in MOT_DUR
* (Register 32).
*
* The Motion interrupt will indicate the axis and polarity of detected motion
* in MOT_DETECT_STATUS (Register 97).
*
* For more details on the Motion detection interrupt, see Section 8.3 of the
* MPU-6000/MPU-6050 Product Specification document as well as Registers 56 and
* 58 of this document.
*
* @return Current motion detection acceleration threshold value (LSB = 2mg)
* @see WOM_THR
*/
uint8_t MPU9255::getMotionDetectionThreshold()
{
	I2Cdev::readByte(devAddr, WOM_THR, buffer);
	return buffer[0];
}
/** Set free-fall event acceleration threshold.
* @param threshold New motion detection acceleration threshold value (LSB = 2mg)
* @see getMotionDetectionThreshold()
* @see WOM_THR
*/
void MPU9255::setMotionDetectionThreshold(uint8_t threshold)
{
	I2Cdev::writeByte(devAddr, WOM_THR, threshold);
}

// FIFO_EN register

/** Get temperature FIFO enabled value.
* When set to 1, this bit enables TEMP_OUT_H and TEMP_OUT_L (Registers 65 and
* 66) to be written into the FIFO buffer.
* @return Current temperature FIFO enabled value
* @see FIFO_EN
*/
bool MPU9255::getTempFIFOEnabled()
{
	I2Cdev::readBit(devAddr, FIFO_EN, TEMP_FIFO_EN_BIT, buffer);
	return buffer[0];
}
/** Set temperature FIFO enabled value.
* @param enabled New temperature FIFO enabled value
* @see getTempFIFOEnabled()
* @see FIFO_EN
*/
void MPU9255::setTempFIFOEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, FIFO_EN, TEMP_FIFO_EN_BIT, enabled);
}
/** Get gyroscope X-axis FIFO enabled value.
* When set to 1, this bit enables GYRO_XOUT_H and GYRO_XOUT_L (Registers 67 and
* 68) to be written into the FIFO buffer.
* @return Current gyroscope X-axis FIFO enabled value
* @see FIFO_EN
*/
bool MPU9255::getXGyroFIFOEnabled()
{
	I2Cdev::readBit(devAddr, FIFO_EN, XG_FIFO_EN_BIT, buffer);
	return buffer[0];
}
/** Set gyroscope X-axis FIFO enabled value.
* @param enabled New gyroscope X-axis FIFO enabled value
* @see getXGyroFIFOEnabled()
* @see FIFO_EN
*/
void MPU9255::setXGyroFIFOEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, FIFO_EN, XG_FIFO_EN_BIT, enabled);
}
/** Get gyroscope Y-axis FIFO enabled value.
* When set to 1, this bit enables GYRO_YOUT_H and GYRO_YOUT_L (Registers 69 and
* 70) to be written into the FIFO buffer.
* @return Current gyroscope Y-axis FIFO enabled value
* @see FIFO_EN
*/
bool MPU9255::getYGyroFIFOEnabled()
{
	I2Cdev::readBit(devAddr, FIFO_EN, YG_FIFO_EN_BIT, buffer);
	return buffer[0];
}
/** Set gyroscope Y-axis FIFO enabled value.
* @param enabled New gyroscope Y-axis FIFO enabled value
* @see getYGyroFIFOEnabled()
* @see FIFO_EN
*/
void MPU9255::setYGyroFIFOEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, FIFO_EN, YG_FIFO_EN_BIT, enabled);
}
/** Get gyroscope Z-axis FIFO enabled value.
* When set to 1, this bit enables GYRO_ZOUT_H and GYRO_ZOUT_L (Registers 71 and
* 72) to be written into the FIFO buffer.
* @return Current gyroscope Z-axis FIFO enabled value
* @see FIFO_EN
*/
bool MPU9255::getZGyroFIFOEnabled()
{
	I2Cdev::readBit(devAddr, FIFO_EN, ZG_FIFO_EN_BIT, buffer);
	return buffer[0];
}
/** Set gyroscope Z-axis FIFO enabled value.
* @param enabled New gyroscope Z-axis FIFO enabled value
* @see getZGyroFIFOEnabled()
* @see FIFO_EN
*/
void MPU9255::setZGyroFIFOEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, FIFO_EN, ZG_FIFO_EN_BIT, enabled);
}
/** Get accelerometer FIFO enabled value.
* When set to 1, this bit enables ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H,
* ACCEL_YOUT_L, ACCEL_ZOUT_H, and ACCEL_ZOUT_L (Registers 59 to 64) to be
* written into the FIFO buffer.
* @return Current accelerometer FIFO enabled value
* @see FIFO_EN
*/
bool MPU9255::getAccelFIFOEnabled()
{
	I2Cdev::readBit(devAddr, FIFO_EN, ACCEL_FIFO_EN_BIT, buffer);
	return buffer[0];
}
/** Set accelerometer FIFO enabled value.
* @param enabled New accelerometer FIFO enabled value
* @see getAccelFIFOEnabled()
* @see FIFO_EN
*/
void MPU9255::setAccelFIFOEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, FIFO_EN, ACCEL_FIFO_EN_BIT, enabled);
}
/** Get Slave 2 FIFO enabled value.
* When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
* associated with Slave 2 to be written into the FIFO buffer.
* @return Current Slave 2 FIFO enabled value
* @see FIFO_EN
*/
bool MPU9255::getSlave2FIFOEnabled()
{
	I2Cdev::readBit(devAddr, FIFO_EN, SLV2_FIFO_EN_BIT, buffer);
	return buffer[0];
}
/** Set Slave 2 FIFO enabled value.
* @param enabled New Slave 2 FIFO enabled value
* @see getSlave2FIFOEnabled()
* @see FIFO_EN
*/
void MPU9255::setSlave2FIFOEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, FIFO_EN, SLV2_FIFO_EN_BIT, enabled);
}
/** Get Slave 1 FIFO enabled value.
* When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
* associated with Slave 1 to be written into the FIFO buffer.
* @return Current Slave 1 FIFO enabled value
* @see FIFO_EN
*/
bool MPU9255::getSlave1FIFOEnabled()
{
	I2Cdev::readBit(devAddr, FIFO_EN, SLV1_FIFO_EN_BIT, buffer);
	return buffer[0];
}
/** Set Slave 1 FIFO enabled value.
* @param enabled New Slave 1 FIFO enabled value
* @see getSlave1FIFOEnabled()
* @see FIFO_EN
*/
void MPU9255::setSlave1FIFOEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, FIFO_EN, SLV1_FIFO_EN_BIT, enabled);
}
/** Get Slave 0 FIFO enabled value.
* When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
* associated with Slave 0 to be written into the FIFO buffer.
* @return Current Slave 0 FIFO enabled value
* @see FIFO_EN
*/
bool MPU9255::getSlave0FIFOEnabled()
{
	I2Cdev::readBit(devAddr, FIFO_EN, SLV0_FIFO_EN_BIT, buffer);
	return buffer[0];
}
/** Set Slave 0 FIFO enabled value.
* @param enabled New Slave 0 FIFO enabled value
* @see getSlave0FIFOEnabled()
* @see FIFO_EN
*/
void MPU9255::setSlave0FIFOEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, FIFO_EN, SLV0_FIFO_EN_BIT, enabled);
}

// I2C_MST_CTRL register

/** Get multi-master enabled value.
* Multi-master capability allows multiple I2C masters to operate on the same
* bus. In circuits where multi-master capability is required, set MULT_MST_EN
* to 1. This will increase current drawn by approximately 30uA.
*
* In circuits where multi-master capability is required, the state of the I2C
* bus must always be monitored by each separate I2C Master. Before an I2C
* Master can assume arbitration of the bus, it must first confirm that no other
* I2C Master has arbitration of the bus. When MULT_MST_EN is set to 1, the
* MPU-60X0's bus arbitration detection logic is turned on, enabling it to
* detect when the bus is available.
*
* @return Current multi-master enabled value
* @see I2C_MST_CTRL
*/
bool MPU9255::getMultiMasterEnabled()
{
	I2Cdev::readBit(devAddr, I2C_MST_CTRL, MULT_MST_EN_BIT, buffer);
	return buffer[0];
}
/** Set multi-master enabled value.
* @param enabled New multi-master enabled value
* @see getMultiMasterEnabled()
* @see I2C_MST_CTRL
*/
void MPU9255::setMultiMasterEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, I2C_MST_CTRL, MULT_MST_EN_BIT, enabled);
}
/** Get wait-for-external-sensor-data enabled value.
* When the WAIT_FOR_ES bit is set to 1, the Data Ready interrupt will be
* delayed until External Sensor data from the Slave Devices are loaded into the
* EXT_SENS_DATA registers. This is used to ensure that both the internal sensor
* data (i.e. from gyro and accel) and external sensor data have been loaded to
* their respective data registers (i.e. the data is synced) when the Data Ready
* interrupt is triggered.
*
* @return Current wait-for-external-sensor-data enabled value
* @see I2C_MST_CTRL
*/
bool MPU9255::getWaitForExternalSensorEnabled()
{
	I2Cdev::readBit(devAddr, I2C_MST_CTRL, WAIT_FOR_ES_BIT, buffer);
	return buffer[0];
}
/** Set wait-for-external-sensor-data enabled value.
* @param enabled New wait-for-external-sensor-data enabled value
* @see getWaitForExternalSensorEnabled()
* @see I2C_MST_CTRL
*/
void MPU9255::setWaitForExternalSensorEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, I2C_MST_CTRL, WAIT_FOR_ES_BIT, enabled);
}
/** Get Slave 3 FIFO enabled value.
* When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
* associated with Slave 3 to be written into the FIFO buffer.
* @return Current Slave 3 FIFO enabled value
* @see MST_CTRL
*/
bool MPU9255::getSlave3FIFOEnabled()
{
	I2Cdev::readBit(devAddr, I2C_MST_CTRL, SLV_3_FIFO_EN_BIT, buffer);
	return buffer[0];
}
/** Set Slave 3 FIFO enabled value.
* @param enabled New Slave 3 FIFO enabled value
* @see getSlave3FIFOEnabled()
* @see MST_CTRL
*/
void MPU9255::setSlave3FIFOEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, I2C_MST_CTRL, SLV_3_FIFO_EN_BIT, enabled);
}
/** Get slave read/write transition enabled value.
* The I2C_MST_P_NSR bit configures the I2C Master's transition from one slave
* read to the next slave read. If the bit equals 0, there will be a restart
* between reads. If the bit equals 1, there will be a stop followed by a start
* of the following read. When a write transaction follows a read transaction,
* the stop followed by a start of the successive write will be always used.
*
* @return Current slave read/write transition enabled value
* @see I2C_MST_CTRL
*/
bool MPU9255::getSlaveReadWriteTransitionEnabled()
{
	I2Cdev::readBit(devAddr, I2C_MST_CTRL, I2C_MST_P_NSR_BIT, buffer);
	return buffer[0];
}
/** Set slave read/write transition enabled value.
* @param enabled New slave read/write transition enabled value
* @see getSlaveReadWriteTransitionEnabled()
* @see I2C_MST_CTRL
*/
void MPU9255::setSlaveReadWriteTransitionEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, I2C_MST_CTRL, I2C_MST_P_NSR_BIT, enabled);
}
/** Get I2C master clock speed.
* I2C_MST_CLK is a 4 bit unsigned value which configures a divider on the
* MPU-60X0 internal 8MHz clock. It sets the I2C master clock speed according to
* the following table:
*
* <pre>
* I2C_MST_CLK | I2C Master Clock Speed | 8MHz Clock Divider
* ------------+------------------------+-------------------
* 0           | 348kHz                 | 23
* 1           | 333kHz                 | 24
* 2           | 320kHz                 | 25
* 3           | 308kHz                 | 26
* 4           | 296kHz                 | 27
* 5           | 286kHz                 | 28
* 6           | 276kHz                 | 29
* 7           | 267kHz                 | 30
* 8           | 258kHz                 | 31
* 9           | 500kHz                 | 16
* 10          | 471kHz                 | 17
* 11          | 444kHz                 | 18
* 12          | 421kHz                 | 19
* 13          | 400kHz                 | 20
* 14          | 381kHz                 | 21
* 15          | 364kHz                 | 22
* </pre>
*
* @return Current I2C master clock speed
* @see I2C_MST_CTRL
*/
uint8_t MPU9255::getMasterClockSpeed()
{
	I2Cdev::readBits(devAddr, I2C_MST_CTRL, I2C_MST_CLK_BIT, I2C_MST_CLK_LENGTH, buffer);
	return buffer[0];
}
/** Set I2C master clock speed.
* @reparam speed Current I2C master clock speed
* @see I2C_MST_CTRL
*/
void MPU9255::setMasterClockSpeed(uint8_t speed)
{
	I2Cdev::writeBits(devAddr, I2C_MST_CTRL, I2C_MST_CLK_BIT, I2C_MST_CLK_LENGTH, speed);
}

// I2C_SLV* registers (Slave 0-3)

/** Get the I2C address of the specified slave (0-3).
* Note that Bit 7 (MSB) controls read/write mode. If Bit 7 is set, it's a read
* operation, and if it is cleared, then it's a write operation. The remaining
* bits (6-0) are the 7-bit device address of the slave device.
*
* In read mode, the result of the read is placed in the lowest available
* EXT_SENS_DATA register. For further information regarding the allocation of
* read results, please refer to the EXT_SENS_DATA register description
* (Registers 73 � 96).
*
* The MPU-6050 supports a total of five slaves, but Slave 4 has unique
* characteristics, and so it has its own functions (getSlave4* and setSlave4*).
*
* I2C data transactions are performed at the Sample Rate, as defined in
* Register 25. The user is responsible for ensuring that I2C data transactions
* to and from each enabled Slave can be completed within a single period of the
* Sample Rate.
*
* The I2C slave access rate can be reduced relative to the Sample Rate. This
* reduced access rate is determined by I2C_MST_DLY (Register 52). Whether a
* slave's access rate is reduced relative to the Sample Rate is determined by
* I2C_MST_DELAY_CTRL (Register 103).
*
* The processing order for the slaves is fixed. The sequence followed for
* processing the slaves is Slave 0, Slave 1, Slave 2, Slave 3 and Slave 4. If a
* particular Slave is disabled it will be skipped.
*
* Each slave can either be accessed at the sample rate or at a reduced sample
* rate. In a case where some slaves are accessed at the Sample Rate and some
* slaves are accessed at the reduced rate, the sequence of accessing the slaves
* (Slave 0 to Slave 4) is still followed. However, the reduced rate slaves will
* be skipped if their access rate dictates that they should not be accessed
* during that particular cycle. For further information regarding the reduced
* access rate, please refer to Register 52. Whether a slave is accessed at the
* Sample Rate or at the reduced rate is determined by the Delay Enable bits in
* Register 103.
*
* @param num Slave number (0-3)
* @return Current address for specified slave
* @see I2C_SLV0_ADDR
*/
uint8_t MPU9255::getSlaveAddress(uint8_t num)
{
	if (num > 3) return 0;
	I2Cdev::readByte(devAddr, I2C_SLV0_ADDR + num * 3, buffer);
	return buffer[0];
}
/** Set the I2C address of the specified slave (0-3).
* @param num Slave number (0-3)
* @param address New address for specified slave
* @see getSlaveAddress()
* @see I2C_SLV0_ADDR
*/
void MPU9255::setSlaveAddress(uint8_t num, uint8_t address)
{
	if (num > 3) return;
	I2Cdev::writeByte(devAddr, I2C_SLV0_ADDR + num * 3, address);
}
/** Get the active internal register for the specified slave (0-3).
* Read/write operations for this slave will be done to whatever internal
* register address is stored in this MPU register.
*
* The MPU-6050 supports a total of five slaves, but Slave 4 has unique
* characteristics, and so it has its own functions.
*
* @param num Slave number (0-3)
* @return Current active register for specified slave
* @see I2C_SLV0_REG
*/
uint8_t MPU9255::getSlaveRegister(uint8_t num)
{
	if (num > 3) return 0;
	I2Cdev::readByte(devAddr, I2C_SLV0_REG + num * 3, buffer);
	return buffer[0];
}
/** Set the active internal register for the specified slave (0-3).
* @param num Slave number (0-3)
* @param reg New active register for specified slave
* @see getSlaveRegister()
* @see I2C_SLV0_REG
*/
void MPU9255::setSlaveRegister(uint8_t num, uint8_t reg)
{
	if (num > 3) return;
	I2Cdev::writeByte(devAddr, I2C_SLV0_REG + num * 3, reg);
}
/** Get the enabled value for the specified slave (0-3).
* When set to 1, this bit enables Slave 0 for data transfer operations. When
* cleared to 0, this bit disables Slave 0 from data transfer operations.
* @param num Slave number (0-3)
* @return Current enabled value for specified slave
* @see I2C_SLV0_CTRL
*/
bool MPU9255::getSlaveEnabled(uint8_t num)
{
	if (num > 3) return 0;
	I2Cdev::readBit(devAddr, I2C_SLV0_CTRL + num * 3, I2C_SLV_EN_BIT, buffer);
	return buffer[0];
}
/** Set the enabled value for the specified slave (0-3).
* @param num Slave number (0-3)
* @param enabled New enabled value for specified slave
* @see getSlaveEnabled()
* @see I2C_SLV0_CTRL
*/
void MPU9255::setSlaveEnabled(uint8_t num, bool enabled)
{
	if (num > 3) return;
	I2Cdev::writeBit(devAddr, I2C_SLV0_CTRL + num * 3, I2C_SLV_EN_BIT, enabled);
}
/** Get word pair byte-swapping enabled for the specified slave (0-3).
* When set to 1, this bit enables byte swapping. When byte swapping is enabled,
* the high and low bytes of a word pair are swapped. Please refer to
* I2C_SLV0_GRP for the pairing convention of the word pairs. When cleared to 0,
* bytes transferred to and from Slave 0 will be written to EXT_SENS_DATA
* registers in the order they were transferred.
*
* @param num Slave number (0-3)
* @return Current word pair byte-swapping enabled value for specified slave
* @see I2C_SLV0_CTRL
*/
bool MPU9255::getSlaveWordByteSwap(uint8_t num)
{
	if (num > 3) return 0;
	I2Cdev::readBit(devAddr, I2C_SLV0_CTRL + num * 3, I2C_SLV_BYTE_SW_BIT, buffer);
	return buffer[0];
}
/** Set word pair byte-swapping enabled for the specified slave (0-3).
* @param num Slave number (0-3)
* @param enabled New word pair byte-swapping enabled value for specified slave
* @see getSlaveWordByteSwap()
* @see I2C_SLV0_CTRL
*/
void MPU9255::setSlaveWordByteSwap(uint8_t num, bool enabled)
{
	if (num > 3) return;
	I2Cdev::writeBit(devAddr, I2C_SLV0_CTRL + num * 3, I2C_SLV_BYTE_SW_BIT, enabled);
}
/** Get write mode for the specified slave (0-3).
* When set to 1, the transaction will read or write data only. When cleared to
* 0, the transaction will write a register address prior to reading or writing
* data. This should equal 0 when specifying the register address within the
* Slave device to/from which the ensuing data transaction will take place.
*
* @param num Slave number (0-3)
* @return Current write mode for specified slave (0 = register address + data, 1 = data only)
* @see I2C_SLV0_CTRL
*/
bool MPU9255::getSlaveWriteMode(uint8_t num)
{
	if (num > 3) return 0;
	I2Cdev::readBit(devAddr, I2C_SLV0_CTRL + num * 3, I2C_SLV_REG_DIS_BIT, buffer);
	return buffer[0];
}
/** Set write mode for the specified slave (0-3).
* @param num Slave number (0-3)
* @param mode New write mode for specified slave (0 = register address + data, 1 = data only)
* @see getSlaveWriteMode()
* @see I2C_SLV0_CTRL
*/
void MPU9255::setSlaveWriteMode(uint8_t num, bool mode)
{
	if (num > 3) return;
	I2Cdev::writeBit(devAddr, I2C_SLV0_CTRL + num * 3, I2C_SLV_REG_DIS_BIT, mode);
}
/** Get word pair grouping order offset for the specified slave (0-3).
* This sets specifies the grouping order of word pairs received from registers.
* When cleared to 0, bytes from register addresses 0 and 1, 2 and 3, etc (even,
* then odd register addresses) are paired to form a word. When set to 1, bytes
* from register addresses are paired 1 and 2, 3 and 4, etc. (odd, then even
* register addresses) are paired to form a word.
*
* @param num Slave number (0-3)
* @return Current word pair grouping order offset for specified slave
* @see I2C_SLV0_CTRL
*/
bool MPU9255::getSlaveWordGroupOffset(uint8_t num)
{
	if (num > 3) return 0;
	I2Cdev::readBit(devAddr, I2C_SLV0_CTRL + num * 3, I2C_SLV_GRP_BIT, buffer);
	return buffer[0];
}
/** Set word pair grouping order offset for the specified slave (0-3).
* @param num Slave number (0-3)
* @param enabled New word pair grouping order offset for specified slave
* @see getSlaveWordGroupOffset()
* @see I2C_SLV0_CTRL
*/
void MPU9255::setSlaveWordGroupOffset(uint8_t num, bool enabled)
{
	if (num > 3) return;
	I2Cdev::writeBit(devAddr, I2C_SLV0_CTRL + num * 3, I2C_SLV_GRP_BIT, enabled);
}
/** Get number of bytes to read for the specified slave (0-3).
* Specifies the number of bytes transferred to and from Slave 0. Clearing this
* bit to 0 is equivalent to disabling the register by writing 0 to I2C_SLV0_EN.
* @param num Slave number (0-3)
* @return Number of bytes to read for specified slave
* @see I2C_SLV0_CTRL
*/
uint8_t MPU9255::getSlaveDataLength(uint8_t num)
{
	if (num > 3) return 0;
	I2Cdev::readBits(devAddr, I2C_SLV0_CTRL + num * 3, I2C_SLV_LEN_BIT, I2C_SLV_LEN_LENGTH, buffer);
	return buffer[0];
}
/** Set number of bytes to read for the specified slave (0-3).
* @param num Slave number (0-3)
* @param length Number of bytes to read for specified slave
* @see getSlaveDataLength()
* @see I2C_SLV0_CTRL
*/
void MPU9255::setSlaveDataLength(uint8_t num, uint8_t length)
{
	if (num > 3) return;
	I2Cdev::writeBits(devAddr, I2C_SLV0_CTRL + num * 3, I2C_SLV_LEN_BIT, I2C_SLV_LEN_LENGTH, length);
}

// I2C_SLV* registers (Slave 4)

/** Get the I2C address of Slave 4.
* Note that Bit 7 (MSB) controls read/write mode. If Bit 7 is set, it's a read
* operation, and if it is cleared, then it's a write operation. The remaining
* bits (6-0) are the 7-bit device address of the slave device.
*
* @return Current address for Slave 4
* @see getSlaveAddress()
* @see I2C_SLV4_ADDR
*/
uint8_t MPU9255::getSlave4Address()
{
	I2Cdev::readByte(devAddr, I2C_SLV4_ADDR, buffer);
	return buffer[0];
}
/** Set the I2C address of Slave 4.
* @param address New address for Slave 4
* @see getSlave4Address()
* @see I2C_SLV4_ADDR
*/
void MPU9255::setSlave4Address(uint8_t address)
{
	I2Cdev::writeByte(devAddr, I2C_SLV4_ADDR, address);
}
/** Get the active internal register for the Slave 4.
* Read/write operations for this slave will be done to whatever internal
* register address is stored in this MPU register.
*
* @return Current active register for Slave 4
* @see I2C_SLV4_REG
*/
uint8_t MPU9255::getSlave4Register()
{
	I2Cdev::readByte(devAddr, I2C_SLV4_REG, buffer);
	return buffer[0];
}
/** Set the active internal register for Slave 4.
* @param reg New active register for Slave 4
* @see getSlave4Register()
* @see I2C_SLV4_REG
*/
void MPU9255::setSlave4Register(uint8_t reg)
{
	I2Cdev::writeByte(devAddr, I2C_SLV4_REG, reg);
}
/** Set new byte to write to Slave 4.
* This register stores the data to be written into the Slave 4. If I2C_SLV4_RW
* is set 1 (set to read), this register has no effect.
* @param data New byte to write to Slave 4
* @see I2C_SLV4_DO
*/
void MPU9255::setSlave4OutputByte(uint8_t data)
{
	I2Cdev::writeByte(devAddr, I2C_SLV4_DO, data);
}
/** Get the enabled value for the Slave 4.
* When set to 1, this bit enables Slave 4 for data transfer operations. When
* cleared to 0, this bit disables Slave 4 from data transfer operations.
* @return Current enabled value for Slave 4
* @see I2C_SLV4_CTRL
*/
bool MPU9255::getSlave4Enabled()
{
	I2Cdev::readBit(devAddr, I2C_SLV4_CTRL, I2C_SLV4_EN_BIT, buffer);
	return buffer[0];
}
/** Set the enabled value for Slave 4.
* @param enabled New enabled value for Slave 4
* @see getSlave4Enabled()
* @see I2C_SLV4_CTRL
*/
void MPU9255::setSlave4Enabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, I2C_SLV4_CTRL, I2C_SLV4_EN_BIT, enabled);
}
/** Get the enabled value for Slave 4 transaction interrupts.
* When set to 1, this bit enables the generation of an interrupt signal upon
* completion of a Slave 4 transaction. When cleared to 0, this bit disables the
* generation of an interrupt signal upon completion of a Slave 4 transaction.
* The interrupt status can be observed in Register 54.
*
* @return Current enabled value for Slave 4 transaction interrupts.
* @see I2C_SLV4_CTRL
*/
bool MPU9255::getSlave4InterruptEnabled()
{
	I2Cdev::readBit(devAddr, I2C_SLV4_CTRL, I2C_SLV4_INT_EN_BIT, buffer);
	return buffer[0];
}
/** Set the enabled value for Slave 4 transaction interrupts.
* @param enabled New enabled value for Slave 4 transaction interrupts.
* @see getSlave4InterruptEnabled()
* @see I2C_SLV4_CTRL
*/
void MPU9255::setSlave4InterruptEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, I2C_SLV4_CTRL, I2C_SLV4_INT_EN_BIT, enabled);
}
/** Get write mode for Slave 4.
* When set to 1, the transaction will read or write data only. When cleared to
* 0, the transaction will write a register address prior to reading or writing
* data. This should equal 0 when specifying the register address within the
* Slave device to/from which the ensuing data transaction will take place.
*
* @return Current write mode for Slave 4 (0 = register address + data, 1 = data only)
* @see I2C_SLV4_CTRL
*/
bool MPU9255::getSlave4WriteMode()
{
	I2Cdev::readBit(devAddr, I2C_SLV4_CTRL, I2C_SLV4_REG_DIS_BIT, buffer);
	return buffer[0];
}
/** Set write mode for the Slave 4.
* @param mode New write mode for Slave 4 (0 = register address + data, 1 = data only)
* @see getSlave4WriteMode()
* @see I2C_SLV4_CTRL
*/
void MPU9255::setSlave4WriteMode(bool mode)
{
	I2Cdev::writeBit(devAddr, I2C_SLV4_CTRL, I2C_SLV4_REG_DIS_BIT, mode);
}
/** Get Slave 4 master delay value.
* This configures the reduced access rate of I2C slaves relative to the Sample
* Rate. When a slave's access rate is decreased relative to the Sample Rate,
* the slave is accessed every:
*
*     1 / (1 + I2C_MST_DLY) samples
*
* This base Sample Rate in turn is determined by SMPLRT_DIV (register 25) and
* DLPF_CFG (register 26). Whether a slave's access rate is reduced relative to
* the Sample Rate is determined by I2C_MST_DELAY_CTRL (register 103). For
* further information regarding the Sample Rate, please refer to register 25.
*
* @return Current Slave 4 master delay value
* @see I2C_SLV4_CTRL
*/
uint8_t MPU9255::getSlave4MasterDelay()
{
	I2Cdev::readBits(devAddr, I2C_SLV4_CTRL, I2C_SLV4_MST_DLY_BIT, I2C_SLV4_MST_DLY_LENGTH, buffer);
	return buffer[0];
}
/** Set Slave 4 master delay value.
* @param delay New Slave 4 master delay value
* @see getSlave4MasterDelay()
* @see I2C_SLV4_CTRL
*/
void MPU9255::setSlave4MasterDelay(uint8_t delay)
{
	I2Cdev::writeBits(devAddr, I2C_SLV4_CTRL, I2C_SLV4_MST_DLY_BIT, I2C_SLV4_MST_DLY_LENGTH, delay);
}
/** Get last available byte read from Slave 4.
* This register stores the data read from Slave 4. This field is populated
* after a read transaction.
* @return Last available byte read from to Slave 4
* @see I2C_SLV4_DI
*/
uint8_t MPU9255::getSlate4InputByte()
{
	I2Cdev::readByte(devAddr, I2C_SLV4_DI, buffer);
	return buffer[0];
}

// I2C_MST_STATUS register

/** Get FSYNC interrupt status.
* This bit reflects the status of the FSYNC interrupt from an external device
* into the MPU-60X0. This is used as a way to pass an external interrupt
* through the MPU-60X0 to the host application processor. When set to 1, this
* bit will cause an interrupt if FSYNC_INT_EN is asserted in INT_PIN_CFG
* (Register 55).
* @return FSYNC interrupt status
* @see I2C_MST_STATUS
*/
bool MPU9255::getPassthroughStatus()
{
	I2Cdev::readBit(devAddr, I2C_MST_STATUS, MST_PASS_THROUGH_BIT, buffer);
	return buffer[0];
}
/** Get Slave 4 transaction done status.
* Automatically sets to 1 when a Slave 4 transaction has completed. This
* triggers an interrupt if the I2C_MST_INT_EN bit in the INT_ENABLE register
* (Register 56) is asserted and if the SLV_4_DONE_INT bit is asserted in the
* I2C_SLV4_CTRL register (Register 52).
* @return Slave 4 transaction done status
* @see I2C_MST_STATUS
*/
bool MPU9255::getSlave4IsDone()
{
	I2Cdev::readBit(devAddr, I2C_MST_STATUS, MST_I2C_SLV4_DONE_BIT, buffer);
	return buffer[0];
}
/** Get master arbitration lost status.
* This bit automatically sets to 1 when the I2C Master has lost arbitration of
* the auxiliary I2C bus (an error condition). This triggers an interrupt if the
* I2C_MST_INT_EN bit in the INT_ENABLE register (Register 56) is asserted.
* @return Master arbitration lost status
* @see I2C_MST_STATUS
*/
bool MPU9255::getLostArbitration()
{
	I2Cdev::readBit(devAddr, I2C_MST_STATUS, MST_I2C_LOST_ARB_BIT, buffer);
	return buffer[0];
}
/** Get Slave 4 NACK status.
* This bit automatically sets to 1 when the I2C Master receives a NACK in a
* transaction with Slave 4. This triggers an interrupt if the I2C_MST_INT_EN
* bit in the INT_ENABLE register (Register 56) is asserted.
* @return Slave 4 NACK interrupt status
* @see I2C_MST_STATUS
*/
bool MPU9255::getSlave4Nack()
{
	I2Cdev::readBit(devAddr, I2C_MST_STATUS, MST_I2C_SLV4_NACK_BIT, buffer);
	return buffer[0];
}
/** Get Slave 3 NACK status.
* This bit automatically sets to 1 when the I2C Master receives a NACK in a
* transaction with Slave 3. This triggers an interrupt if the I2C_MST_INT_EN
* bit in the INT_ENABLE register (Register 56) is asserted.
* @return Slave 3 NACK interrupt status
* @see I2C_MST_STATUS
*/
bool MPU9255::getSlave3Nack()
{
	I2Cdev::readBit(devAddr, I2C_MST_STATUS, MST_I2C_SLV3_NACK_BIT, buffer);
	return buffer[0];
}
/** Get Slave 2 NACK status.
* This bit automatically sets to 1 when the I2C Master receives a NACK in a
* transaction with Slave 2. This triggers an interrupt if the I2C_MST_INT_EN
* bit in the INT_ENABLE register (Register 56) is asserted.
* @return Slave 2 NACK interrupt status
* @see I2C_MST_STATUS
*/
bool MPU9255::getSlave2Nack()
{
	I2Cdev::readBit(devAddr, I2C_MST_STATUS, MST_I2C_SLV2_NACK_BIT, buffer);
	return buffer[0];
}
/** Get Slave 1 NACK status.
* This bit automatically sets to 1 when the I2C Master receives a NACK in a
* transaction with Slave 1. This triggers an interrupt if the I2C_MST_INT_EN
* bit in the INT_ENABLE register (Register 56) is asserted.
* @return Slave 1 NACK interrupt status
* @see I2C_MST_STATUS
*/
bool MPU9255::getSlave1Nack()
{
	I2Cdev::readBit(devAddr, I2C_MST_STATUS, MST_I2C_SLV1_NACK_BIT, buffer);
	return buffer[0];
}
/** Get Slave 0 NACK status.
* This bit automatically sets to 1 when the I2C Master receives a NACK in a
* transaction with Slave 0. This triggers an interrupt if the I2C_MST_INT_EN
* bit in the INT_ENABLE register (Register 56) is asserted.
* @return Slave 0 NACK interrupt status
* @see I2C_MST_STATUS
*/
bool MPU9255::getSlave0Nack()
{
	I2Cdev::readBit(devAddr, I2C_MST_STATUS, MST_I2C_SLV0_NACK_BIT, buffer);
	return buffer[0];
}

// INT_PIN_CFG register

/** Get interrupt logic level mode.
* Will be set 0 for active-high, 1 for active-low.
* @return Current interrupt mode (0=active-high, 1=active-low)
* @see INT_PIN_CFG
* @see INTCFG_INT_LEVEL_BIT
*/
bool MPU9255::getInterruptMode()
{
	I2Cdev::readBit(devAddr, INT_PIN_CFG, INTCFG_INT_LEVEL_BIT, buffer);
	return buffer[0];
}
/** Set interrupt logic level mode.
* @param mode New interrupt mode (0=active-high, 1=active-low)
* @see getInterruptMode()
* @see INT_PIN_CFG
* @see INTCFG_INT_LEVEL_BIT
*/
void MPU9255::setInterruptMode(bool mode)
{
	I2Cdev::writeBit(devAddr, INT_PIN_CFG, INTCFG_INT_LEVEL_BIT, mode);
}
/** Get interrupt drive mode.
* Will be set 0 for push-pull, 1 for open-drain.
* @return Current interrupt drive mode (0=push-pull, 1=open-drain)
* @see INT_PIN_CFG
* @see INTCFG_INT_OPEN_BIT
*/
bool MPU9255::getInterruptDrive()
{
	I2Cdev::readBit(devAddr, INT_PIN_CFG, INTCFG_INT_OPEN_BIT, buffer);
	return buffer[0];
}
/** Set interrupt drive mode.
* @param drive New interrupt drive mode (0=push-pull, 1=open-drain)
* @see getInterruptDrive()
* @see INT_PIN_CFG
* @see INTCFG_INT_OPEN_BIT
*/
void MPU9255::setInterruptDrive(bool drive)
{
	I2Cdev::writeBit(devAddr, INT_PIN_CFG, INTCFG_INT_OPEN_BIT, drive);
}
/** Get interrupt latch mode.
* Will be set 0 for 50us-pulse, 1 for latch-until-int-cleared.
* @return Current latch mode (0=50us-pulse, 1=latch-until-int-cleared)
* @see INT_PIN_CFG
* @see INTCFG_LATCH_INT_EN_BIT
*/
bool MPU9255::getInterruptLatch()
{
	I2Cdev::readBit(devAddr, INT_PIN_CFG, INTCFG_LATCH_INT_EN_BIT, buffer);
	return buffer[0];
}
/** Set interrupt latch mode.
* @param latch New latch mode (0=50us-pulse, 1=latch-until-int-cleared)
* @see getInterruptLatch()
* @see INT_PIN_CFG
* @see INTCFG_LATCH_INT_EN_BIT
*/
void MPU9255::setInterruptLatch(bool latch)
{
	I2Cdev::writeBit(devAddr, INT_PIN_CFG, INTCFG_LATCH_INT_EN_BIT, latch);
}
/** Get interrupt latch clear mode.
* Will be set 0 for status-read-only, 1 for any-register-read.
* @return Current latch clear mode (0=status-read-only, 1=any-register-read)
* @see INT_PIN_CFG
* @see INTCFG_INT_RD_CLEAR_BIT
*/
bool MPU9255::getInterruptLatchClear()
{
	I2Cdev::readBit(devAddr, INT_PIN_CFG, INTCFG_INT_RD_CLEAR_BIT, buffer);
	return buffer[0];
}
/** Set interrupt latch clear mode.
* @param clear New latch clear mode (0=status-read-only, 1=any-register-read)
* @see getInterruptLatchClear()
* @see INT_PIN_CFG
* @see INTCFG_INT_RD_CLEAR_BIT
*/
void MPU9255::setInterruptLatchClear(bool clear)
{
	I2Cdev::writeBit(devAddr, INT_PIN_CFG, INTCFG_INT_RD_CLEAR_BIT, clear);
}
/** Get FSYNC interrupt logic level mode.
* @return Current FSYNC interrupt mode (0=active-high, 1=active-low)
* @see getFSyncInterruptMode()
* @see INT_PIN_CFG
* @see INTCFG_FSYNC_INT_LEVEL_BIT
*/
bool MPU9255::getFSyncInterruptLevel()
{
	I2Cdev::readBit(devAddr, INT_PIN_CFG, INTCFG_FSYNC_INT_LEVEL_BIT, buffer);
	return buffer[0];
}
/** Set FSYNC interrupt logic level mode.
* @param mode New FSYNC interrupt mode (0=active-high, 1=active-low)
* @see getFSyncInterruptMode()
* @see INT_PIN_CFG
* @see INTCFG_FSYNC_INT_LEVEL_BIT
*/
void MPU9255::setFSyncInterruptLevel(bool level)
{
	I2Cdev::writeBit(devAddr, INT_PIN_CFG, INTCFG_FSYNC_INT_LEVEL_BIT, level);
}
/** Get FSYNC pin interrupt enabled setting.
* Will be set 0 for disabled, 1 for enabled.
* @return Current interrupt enabled setting
* @see INT_PIN_CFG
* @see INTCFG_FSYNC_INT_EN_BIT
*/
bool MPU9255::getFSyncInterruptEnabled()
{
	I2Cdev::readBit(devAddr, INT_PIN_CFG, INTCFG_FSYNC_INT_EN_BIT, buffer);
	return buffer[0];
}
/** Set FSYNC pin interrupt enabled setting.
* @param enabled New FSYNC pin interrupt enabled setting
* @see getFSyncInterruptEnabled()
* @see INT_PIN_CFG
* @see INTCFG_FSYNC_INT_EN_BIT
*/
void MPU9255::setFSyncInterruptEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, INT_PIN_CFG, INTCFG_FSYNC_INT_EN_BIT, enabled);
}
/** Get I2C bypass enabled status.
* When this bit is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to
* 0, the host application processor will be able to directly access the
* auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host
* application processor will not be able to directly access the auxiliary I2C
* bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 106
* bit[5]).
* @return Current I2C bypass enabled status
* @see INT_PIN_CFG
* @see INTCFG_I2C_BYPASS_EN_BIT
*/
bool MPU9255::getI2CBypassEnabled()
{
	I2Cdev::readBit(devAddr, INT_PIN_CFG, INTCFG_I2C_BYPASS_EN_BIT, buffer);
	return buffer[0];
}
/** Set I2C bypass enabled status.
* When this bit is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to
* 0, the host application processor will be able to directly access the
* auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host
* application processor will not be able to directly access the auxiliary I2C
* bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 106
* bit[5]).
* @param enabled New I2C bypass enabled status
* @see INT_PIN_CFG
* @see INTCFG_I2C_BYPASS_EN_BIT
*/
void MPU9255::setI2CBypassEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, INT_PIN_CFG, INTCFG_I2C_BYPASS_EN_BIT, enabled);
}
/** Get reference clock output enabled status.
* When this bit is equal to 1, a reference clock output is provided at the
* CLKOUT pin. When this bit is equal to 0, the clock output is disabled. For
* further information regarding CLKOUT, please refer to the MPU-60X0 Product
* Specification document.
* @return Current reference clock output enabled status
* @see INT_PIN_CFG
* @see INTCFG_CLKOUT_EN_BIT
*/
bool MPU9255::getClockOutputEnabled()
{
	I2Cdev::readBit(devAddr, INT_PIN_CFG, INTCFG_CLKOUT_EN_BIT, buffer);
	return buffer[0];
}
/** Set reference clock output enabled status.
* When this bit is equal to 1, a reference clock output is provided at the
* CLKOUT pin. When this bit is equal to 0, the clock output is disabled. For
* further information regarding CLKOUT, please refer to the MPU-60X0 Product
* Specification document.
* @param enabled New reference clock output enabled status
* @see INT_PIN_CFG
* @see INTCFG_CLKOUT_EN_BIT
*/
void MPU9255::setClockOutputEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, INT_PIN_CFG, INTCFG_CLKOUT_EN_BIT, enabled);
}

// INT_ENABLE register

/** Get Wake on motion interrupt enabled status.
* Will be set 0 for disabled, 1 for enabled.
* @return Current interrupt enabled status
* @see INT_ENABLE
* @see INTERRUPT_WOM_BIT
**/
bool MPU9255::getIntWakeOnMotionEnabled()
{
	I2Cdev::readBit(devAddr, INT_ENABLE, INTERRUPT_WOM_BIT, buffer);
	return buffer[0];
}
/** Set Wake on motion interrupt enabled status.
* @param enabled New interrupt enabled status
* @see getIntMotionEnabled()
* @see INT_ENABLE
* @see INTERRUPT_WOM_BIT
**/
void MPU9255::setIntWakeOnMotionEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, INT_ENABLE, INTERRUPT_WOM_BIT, enabled);
}

/** Get FIFO Buffer Overflow interrupt enabled status.
* Will be set 0 for disabled, 1 for enabled.
* @return Current interrupt enabled status
* @see INT_ENABLE
* @see INTERRUPT_FIFO_OFLOW_BIT
**/
bool MPU9255::getIntFIFOBufferOverflowEnabled()
{
	I2Cdev::readBit(devAddr, INT_ENABLE, INTERRUPT_FIFO_OFLOW_BIT, buffer);
	return buffer[0];
}
/** Set FIFO Buffer Overflow interrupt enabled status.
* @param enabled New interrupt enabled status
* @see getIntFIFOBufferOverflowEnabled()
* @see INT_ENABLE
* @see INTERRUPT_FIFO_OFLOW_BIT
**/
void MPU9255::setIntFIFOBufferOverflowEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, INT_ENABLE, INTERRUPT_FIFO_OFLOW_BIT, enabled);
}
/** Get FSync interrupt enabled status.
* This enables any of the I2C Master interrupt sources to generate an
* interrupt. Will be set 0 for disabled, 1 for enabled.
* @return Current interrupt enabled status
* @see INT_ENABLE
* @see INTERRUPT_I2C_MST_INT_BIT
**/
bool MPU9255::getIntFSyncMasterEnabled()
{
	I2Cdev::readBit(devAddr, INT_ENABLE, INTERRUPT_FSYNC_INT_BIT, buffer);
	return buffer[0];
}
/** Set FSync interrupt enabled status.
* @param enabled New interrupt enabled status
* @see getIntI2CMasterEnabled()
* @see INT_ENABLE
* @see INTERRUPT_I2C_MST_INT_BIT
**/
void MPU9255::setIntFSyncEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, INT_ENABLE, INTERRUPT_FSYNC_INT_BIT, enabled);
}
/** Get Data Ready interrupt enabled setting.
* This event occurs each time a write operation to all of the sensor registers
* has been completed. Will be set 0 for disabled, 1 for enabled.
* @return Current interrupt enabled status
* @see INT_ENABLE
* @see INTERRUPT_DATA_RDY_BIT
*/
bool MPU9255::getIntDataReadyEnabled()
{
	I2Cdev::readBit(devAddr, INT_ENABLE, INTERRUPT_DATA_RDY_BIT, buffer);
	return buffer[0];
}
/** Set Data Ready interrupt enabled status.
* @param enabled New interrupt enabled status
* @see getIntDataReadyEnabled()
* @see INT_CFG
* @see INTERRUPT_DATA_RDY_BIT
*/
void MPU9255::setIntDataReadyEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, INT_ENABLE, INTERRUPT_DATA_RDY_BIT, enabled);
}

// INT_STATUS register

/** Get FIFO Buffer Overflow interrupt status.
* This bit automatically sets to 1 when a Free Fall interrupt has been
* generated. The bit clears to 0 after the register has been read.
* @return Current interrupt status
* @see INT_STATUS
* @see INTERRUPT_FIFO_OFLOW_BIT
*/
bool MPU9255::getIntFIFOBufferOverflowStatus()
{
	I2Cdev::readBit(devAddr, INT_STATUS, INTERRUPT_FIFO_OFLOW_BIT, buffer);
	return buffer[0];
}

/** Get Data Ready interrupt status.
* This bit automatically sets to 1 when a Data Ready interrupt has been
* generated. The bit clears to 0 after the register has been read.
* @return Current interrupt status
* @see INT_STATUS
* @see INTERRUPT_DATA_RDY_BIT
*/
bool MPU9255::getIntDataReadyStatus()
{
	I2Cdev::readBit(devAddr, INT_STATUS, INTERRUPT_DATA_RDY_BIT, buffer);
	return buffer[0];
}

// ACCEL_*OUT_* registers

/** Get raw 9-axis motion sensor readings (accel/gyro/compass).
* FUNCTION NOT FULLY IMPLEMENTED YET.
* @param ax 16-bit signed integer container for accelerometer X-axis value
* @param ay 16-bit signed integer container for accelerometer Y-axis value
* @param az 16-bit signed integer container for accelerometer Z-axis value
* @param gx 16-bit signed integer container for gyroscope X-axis value
* @param gy 16-bit signed integer container for gyroscope Y-axis value
* @param gz 16-bit signed integer container for gyroscope Z-axis value
* @param mx 16-bit signed integer container for magnetometer X-axis value
* @param my 16-bit signed integer container for magnetometer Y-axis value
* @param mz 16-bit signed integer container for magnetometer Z-axis value
* @see getMotion6()
* @see getAcceleration()
* @see getRotation()
* @see ACCEL_XOUT_H
*/
void MPU9255::getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz)
{
	getMotion6(ax, ay, az, gx, gy, gz);
	// TODO: magnetometer integration
}
/** Get raw 6-axis motion sensor readings (accel/gyro).
* Retrieves all currently available motion sensor values.
* @param ax 16-bit signed integer container for accelerometer X-axis value
* @param ay 16-bit signed integer container for accelerometer Y-axis value
* @param az 16-bit signed integer container for accelerometer Z-axis value
* @param gx 16-bit signed integer container for gyroscope X-axis value
* @param gy 16-bit signed integer container for gyroscope Y-axis value
* @param gz 16-bit signed integer container for gyroscope Z-axis value
* @see getAcceleration()
* @see getRotation()
* @see ACCEL_XOUT_H
*/
void MPU9255::getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
	I2Cdev::readBytes(devAddr, ACCEL_XOUT_H, 14, buffer);
	*ax = (((int16_t)buffer[0]) << 8) | buffer[1];
	*ay = (((int16_t)buffer[2]) << 8) | buffer[3];
	*az = (((int16_t)buffer[4]) << 8) | buffer[5];
	*gx = (((int16_t)buffer[8]) << 8) | buffer[9];
	*gy = (((int16_t)buffer[10]) << 8) | buffer[11];
	*gz = (((int16_t)buffer[12]) << 8) | buffer[13];
}
/** Get 3-axis accelerometer readings.
* These registers store the most recent accelerometer measurements.
* Accelerometer measurements are written to these registers at the Sample Rate
* as defined in Register 25.
*
* The accelerometer measurement registers, along with the temperature
* measurement registers, gyroscope measurement registers, and external sensor
* data registers, are composed of two sets of registers: an internal register
* set and a user-facing read register set.
*
* The data within the accelerometer sensors' internal register set is always
* updated at the Sample Rate. Meanwhile, the user-facing read register set
* duplicates the internal register set's data values whenever the serial
* interface is idle. This guarantees that a burst read of sensor registers will
* read measurements from the same sampling instant. Note that if burst reads
* are not used, the user is responsible for ensuring a set of single byte reads
* correspond to a single sampling instant by checking the Data Ready interrupt.
*
* Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS
* (Register 28). For each full scale setting, the accelerometers' sensitivity
* per LSB in ACCEL_xOUT is shown in the table below:
*
* <pre>
* AFS_SEL | Full Scale Range | LSB Sensitivity
* --------+------------------+----------------
* 0       | +/- 2g           | 8192 LSB/mg
* 1       | +/- 4g           | 4096 LSB/mg
* 2       | +/- 8g           | 2048 LSB/mg
* 3       | +/- 16g          | 1024 LSB/mg
* </pre>
*
* @param x 16-bit signed integer container for X-axis acceleration
* @param y 16-bit signed integer container for Y-axis acceleration
* @param z 16-bit signed integer container for Z-axis acceleration
* @see GYRO_XOUT_H
*/
void MPU9255::getAcceleration(int16_t* x, int16_t* y, int16_t* z)
{
	I2Cdev::readBytes(devAddr, ACCEL_XOUT_H, 6, buffer);
	*x = (((int16_t)buffer[0]) << 8) | buffer[1];
	*y = (((int16_t)buffer[2]) << 8) | buffer[3];
	*z = (((int16_t)buffer[4]) << 8) | buffer[5];
}
/** Get X-axis accelerometer reading.
* @return X-axis acceleration measurement in 16-bit 2's complement format
* @see getMotion6()
* @see ACCEL_XOUT_H
*/
int16_t MPU9255::getAccelerationX()
{
	I2Cdev::readBytes(devAddr, ACCEL_XOUT_H, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
/** Get Y-axis accelerometer reading.
* @return Y-axis acceleration measurement in 16-bit 2's complement format
* @see getMotion6()
* @see ACCEL_YOUT_H
*/
int16_t MPU9255::getAccelerationY()
{
	I2Cdev::readBytes(devAddr, ACCEL_YOUT_H, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
/** Get Z-axis accelerometer reading.
* @return Z-axis acceleration measurement in 16-bit 2's complement format
* @see getMotion6()
* @see ACCEL_ZOUT_H
*/
int16_t MPU9255::getAccelerationZ()
{
	I2Cdev::readBytes(devAddr, ACCEL_ZOUT_H, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// TEMP_OUT_* registers

/** Get current internal temperature.
* @return Temperature reading in 16-bit 2's complement format
* @see TEMP_OUT_H
*/
int16_t MPU9255::getTemperature()
{
	I2Cdev::readBytes(devAddr, TEMP_OUT_H, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// GYRO_*OUT_* registers

/** Get 3-axis gyroscope readings.
* These gyroscope measurement registers, along with the accelerometer
* measurement registers, temperature measurement registers, and external sensor
* data registers, are composed of two sets of registers: an internal register
* set and a user-facing read register set.
* The data within the gyroscope sensors' internal register set is always
* updated at the Sample Rate. Meanwhile, the user-facing read register set
* duplicates the internal register set's data values whenever the serial
* interface is idle. This guarantees that a burst read of sensor registers will
* read measurements from the same sampling instant. Note that if burst reads
* are not used, the user is responsible for ensuring a set of single byte reads
* correspond to a single sampling instant by checking the Data Ready interrupt.
*
* Each 16-bit gyroscope measurement has a full scale defined in FS_SEL
* (Register 27). For each full scale setting, the gyroscopes' sensitivity per
* LSB in GYRO_xOUT is shown in the table below:
*
* <pre>
* FS_SEL | Full Scale Range   | LSB Sensitivity
* -------+--------------------+----------------
* 0      | +/- 250 degrees/s  | 131 LSB/deg/s
* 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
* 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
* 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
* </pre>
*
* @param x 16-bit signed integer container for X-axis rotation
* @param y 16-bit signed integer container for Y-axis rotation
* @param z 16-bit signed integer container for Z-axis rotation
* @see getMotion6()
* @see GYRO_XOUT_H
*/
void MPU9255::getRotation(int16_t* x, int16_t* y, int16_t* z)
{
	I2Cdev::readBytes(devAddr, GYRO_XOUT_H, 6, buffer);
	*x = (((int16_t)buffer[0]) << 8) | buffer[1];
	*y = (((int16_t)buffer[2]) << 8) | buffer[3];
	*z = (((int16_t)buffer[4]) << 8) | buffer[5];
}
/** Get X-axis gyroscope reading.
* @return X-axis rotation measurement in 16-bit 2's complement format
* @see getMotion6()
* @see GYRO_XOUT_H
*/
int16_t MPU9255::getRotationX()
{
	I2Cdev::readBytes(devAddr, GYRO_XOUT_H, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
/** Get Y-axis gyroscope reading.
* @return Y-axis rotation measurement in 16-bit 2's complement format
* @see getMotion6()
* @see GYRO_YOUT_H
*/
int16_t MPU9255::getRotationY()
{
	I2Cdev::readBytes(devAddr, GYRO_YOUT_H, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
/** Get Z-axis gyroscope reading.
* @return Z-axis rotation measurement in 16-bit 2's complement format
* @see getMotion6()
* @see GYRO_ZOUT_H
*/
int16_t MPU9255::getRotationZ()
{
	I2Cdev::readBytes(devAddr, GYRO_ZOUT_H, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// EXT_SENS_DATA_* registers

/** Read single byte from external sensor data register.
* These registers store data read from external sensors by the Slave 0, 1, 2,
* and 3 on the auxiliary I2C interface. Data read by Slave 4 is stored in
* I2C_SLV4_DI (Register 53).
*
* External sensor data is written to these registers at the Sample Rate as
* defined in Register 25. This access rate can be reduced by using the Slave
* Delay Enable registers (Register 103).
*
* External sensor data registers, along with the gyroscope measurement
* registers, accelerometer measurement registers, and temperature measurement
* registers, are composed of two sets of registers: an internal register set
* and a user-facing read register set.
*
* The data within the external sensors' internal register set is always updated
* at the Sample Rate (or the reduced access rate) whenever the serial interface
* is idle. This guarantees that a burst read of sensor registers will read
* measurements from the same sampling instant. Note that if burst reads are not
* used, the user is responsible for ensuring a set of single byte reads
* correspond to a single sampling instant by checking the Data Ready interrupt.
*
* Data is placed in these external sensor data registers according to
* I2C_SLV0_CTRL, I2C_SLV1_CTRL, I2C_SLV2_CTRL, and I2C_SLV3_CTRL (Registers 39,
* 42, 45, and 48). When more than zero bytes are read (I2C_SLVx_LEN > 0) from
* an enabled slave (I2C_SLVx_EN = 1), the slave is read at the Sample Rate (as
* defined in Register 25) or delayed rate (if specified in Register 52 and
* 103). During each Sample cycle, slave reads are performed in order of Slave
* number. If all slaves are enabled with more than zero bytes to be read, the
* order will be Slave 0, followed by Slave 1, Slave 2, and Slave 3.
*
* Each enabled slave will have EXT_SENS_DATA registers associated with it by
* number of bytes read (I2C_SLVx_LEN) in order of slave number, starting from
* EXT_SENS_DATA_00. Note that this means enabling or disabling a slave may
* change the higher numbered slaves' associated registers. Furthermore, if
* fewer total bytes are being read from the external sensors as a result of
* such a change, then the data remaining in the registers which no longer have
* an associated slave device (i.e. high numbered registers) will remain in
* these previously allocated registers unless reset.
*
* If the sum of the read lengths of all SLVx transactions exceed the number of
* available EXT_SENS_DATA registers, the excess bytes will be dropped. There
* are 24 EXT_SENS_DATA registers and hence the total read lengths between all
* the slaves cannot be greater than 24 or some bytes will be lost.
*
* Note: Slave 4's behavior is distinct from that of Slaves 0-3. For further
* information regarding the characteristics of Slave 4, please refer to
* Registers 49 to 53.
*
* EXAMPLE:
* Suppose that Slave 0 is enabled with 4 bytes to be read (I2C_SLV0_EN = 1 and
* I2C_SLV0_LEN = 4) while Slave 1 is enabled with 2 bytes to be read so that
* I2C_SLV1_EN = 1 and I2C_SLV1_LEN = 2. In such a situation, EXT_SENS_DATA _00
* through _03 will be associated with Slave 0, while EXT_SENS_DATA _04 and 05
* will be associated with Slave 1. If Slave 2 is enabled as well, registers
* starting from EXT_SENS_DATA_06 will be allocated to Slave 2.
*
* If Slave 2 is disabled while Slave 3 is enabled in this same situation, then
* registers starting from EXT_SENS_DATA_06 will be allocated to Slave 3
* instead.
*
* REGISTER ALLOCATION FOR DYNAMIC DISABLE VS. NORMAL DISABLE:
* If a slave is disabled at any time, the space initially allocated to the
* slave in the EXT_SENS_DATA register, will remain associated with that slave.
* This is to avoid dynamic adjustment of the register allocation.
*
* The allocation of the EXT_SENS_DATA registers is recomputed only when (1) all
* slaves are disabled, or (2) the I2C_MST_RST bit is set (Register 106).
*
* This above is also true if one of the slaves gets NACKed and stops
* functioning.
*
* @param position Starting position (0-23)
* @return Byte read from register
*/
uint8_t MPU9255::getExternalSensorByte(int position)
{
	I2Cdev::readByte(devAddr, EXT_SENS_DATA_00 + position, buffer);
	return buffer[0];
}
/** Read word (2 bytes) from external sensor data registers.
* @param position Starting position (0-21)
* @return Word read from register
* @see getExternalSensorByte()
*/
uint16_t MPU9255::getExternalSensorWord(int position)
{
	I2Cdev::readBytes(devAddr, EXT_SENS_DATA_00 + position, 2, buffer);
	return (((uint16_t)buffer[0]) << 8) | buffer[1];
}
/** Read double word (4 bytes) from external sensor data registers.
* @param position Starting position (0-20)
* @return Double word read from registers
* @see getExternalSensorByte()
*/
uint32_t MPU9255::getExternalSensorDWord(int position)
{
	I2Cdev::readBytes(devAddr, EXT_SENS_DATA_00 + position, 4, buffer);
	return (((uint32_t)buffer[0]) << 24) | (((uint32_t)buffer[1]) << 16) | (((uint16_t)buffer[2]) << 8) | buffer[3];
}

// MOT_DETECT_STATUS register

/** Get X-axis negative motion detection interrupt status.
* @return Motion detection status
* @see MOT_DETECT_STATUS
* @see MOTION_MOT_XNEG_BIT
*/
bool MPU9255::getXNegMotionDetected()
{
	I2Cdev::readBit(devAddr, MOT_DETECT_STATUS, MOTION_MOT_XNEG_BIT, buffer);
	return buffer[0];
}
/** Get X-axis positive motion detection interrupt status.
* @return Motion detection status
* @see MOT_DETECT_STATUS
* @see MOTION_MOT_XPOS_BIT
*/
bool MPU9255::getXPosMotionDetected()
{
	I2Cdev::readBit(devAddr, MOT_DETECT_STATUS, MOTION_MOT_XPOS_BIT, buffer);
	return buffer[0];
}
/** Get Y-axis negative motion detection interrupt status.
* @return Motion detection status
* @see MOT_DETECT_STATUS
* @see MOTION_MOT_YNEG_BIT
*/
bool MPU9255::getYNegMotionDetected()
{
	I2Cdev::readBit(devAddr, MOT_DETECT_STATUS, MOTION_MOT_YNEG_BIT, buffer);
	return buffer[0];
}
/** Get Y-axis positive motion detection interrupt status.
* @return Motion detection status
* @see MOT_DETECT_STATUS
* @see MOTION_MOT_YPOS_BIT
*/
bool MPU9255::getYPosMotionDetected()
{
	I2Cdev::readBit(devAddr, MOT_DETECT_STATUS, MOTION_MOT_YPOS_BIT, buffer);
	return buffer[0];
}
/** Get Z-axis negative motion detection interrupt status.
* @return Motion detection status
* @see MOT_DETECT_STATUS
* @see MOTION_MOT_ZNEG_BIT
*/
bool MPU9255::getZNegMotionDetected()
{
	I2Cdev::readBit(devAddr, MOT_DETECT_STATUS, MOTION_MOT_ZNEG_BIT, buffer);
	return buffer[0];
}
/** Get Z-axis positive motion detection interrupt status.
* @return Motion detection status
* @see MOT_DETECT_STATUS
* @see MOTION_MOT_ZPOS_BIT
*/
bool MPU9255::getZPosMotionDetected()
{
	I2Cdev::readBit(devAddr, MOT_DETECT_STATUS, MOTION_MOT_ZPOS_BIT, buffer);
	return buffer[0];
}
/** Get zero motion detection interrupt status.
* @return Motion detection status
* @see MOT_DETECT_STATUS
* @see MOTION_MOT_ZRMOT_BIT
*/
bool MPU9255::getZeroMotionDetected()
{
	I2Cdev::readBit(devAddr, MOT_DETECT_STATUS, MOTION_MOT_ZRMOT_BIT, buffer);
	return buffer[0];
}

// I2C_SLV*_DO register

/** Write byte to Data Output container for specified slave.
* This register holds the output data written into Slave when Slave is set to
* write mode. For further information regarding Slave control, please
* refer to Registers 37 to 39 and immediately following.
* @param num Slave number (0-3)
* @param data Byte to write
* @see I2C_SLV0_DO
*/
void MPU9255::setSlaveOutputByte(uint8_t num, uint8_t data)
{
	if (num > 3) return;
	I2Cdev::writeByte(devAddr, I2C_SLV0_DO + num, data);
}

// I2C_MST_DELAY_CTRL register

/** Get external data shadow delay enabled status.
* This register is used to specify the timing of external sensor data
* shadowing. When DELAY_ES_SHADOW is set to 1, shadowing of external
* sensor data is delayed until all data has been received.
* @return Current external data shadow delay enabled status.
* @see I2C_MST_DELAY_CTRL
* @see DELAYCTRL_DELAY_ES_SHADOW_BIT
*/
bool MPU9255::getExternalShadowDelayEnabled()
{
	I2Cdev::readBit(devAddr, I2C_MST_DELAY_CTRL, DELAYCTRL_DELAY_ES_SHADOW_BIT, buffer);
	return buffer[0];
}
/** Set external data shadow delay enabled status.
* @param enabled New external data shadow delay enabled status.
* @see getExternalShadowDelayEnabled()
* @see I2C_MST_DELAY_CTRL
* @see DELAYCTRL_DELAY_ES_SHADOW_BIT
*/
void MPU9255::setExternalShadowDelayEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, I2C_MST_DELAY_CTRL, DELAYCTRL_DELAY_ES_SHADOW_BIT, enabled);
}
/** Get slave delay enabled status.
* When a particular slave delay is enabled, the rate of access for the that
* slave device is reduced. When a slave's access rate is decreased relative to
* the Sample Rate, the slave is accessed every:
*
*     1 / (1 + I2C_MST_DLY) Samples
*
* This base Sample Rate in turn is determined by SMPLRT_DIV (register  * 25)
* and DLPF_CFG (register 26).
*
* For further information regarding I2C_MST_DLY, please refer to register 52.
* For further information regarding the Sample Rate, please refer to register 25.
*
* @param num Slave number (0-4)
* @return Current slave delay enabled status.
* @see I2C_MST_DELAY_CTRL
* @see DELAYCTRL_I2C_SLV0_DLY_EN_BIT
*/
bool MPU9255::getSlaveDelayEnabled(uint8_t num)
{
	// DELAYCTRL_I2C_SLV4_DLY_EN_BIT is 4, SLV3 is 3, etc.
	if (num > 4) return 0;
	I2Cdev::readBit(devAddr, I2C_MST_DELAY_CTRL, num, buffer);
	return buffer[0];
}
/** Set slave delay enabled status.
* @param num Slave number (0-4)
* @param enabled New slave delay enabled status.
* @see I2C_MST_DELAY_CTRL
* @see DELAYCTRL_I2C_SLV0_DLY_EN_BIT
*/
void MPU9255::setSlaveDelayEnabled(uint8_t num, bool enabled)
{
	I2Cdev::writeBit(devAddr, I2C_MST_DELAY_CTRL, num, enabled);
}

// SIGNAL_PATH_RESET register

/** Reset gyroscope signal path.
* The reset will revert the signal path analog to digital converters and
* filters to their power up configurations.
* @see SIGNAL_PATH_RESET
* @see PATHRESET_GYRO_RESET_BIT
*/
void MPU9255::resetGyroscopePath()
{
	I2Cdev::writeBit(devAddr, SIGNAL_PATH_RESET, PATHRESET_GYRO_RESET_BIT, true);
}
/** Reset accelerometer signal path.
* The reset will revert the signal path analog to digital converters and
* filters to their power up configurations.
* @see SIGNAL_PATH_RESET
* @see PATHRESET_ACCEL_RESET_BIT
*/
void MPU9255::resetAccelerometerPath()
{
	I2Cdev::writeBit(devAddr, SIGNAL_PATH_RESET, PATHRESET_ACCEL_RESET_BIT, true);
}
/** Reset temperature sensor signal path.
* The reset will revert the signal path analog to digital converters and
* filters to their power up configurations.
* @see SIGNAL_PATH_RESET
* @see PATHRESET_TEMP_RESET_BIT
*/
void MPU9255::resetTemperaturePath()
{
	I2Cdev::writeBit(devAddr, SIGNAL_PATH_RESET, PATHRESET_TEMP_RESET_BIT, true);
}

// MOT_DETECT_CTRL register

/** Get accelerometer power-on delay.
* The accelerometer data path provides samples to the sensor registers, Motion
* detection, Zero Motion detection, and Free Fall detection modules. The
* signal path contains filters which must be flushed on wake-up with new
* samples before the detection modules begin operations. The default wake-up
* delay, of 4ms can be lengthened by up to 3ms. This additional delay is
* specified in ACCEL_ON_DELAY in units of 1 LSB = 1 ms. The user may select
* any value above zero unless instructed otherwise by InvenSense. Please refer
* to Section 8 of the MPU-6000/MPU-6050 Product Specification document for
* further information regarding the detection modules.
* @return Current accelerometer power-on delay
* @see MOT_DETECT_CTRL
* @see DETECT_ACCEL_ON_DELAY_BIT
*/
uint8_t MPU9255::getAccelerometerPowerOnDelay()
{
	I2Cdev::readBits(devAddr, MOT_DETECT_CTRL, DETECT_ACCEL_ON_DELAY_BIT, DETECT_ACCEL_ON_DELAY_LENGTH, buffer);
	return buffer[0];
}
/** Set accelerometer power-on delay.
* @param delay New accelerometer power-on delay (0-3)
* @see getAccelerometerPowerOnDelay()
* @see MOT_DETECT_CTRL
* @see DETECT_ACCEL_ON_DELAY_BIT
*/
void MPU9255::setAccelerometerPowerOnDelay(uint8_t delay)
{
	I2Cdev::writeBits(devAddr, MOT_DETECT_CTRL, DETECT_ACCEL_ON_DELAY_BIT, DETECT_ACCEL_ON_DELAY_LENGTH, delay);
}
/** Get Free Fall detection counter decrement configuration.
* Detection is registered by the Free Fall detection module after accelerometer
* measurements meet their respective threshold conditions over a specified
* number of samples. When the threshold conditions are met, the corresponding
* detection counter increments by 1. The user may control the rate at which the
* detection counter decrements when the threshold condition is not met by
* configuring FF_COUNT. The decrement rate can be set according to the
* following table:
*
* <pre>
* FF_COUNT | Counter Decrement
* ---------+------------------
* 0        | Reset
* 1        | 1
* 2        | 2
* 3        | 4
* </pre>
*
* When FF_COUNT is configured to 0 (reset), any non-qualifying sample will
* reset the counter to 0. For further information on Free Fall detection,
* please refer to Registers 29 to 32.
*
* @return Current decrement configuration
* @see MOT_DETECT_CTRL
* @see DETECT_FF_COUNT_BIT
*/
uint8_t MPU9255::getFreefallDetectionCounterDecrement()
{
	I2Cdev::readBits(devAddr, MOT_DETECT_CTRL, DETECT_FF_COUNT_BIT, DETECT_FF_COUNT_LENGTH, buffer);
	return buffer[0];
}
/** Set Free Fall detection counter decrement configuration.
* @param decrement New decrement configuration value
* @see getFreefallDetectionCounterDecrement()
* @see MOT_DETECT_CTRL
* @see DETECT_FF_COUNT_BIT
*/
void MPU9255::setFreefallDetectionCounterDecrement(uint8_t decrement)
{
	I2Cdev::writeBits(devAddr, MOT_DETECT_CTRL, DETECT_FF_COUNT_BIT, DETECT_FF_COUNT_LENGTH, decrement);
}
/** Get Motion detection counter decrement configuration.
* Detection is registered by the Motion detection module after accelerometer
* measurements meet their respective threshold conditions over a specified
* number of samples. When the threshold conditions are met, the corresponding
* detection counter increments by 1. The user may control the rate at which the
* detection counter decrements when the threshold condition is not met by
* configuring MOT_COUNT. The decrement rate can be set according to the
* following table:
*
* <pre>
* MOT_COUNT | Counter Decrement
* ----------+------------------
* 0         | Reset
* 1         | 1
* 2         | 2
* 3         | 4
* </pre>
*
* When MOT_COUNT is configured to 0 (reset), any non-qualifying sample will
* reset the counter to 0. For further information on Motion detection,
* please refer to Registers 29 to 32.
*
*/
uint8_t MPU9255::getMotionDetectionCounterDecrement()
{
	I2Cdev::readBits(devAddr, MOT_DETECT_CTRL, DETECT_MOT_COUNT_BIT, DETECT_MOT_COUNT_LENGTH, buffer);
	return buffer[0];
}
/** Set Motion detection counter decrement configuration.
* @param decrement New decrement configuration value
* @see getMotionDetectionCounterDecrement()
* @see MOT_DETECT_CTRL
* @see DETECT_MOT_COUNT_BIT
*/
void MPU9255::setMotionDetectionCounterDecrement(uint8_t decrement)
{
	I2Cdev::writeBits(devAddr, MOT_DETECT_CTRL, DETECT_MOT_COUNT_BIT, DETECT_MOT_COUNT_LENGTH, decrement);
}

// USER_CTRL register

/** Get FIFO enabled status.
* When this bit is set to 0, the FIFO buffer is disabled. The FIFO buffer
* cannot be written to or read from while disabled. The FIFO buffer's state
* does not change unless the MPU-60X0 is power cycled.
* @return Current FIFO enabled status
* @see USER_CTRL
* @see USERCTRL_FIFO_EN_BIT
*/
bool MPU9255::getFIFOEnabled()
{
	I2Cdev::readBit(devAddr, USER_CTRL, USERCTRL_FIFO_EN_BIT, buffer);
	return buffer[0];
}
/** Set FIFO enabled status.
* @param enabled New FIFO enabled status
* @see getFIFOEnabled()
* @see USER_CTRL
* @see USERCTRL_FIFO_EN_BIT
*/
void MPU9255::setFIFOEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, USER_CTRL, USERCTRL_FIFO_EN_BIT, enabled);
}
/** Get I2C Master Mode enabled status.
* When this mode is enabled, the MPU-60X0 acts as the I2C Master to the
* external sensor slave devices on the auxiliary I2C bus. When this bit is
* cleared to 0, the auxiliary I2C bus lines (AUX_DA and AUX_CL) are logically
* driven by the primary I2C bus (SDA and SCL). This is a precondition to
* enabling Bypass Mode. For further information regarding Bypass Mode, please
* refer to Register 55.
* @return Current I2C Master Mode enabled status
* @see USER_CTRL
* @see USERCTRL_I2C_MST_EN_BIT
*/
bool MPU9255::getI2CMasterModeEnabled()
{
	I2Cdev::readBit(devAddr, USER_CTRL, USERCTRL_I2C_MST_EN_BIT, buffer);
	return buffer[0];
}
/** Set I2C Master Mode enabled status.
* @param enabled New I2C Master Mode enabled status
* @see getI2CMasterModeEnabled()
* @see USER_CTRL
* @see USERCTRL_I2C_MST_EN_BIT
*/
void MPU9255::setI2CMasterModeEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, USER_CTRL, USERCTRL_I2C_MST_EN_BIT, enabled);
}
/** Switch from I2C to SPI mode (MPU-6000 only)
* If this is set, the primary SPI interface will be enabled in place of the
* disabled primary I2C interface.
*/
void MPU9255::switchSPIEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, USER_CTRL, USERCTRL_I2C_IF_DIS_BIT, enabled);
}
/** Reset the FIFO.
* This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This
* bit automatically clears to 0 after the reset has been triggered.
* @see USER_CTRL
* @see USERCTRL_FIFO_RESET_BIT
*/
void MPU9255::resetFIFO()
{
	I2Cdev::writeBit(devAddr, USER_CTRL, USERCTRL_FIFO_RESET_BIT, true);
}
/** Reset the I2C Master.
* This bit resets the I2C Master when set to 1 while I2C_MST_EN equals 0.
* This bit automatically clears to 0 after the reset has been triggered.
* @see USER_CTRL
* @see USERCTRL_I2C_MST_RESET_BIT
*/
void MPU9255::resetI2CMaster()
{
	I2Cdev::writeBit(devAddr, USER_CTRL, USERCTRL_I2C_MST_RESET_BIT, true);
}
/** Reset all sensor registers and signal paths.
* When set to 1, this bit resets the signal paths for all sensors (gyroscopes,
* accelerometers, and temperature sensor). This operation will also clear the
* sensor registers. This bit automatically clears to 0 after the reset has been
* triggered.
*
* When resetting only the signal path (and not the sensor registers), please
* use Register 104, SIGNAL_PATH_RESET.
*
* @see USER_CTRL
* @see USERCTRL_SIG_COND_RESET_BIT
*/
void MPU9255::resetSensors()
{
	I2Cdev::writeBit(devAddr, USER_CTRL, USERCTRL_SIG_COND_RESET_BIT, true);
}

// PWR_MGMT_1 register

/** Trigger a full device reset.
* A small delay of ~50ms may be desirable after triggering a reset.
* @see PWR_MGMT_1
* @see PWR1_DEVICE_RESET_BIT
*/
void MPU9255::reset()
{
	I2Cdev::writeBit(devAddr, PWR_MGMT_1, PWR1_DEVICE_RESET_BIT, true);
}
/** Get sleep mode status.
* Setting the SLEEP bit in the register puts the device into very low power
* sleep mode. In this mode, only the serial interface and internal registers
* remain active, allowing for a very low standby current. Clearing this bit
* puts the device back into normal mode. To save power, the individual standby
* selections for each of the gyros should be used if any gyro axis is not used
* by the application.
* @return Current sleep mode enabled status
* @see PWR_MGMT_1
* @see PWR1_SLEEP_BIT
*/
bool MPU9255::getSleepEnabled()
{
	I2Cdev::readBit(devAddr, PWR_MGMT_1, PWR1_SLEEP_BIT, buffer);
	return buffer[0];
}
/** Set sleep mode status.
* @param enabled New sleep mode enabled status
* @see getSleepEnabled()
* @see PWR_MGMT_1
* @see PWR1_SLEEP_BIT
*/
void MPU9255::setSleepEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, PWR_MGMT_1, PWR1_SLEEP_BIT, enabled);
}
/** Get wake cycle enabled status.
* When this bit is set to 1 and SLEEP is disabled, the MPU-60X0 will cycle
* between sleep mode and waking up to take a single sample of data from active
* sensors at a rate determined by LP_WAKE_CTRL (register 108).
* @return Current sleep mode enabled status
* @see PWR_MGMT_1
* @see PWR1_CYCLE_BIT
*/
bool MPU9255::getWakeCycleEnabled()
{
	I2Cdev::readBit(devAddr, PWR_MGMT_1, PWR1_CYCLE_BIT, buffer);
	return buffer[0];
}
/** Set wake cycle enabled status.
* @param enabled New sleep mode enabled status
* @see getWakeCycleEnabled()
* @see PWR_MGMT_1
* @see PWR1_CYCLE_BIT
*/
void MPU9255::setWakeCycleEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, PWR_MGMT_1, PWR1_CYCLE_BIT, enabled);
}
/** Get temperature sensor enabled status.
* Control the usage of the internal temperature sensor.
*
* Note: this register stores the *disabled* value, but for consistency with the
* rest of the code, the function is named and used with standard true/false
* values to indicate whether the sensor is enabled or disabled, respectively.
*
* @return Current temperature sensor enabled status
* @see PWR_MGMT_1
* @see PWR1_TEMP_DIS_BIT
*/
bool MPU9255::getTempSensorEnabled()
{
	//TODO I2Cdev::readBit(devAddr, PWR_MGMT_1, PWR1_TEMP_DIS_BIT, buffer);
	return buffer[0] == 0; // 1 is actually disabled here
}
/** Set temperature sensor enabled status.
* Note: this register stores the *disabled* value, but for consistency with the
* rest of the code, the function is named and used with standard true/false
* values to indicate whether the sensor is enabled or disabled, respectively.
*
* @param enabled New temperature sensor enabled status
* @see getTempSensorEnabled()
* @see PWR_MGMT_1
* @see PWR1_TEMP_DIS_BIT
*/
void MPU9255::setTempSensorEnabled(bool enabled)
{
	// 1 is actually disabled here
	//TODO I2Cdev::writeBit(devAddr, PWR_MGMT_1, PWR1_TEMP_DIS_BIT, !enabled);
}
/** Get clock source setting.
* @return Current clock source setting
* @see PWR_MGMT_1
* @see PWR1_CLKSEL_BIT
* @see PWR1_CLKSEL_LENGTH
*/
uint8_t MPU9255::getClockSource()
{
	I2Cdev::readBits(devAddr, PWR_MGMT_1, PWR1_CLKSEL_BIT, PWR1_CLKSEL_LENGTH, buffer);
	return buffer[0];
}

/** Set clock source setting.
* An internal 8MHz oscillator, gyroscope based clock, or external sources can
* be selected as the MPU-9255 clock source. When the internal 8 MHz oscillator
* or an external source is chosen as the clock source, the MPU-60X0 can operate
* in low power modes with the gyroscopes disabled.
*
* Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
* However, it is highly recommended that the device be configured to use one of
* the gyroscopes (or an external clock source) as the clock reference for
* improved stability. The clock source can be selected according to the following table:
*
* <pre>
* CLK_SEL | Clock Source
* --------+--------------------------------------
* 0       | Internal oscillator
* 1       | PLL with X Gyro reference
* 2       | PLL with Y Gyro reference
* 3       | PLL with Z Gyro reference
* 4       | PLL with external 32.768kHz reference
* 5       | PLL with external 19.2MHz reference
* 6       | Reserved
* 7       | Stops the clock and keeps the timing generator in reset
* </pre>
*
* @param source New clock source setting
* @see getClockSource()
* @see PWR_MGMT_1
* @see PWR1_CLKSEL_BIT
* @see PWR1_CLKSEL_LENGTH
*/
void MPU9255::setClockSource(uint8_t source)
{
	I2Cdev::writeBits(devAddr, PWR_MGMT_1, PWR1_CLKSEL_BIT, PWR1_CLKSEL_LENGTH, source);
}

// PWR_MGMT_2 register

/** Get X-axis accelerometer standby enabled status.
* If enabled, the X-axis will not gather or report data (or use power).
* @return Current X-axis standby enabled status
* @see PWR_MGMT_2
* @see PWR2_STBY_XA_BIT
*/
bool MPU9255::getStandbyXAccelEnabled()
{
	I2Cdev::readBit(devAddr, PWR_MGMT_2, PWR2_STBY_XA_BIT, buffer);
	return buffer[0];
}
/** Set X-axis accelerometer standby enabled status.
* @param New X-axis standby enabled status
* @see getStandbyXAccelEnabled()
* @see PWR_MGMT_2
* @see PWR2_STBY_XA_BIT
*/
void MPU9255::setStandbyXAccelEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, PWR_MGMT_2, PWR2_STBY_XA_BIT, enabled);
}
/** Get Y-axis accelerometer standby enabled status.
* If enabled, the Y-axis will not gather or report data (or use power).
* @return Current Y-axis standby enabled status
* @see PWR_MGMT_2
* @see PWR2_STBY_YA_BIT
*/
bool MPU9255::getStandbyYAccelEnabled()
{
	I2Cdev::readBit(devAddr, PWR_MGMT_2, PWR2_STBY_YA_BIT, buffer);
	return buffer[0];
}
/** Set Y-axis accelerometer standby enabled status.
* @param New Y-axis standby enabled status
* @see getStandbyYAccelEnabled()
* @see PWR_MGMT_2
* @see PWR2_STBY_YA_BIT
*/
void MPU9255::setStandbyYAccelEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, PWR_MGMT_2, PWR2_STBY_YA_BIT, enabled);
}
/** Get Z-axis accelerometer standby enabled status.
* If enabled, the Z-axis will not gather or report data (or use power).
* @return Current Z-axis standby enabled status
* @see PWR_MGMT_2
* @see PWR2_STBY_ZA_BIT
*/
bool MPU9255::getStandbyZAccelEnabled()
{
	I2Cdev::readBit(devAddr, PWR_MGMT_2, PWR2_STBY_ZA_BIT, buffer);
	return buffer[0];
}
/** Set Z-axis accelerometer standby enabled status.
* @param New Z-axis standby enabled status
* @see getStandbyZAccelEnabled()
* @see PWR_MGMT_2
* @see PWR2_STBY_ZA_BIT
*/
void MPU9255::setStandbyZAccelEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, PWR_MGMT_2, PWR2_STBY_ZA_BIT, enabled);
}
/** Get X-axis gyroscope standby enabled status.
* If enabled, the X-axis will not gather or report data (or use power).
* @return Current X-axis standby enabled status
* @see PWR_MGMT_2
* @see PWR2_STBY_XG_BIT
*/
bool MPU9255::getStandbyXGyroEnabled()
{
	I2Cdev::readBit(devAddr, PWR_MGMT_2, PWR2_STBY_XG_BIT, buffer);
	return buffer[0];
}
/** Set X-axis gyroscope standby enabled status.
* @param New X-axis standby enabled status
* @see getStandbyXGyroEnabled()
* @see PWR_MGMT_2
* @see PWR2_STBY_XG_BIT
*/
void MPU9255::setStandbyXGyroEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, PWR_MGMT_2, PWR2_STBY_XG_BIT, enabled);
}
/** Get Y-axis gyroscope standby enabled status.
* If enabled, the Y-axis will not gather or report data (or use power).
* @return Current Y-axis standby enabled status
* @see PWR_MGMT_2
* @see PWR2_STBY_YG_BIT
*/
bool MPU9255::getStandbyYGyroEnabled()
{
	I2Cdev::readBit(devAddr, PWR_MGMT_2, PWR2_STBY_YG_BIT, buffer);
	return buffer[0];
}
/** Set Y-axis gyroscope standby enabled status.
* @param New Y-axis standby enabled status
* @see getStandbyYGyroEnabled()
* @see PWR_MGMT_2
* @see PWR2_STBY_YG_BIT
*/
void MPU9255::setStandbyYGyroEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, PWR_MGMT_2, PWR2_STBY_YG_BIT, enabled);
}
/** Get Z-axis gyroscope standby enabled status.
* If enabled, the Z-axis will not gather or report data (or use power).
* @return Current Z-axis standby enabled status
* @see PWR_MGMT_2
* @see PWR2_STBY_ZG_BIT
*/
bool MPU9255::getStandbyZGyroEnabled()
{
	I2Cdev::readBit(devAddr, PWR_MGMT_2, PWR2_STBY_ZG_BIT, buffer);
	return buffer[0];
}
/** Set Z-axis gyroscope standby enabled status.
* @param New Z-axis standby enabled status
* @see getStandbyZGyroEnabled()
* @see PWR_MGMT_2
* @see PWR2_STBY_ZG_BIT
*/
void MPU9255::setStandbyZGyroEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, PWR_MGMT_2, PWR2_STBY_ZG_BIT, enabled);
}

// FIFO_COUNT* registers

/** Get current FIFO buffer size.
* This value indicates the number of bytes stored in the FIFO buffer. This
* number is in turn the number of bytes that can be read from the FIFO buffer
* and it is directly proportional to the number of samples available given the
* set of sensor data bound to be stored in the FIFO (register 35 and 36).
* @return Current FIFO buffer size
*/
uint16_t MPU9255::getFIFOCount()
{
	I2Cdev::readBytes(devAddr, FIFO_COUNTH, 2, buffer);
	return (((uint16_t)buffer[0]) << 8) | buffer[1];
}

// FIFO_R_W register

/** Get byte from FIFO buffer.
* This register is used to read and write data from the FIFO buffer. Data is
* written to the FIFO in order of register number (from lowest to highest). If
* all the FIFO enable flags (see below) are enabled and all External Sensor
* Data registers (Registers 73 to 96) are associated with a Slave device, the
* contents of registers 59 through 96 will be written in order at the Sample
* Rate.
*
* The contents of the sensor data registers (Registers 59 to 96) are written
* into the FIFO buffer when their corresponding FIFO enable flags are set to 1
* in FIFO_EN (Register 35). An additional flag for the sensor data registers
* associated with I2C Slave 3 can be found in I2C_MST_CTRL (Register 36).
*
* If the FIFO buffer has overflowed, the status bit FIFO_OFLOW_INT is
* automatically set to 1. This bit is located in INT_STATUS (Register 58).
* When the FIFO buffer has overflowed, the oldest data will be lost and new
* data will be written to the FIFO.
*
* If the FIFO buffer is empty, reading this register will return the last byte
* that was previously read from the FIFO until new data is available. The user
* should check FIFO_COUNT to ensure that the FIFO buffer is not read when
* empty.
*
* @return Byte from FIFO buffer
*/
uint8_t MPU9255::getFIFOByte()
{
	I2Cdev::readByte(devAddr, FIFO_R_W, buffer);
	return buffer[0];
}
/** Write byte to FIFO buffer.
* @see getFIFOByte()
* @see FIFO_R_W
*/
void MPU9255::setFIFOByte(uint8_t data)
{
	I2Cdev::writeByte(devAddr, FIFO_R_W, data);
}

// WHO_AM_I register

/** Get Device ID.
* This register is used to verify the identity of the device (0b110100).
* @return Device ID (should be 0x68, 104 dec, 150 oct)
* @see WHO_AM_I
* @see WHO_AM_I_BIT
* @see WHO_AM_I_LENGTH
*/
uint8_t MPU9255::getDeviceID()
{
	I2Cdev::readBits(devAddr, WHO_AM_I_MPU9255, WHO_AM_I_BIT, WHO_AM_I_LENGTH, buffer);
	return buffer[0];
}
/** Set Device ID.
* Write a new ID into the WHO_AM_I register (no idea why this should ever be
* necessary though).
* @param id New device ID to set.
* @see getDeviceID()
* @see WHO_AM_I
* @see WHO_AM_I_BIT
* @see WHO_AM_I_LENGTH
*/
void MPU9255::setDeviceID(uint8_t id)
{
	I2Cdev::writeBits(devAddr, WHO_AM_I_MPU9255, WHO_AM_I_BIT, WHO_AM_I_LENGTH, id);
}

// XA_OFFS_* registers

int16_t MPU9255::getXAccelOffset()
{
	I2Cdev::readBytes(devAddr, XA_OFFSET_H, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU9255::setXAccelOffset(int16_t offset)
{
	I2Cdev::writeWord(devAddr, XA_OFFSET_H, offset);
}

// YA_OFFS_* register

int16_t MPU9255::getYAccelOffset()
{
	I2Cdev::readBytes(devAddr, YA_OFFSET_H, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU9255::setYAccelOffset(int16_t offset)
{
	I2Cdev::writeWord(devAddr, YA_OFFSET_H, offset);
}

// ZA_OFFS_* register

int16_t MPU9255::getZAccelOffset()
{
	I2Cdev::readBytes(devAddr, ZA_OFFSET_H, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU9255::setZAccelOffset(int16_t offset)
{
	I2Cdev::writeWord(devAddr, ZA_OFFSET_H, offset);
}

// INT_ENABLE register (DMP functions)

// DMP_INT_STATUS

bool MPU9255::getDMPInt5Status()
{
	I2Cdev::readBit(devAddr, DMP_INT_STATUS, DMPINT_5_BIT, buffer);
	return buffer[0];
}
bool MPU9255::getDMPInt4Status()
{
	I2Cdev::readBit(devAddr, DMP_INT_STATUS, DMPINT_4_BIT, buffer);
	return buffer[0];
}
bool MPU9255::getDMPInt3Status()
{
	I2Cdev::readBit(devAddr, DMP_INT_STATUS, DMPINT_3_BIT, buffer);
	return buffer[0];
}
bool MPU9255::getDMPInt2Status()
{
	I2Cdev::readBit(devAddr, DMP_INT_STATUS, DMPINT_2_BIT, buffer);
	return buffer[0];
}
bool MPU9255::getDMPInt1Status()
{
	I2Cdev::readBit(devAddr, DMP_INT_STATUS, DMPINT_1_BIT, buffer);
	return buffer[0];
}
bool MPU9255::getDMPInt0Status()
{
	I2Cdev::readBit(devAddr, DMP_INT_STATUS, DMPINT_0_BIT, buffer);
	return buffer[0];
}
// USER_CTRL register (DMP functions)

bool MPU9255::getDMPEnabled()
{
	I2Cdev::readBit(devAddr, USER_CTRL, USERCTRL_DMP_EN_BIT, buffer);
	return buffer[0];
}
void MPU9255::setDMPEnabled(bool enabled)
{
	I2Cdev::writeBit(devAddr, USER_CTRL, USERCTRL_DMP_EN_BIT, enabled);
}
void MPU9255::resetDMP()
{
	I2Cdev::writeBit(devAddr, USER_CTRL, USERCTRL_DMP_RESET_BIT, true);
}

// DMP_BANK register

void MPU9255::setMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank)
{
	bank &= 0x1F;
	if (userBank) bank |= 0x20;
	if (prefetchEnabled) bank |= 0x40;
	I2Cdev::writeByte(devAddr, DMP_BANK, bank);
}

// DMP_RW_PNT register

void MPU9255::setMemoryStartAddress(uint8_t address)
{
	I2Cdev::writeByte(devAddr, DMP_RW_PNT, address);
}

// DMP_REG  register

uint8_t MPU9255::readMemoryByte()
{
	I2Cdev::readByte(devAddr, DMP_REG, buffer);
	return buffer[0];
}
void MPU9255::writeMemoryByte(uint8_t data)
{
	I2Cdev::writeByte(devAddr, DMP_REG, data);
}
void MPU9255::readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address)
{
	setMemoryBank(bank);
	setMemoryStartAddress(address);
	uint8_t chunkSize;
	for (uint16_t i = 0; i < dataSize;)
	{
		// determine correct chunk size according to bank position and data size
		chunkSize = DMP_MEMORY_CHUNK_SIZE;

		// make sure we don't go past the data size
		if (i + chunkSize > dataSize) chunkSize = dataSize - i;

		// make sure this chunk doesn't go past the bank boundary (256 bytes)
		if (chunkSize > 256 - address) chunkSize = 256 - address;

		// read the chunk of data as specified
		I2Cdev::readBytes(devAddr, DMP_REG, chunkSize, data + i);

		// increase byte index by [chunkSize]
		i += chunkSize;

		// uint8_t automatically wraps to 0 at 256
		address + chunkSize;

		// if we aren't done, update bank (if necessary) and address
		if (i < dataSize)
		{
			if (address == 0) bank++;
			setMemoryBank(bank);
			setMemoryStartAddress(address);
		}
	}
}
bool MPU9255::writeMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify, bool useProgMem)
{
	setMemoryBank(bank);
	setMemoryStartAddress(address);
	uint8_t chunkSize;
	uint8_t *verifyBuffer;
	uint8_t *progBuffer;
	uint16_t i;
	uint8_t j;
	if (verify) verifyBuffer = (uint8_t *)malloc(DMP_MEMORY_CHUNK_SIZE);
	if (useProgMem) progBuffer = (uint8_t *)malloc(DMP_MEMORY_CHUNK_SIZE);
	for (i = 0; i < dataSize;)
	{
		// determine correct chunk size according to bank position and data size
		chunkSize = DMP_MEMORY_CHUNK_SIZE;

		// make sure we don't go past the data size
		if (i + chunkSize > dataSize) chunkSize = dataSize - i;

		// make sure this chunk doesn't go past the bank boundary (256 bytes)
		if (chunkSize > 256 - address) chunkSize = 256 - address;

		if (useProgMem)
		{
			// write the chunk of data as specified
			for (j = 0; j < chunkSize; j++) progBuffer[j] = pgm_read_byte(data + i + j);
			I2Cdev::writeBytes(devAddr, DMP_REG, chunkSize, progBuffer);
		}
		else
		{
			// write the chunk of data as specified
			I2Cdev::writeBytes(devAddr, DMP_REG, chunkSize, data + i);
		}

		// verify data if needed
		if (verify && verifyBuffer)
		{
			setMemoryBank(bank);
			setMemoryStartAddress(address);
			I2Cdev::readBytes(devAddr, DMP_REG, chunkSize, verifyBuffer);
			if (memcmp(data + i, verifyBuffer, chunkSize) != 0)
			{
				Serial.print("Block write verification error, bank ");
				Serial.print(bank, DEC);
				Serial.print(", address ");
				Serial.print(address, DEC);
				Serial.print("!\nExpected:");
				for (j = 0; j < chunkSize; j++)
				{
					Serial.print(" 0x");
					if (useProgMem)
					{
						if (progBuffer[j] < 16) Serial.print("0");
						Serial.print(progBuffer[j], HEX);
					}
					else
					{
						if (data[i + j] < 16) Serial.print("0");
						Serial.print(data[i + j], HEX);
					}
				}
				Serial.print("\nReceived:");
				for (uint8_t j = 0; j < chunkSize; j++)
				{
					Serial.print(" 0x");
					if (verifyBuffer[i + j] < 16) Serial.print("0");
					Serial.print(verifyBuffer[i + j], HEX);
				}
				Serial.print("\n");
				free(verifyBuffer);
				if (useProgMem) free(progBuffer);
				return false; // uh oh.
			}
		}

		// increase byte index by [chunkSize]
		i += chunkSize;

		// uint8_t automatically wraps to 0 at 256
		address + chunkSize;

		// if we aren't done, update bank (if necessary) and address
		if (i < dataSize)
		{
			if (address == 0) bank++;
			setMemoryBank(bank);
			setMemoryStartAddress(address);
		}
	}
	if (verify) free(verifyBuffer);
	if (useProgMem) free(progBuffer);
	return true;
}
bool MPU9255::writeProgMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify)
{
	writeMemoryBlock(data, dataSize, bank, address, verify, true);
}

// DMP_REG_1 register

uint8_t MPU9255::getDMPConfig1()
{
	I2Cdev::readByte(devAddr, DMP_REG_1, buffer);
	return buffer[0];
}
void MPU9255::setDMPConfig1(uint8_t config)
{
	I2Cdev::writeByte(devAddr, DMP_REG_1, config);
}

// DMP_REG_2 register

uint8_t MPU9255::getDMPConfig2()
{
	I2Cdev::readByte(devAddr, DMP_REG_2, buffer);
	return buffer[0];
}
void MPU9255::setDMPConfig2(uint8_t config)
{
	I2Cdev::writeByte(devAddr, DMP_REG_2, config);
}
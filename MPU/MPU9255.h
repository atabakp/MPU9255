/*
Note: The MPU9250 is an I2C sensor and uses the Arduino Wire library.
Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or
a 3.3 V Teensy 3.1. We have disabled the internal pull-ups used by the Wire
library in the Wire.h/twi.c utility file. We are also using the 400 kHz fast
I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
*/
#ifndef _MPU9255_H_
#define _MPU9255_H_

#include "MPU9255Regs.h"
#include <i2c_t3.h>
#include "I2Cdev.h"
#define SERIAL_DEBUG true

class MPU9255
{
public:
	MPU9255();
	MPU9255(uint8_t address);

	void initialize();
	bool testConnection();

	void SelfTest(float * destination);
	float getMres();
	float getGres();
	float getAres();

	void calibrateAccel(float * dest1, float * dest2);
	void calibrateGyro();
	// SMPLRT_DIV register
	uint8_t getRate();
	void setRate(uint8_t rate);

	// CONFIG register
	uint8_t getExternalFrameSync();
	void setExternalFrameSync(uint8_t sync);
	uint8_t getDLPFMode();
	void setDLPFMode(uint8_t bandwidth);

	// GYRO_CONFIG register
	bool getGyroXSelfTest();
	void setGyroXSelfTest(bool enabled);
	bool getGyroYSelfTest();
	void setGyroYSelfTest(bool enabled);
	bool getGyroZSelfTest();
	void setGyroZSelfTest(bool enabled);
	uint8_t getFullScaleGyroRange();
	void setFullScaleGyroRange(uint8_t range);

	// SELF_TEST registers
	uint8_t getAccelXSelfTestFactoryTrim();
	uint8_t getAccelYSelfTestFactoryTrim();
	uint8_t getAccelZSelfTestFactoryTrim();

	uint8_t getGyroXSelfTestFactoryTrim();
	uint8_t getGyroYSelfTestFactoryTrim();
	uint8_t getGyroZSelfTestFactoryTrim();

	// ACCEL_CONFIG register
	bool getAccelXSelfTest();
	void setAccelXSelfTest(bool enabled);
	bool getAccelYSelfTest();
	void setAccelYSelfTest(bool enabled);
	bool getAccelZSelfTest();
	void setAccelZSelfTest(bool enabled);
	uint8_t getFullScaleAccelRange();
	void setFullScaleAccelRange(uint8_t range);
	void setAccelDLPF(uint8_t mode);

	// MOT_THR register
	uint8_t getMotionDetectionThreshold();
	void setMotionDetectionThreshold(uint8_t threshold);

	// FIFO_EN register
	bool getTempFIFOEnabled();
	void setTempFIFOEnabled(bool enabled);
	bool getXGyroFIFOEnabled();
	void setXGyroFIFOEnabled(bool enabled);
	bool getYGyroFIFOEnabled();
	void setYGyroFIFOEnabled(bool enabled);
	bool getZGyroFIFOEnabled();
	void setZGyroFIFOEnabled(bool enabled);
	bool getAccelFIFOEnabled();
	void setAccelFIFOEnabled(bool enabled);
	bool getSlave2FIFOEnabled();
	void setSlave2FIFOEnabled(bool enabled);
	bool getSlave1FIFOEnabled();
	void setSlave1FIFOEnabled(bool enabled);
	bool getSlave0FIFOEnabled();
	void setSlave0FIFOEnabled(bool enabled);

	// I2C_MST_CTRL register
	bool getMultiMasterEnabled();
	void setMultiMasterEnabled(bool enabled);
	bool getWaitForExternalSensorEnabled();
	void setWaitForExternalSensorEnabled(bool enabled);
	bool getSlave3FIFOEnabled();
	void setSlave3FIFOEnabled(bool enabled);
	bool getSlaveReadWriteTransitionEnabled();
	void setSlaveReadWriteTransitionEnabled(bool enabled);
	uint8_t getMasterClockSpeed();
	void setMasterClockSpeed(uint8_t speed);

	// I2C_SLV* registers (Slave 0-3)
	uint8_t getSlaveAddress(uint8_t num);
	void setSlaveAddress(uint8_t num, uint8_t address);
	uint8_t getSlaveRegister(uint8_t num);
	void setSlaveRegister(uint8_t num, uint8_t reg);
	bool getSlaveEnabled(uint8_t num);
	void setSlaveEnabled(uint8_t num, bool enabled);
	bool getSlaveWordByteSwap(uint8_t num);
	void setSlaveWordByteSwap(uint8_t num, bool enabled);
	bool getSlaveWriteMode(uint8_t num);
	void setSlaveWriteMode(uint8_t num, bool mode);
	bool getSlaveWordGroupOffset(uint8_t num);
	void setSlaveWordGroupOffset(uint8_t num, bool enabled);
	uint8_t getSlaveDataLength(uint8_t num);
	void setSlaveDataLength(uint8_t num, uint8_t length);

	// I2C_SLV* registers (Slave 4)
	uint8_t getSlave4Address();
	void setSlave4Address(uint8_t address);
	uint8_t getSlave4Register();
	void setSlave4Register(uint8_t reg);
	void setSlave4OutputByte(uint8_t data);
	bool getSlave4Enabled();
	void setSlave4Enabled(bool enabled);
	bool getSlave4InterruptEnabled();
	void setSlave4InterruptEnabled(bool enabled);
	bool getSlave4WriteMode();
	void setSlave4WriteMode(bool mode);
	uint8_t getSlave4MasterDelay();
	void setSlave4MasterDelay(uint8_t delay);
	uint8_t getSlate4InputByte();

	// I2C_MST_STATUS register
	bool getPassthroughStatus();
	bool getSlave4IsDone();
	bool getLostArbitration();
	bool getSlave4Nack();
	bool getSlave3Nack();
	bool getSlave2Nack();
	bool getSlave1Nack();
	bool getSlave0Nack();

	// INT_PIN_CFG register
	bool getInterruptMode();
	void setInterruptMode(bool mode);
	bool getInterruptDrive();
	void setInterruptDrive(bool drive);
	bool getInterruptLatch();
	void setInterruptLatch(bool latch);
	bool getInterruptLatchClear();
	void setInterruptLatchClear(bool clear);
	bool getFSyncInterruptLevel();
	void setFSyncInterruptLevel(bool level);
	bool getFSyncInterruptEnabled();
	void setFSyncInterruptEnabled(bool enabled);
	bool getI2CBypassEnabled();
	void setI2CBypassEnabled(bool enabled);
	bool getClockOutputEnabled();
	void setClockOutputEnabled(bool enabled);

	// INT_ENABLE register

	bool getIntWakeOnMotionEnabled();
	void setIntWakeOnMotionEnabled(bool enabled);
	bool getIntFIFOBufferOverflowEnabled();
	void setIntFIFOBufferOverflowEnabled(bool enabled);
	bool getIntFSyncMasterEnabled();
	void setIntFSyncEnabled(bool enabled);
	bool getIntDataReadyEnabled();
	void setIntDataReadyEnabled(bool enabled);

	// INT_STATUS register
	bool getIntFreefallStatus();
	bool getIntMotionStatus();
	bool getIntZeroMotionStatus();
	bool getIntFIFOBufferOverflowStatus();
	bool getIntI2CMasterStatus();
	bool getIntDataReadyStatus();

	// ACCEL_*OUT_* registers
	void getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz);
	void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
	void getAcceleration(int16_t* x, int16_t* y, int16_t* z);
	int16_t getAccelerationX();
	int16_t getAccelerationY();
	int16_t getAccelerationZ();

	// TEMP_OUT_* registers
	int16_t getTemperature();

	// GYRO_*OUT_* registers
	void getRotation(int16_t* x, int16_t* y, int16_t* z);
	int16_t getRotationX();
	int16_t getRotationY();
	int16_t getRotationZ();

	// EXT_SENS_DATA_* registers
	uint8_t getExternalSensorByte(int position);
	uint16_t getExternalSensorWord(int position);
	uint32_t getExternalSensorDWord(int position);

	// MOT_DETECT_STATUS register
	bool getXNegMotionDetected();
	bool getXPosMotionDetected();
	bool getYNegMotionDetected();
	bool getYPosMotionDetected();
	bool getZNegMotionDetected();
	bool getZPosMotionDetected();
	bool getZeroMotionDetected();

	// I2C_SLV*_DO register
	void setSlaveOutputByte(uint8_t num, uint8_t data);

	// I2C_MST_DELAY_CTRL register
	bool getExternalShadowDelayEnabled();
	void setExternalShadowDelayEnabled(bool enabled);
	bool getSlaveDelayEnabled(uint8_t num);
	void setSlaveDelayEnabled(uint8_t num, bool enabled);

	// SIGNAL_PATH_RESET register
	void resetGyroscopePath();
	void resetAccelerometerPath();
	void resetTemperaturePath();

	// MOT_DETECT_CTRL register
	uint8_t getAccelerometerPowerOnDelay();
	void setAccelerometerPowerOnDelay(uint8_t delay);
	uint8_t getFreefallDetectionCounterDecrement();
	void setFreefallDetectionCounterDecrement(uint8_t decrement);
	uint8_t getMotionDetectionCounterDecrement();
	void setMotionDetectionCounterDecrement(uint8_t decrement);

	// USER_CTRL register
	bool getFIFOEnabled();
	void setFIFOEnabled(bool enabled);
	bool getI2CMasterModeEnabled();
	void setI2CMasterModeEnabled(bool enabled);
	void switchSPIEnabled(bool enabled);
	void resetFIFO();
	void resetI2CMaster();
	void resetSensors();

	// PWR_MGMT_1 register
	void reset();
	bool getSleepEnabled();
	void setSleepEnabled(bool enabled);
	bool getWakeCycleEnabled();
	void setWakeCycleEnabled(bool enabled);
	bool getTempSensorEnabled();
	void setTempSensorEnabled(bool enabled);
	uint8_t getClockSource();
	void setClockSource(uint8_t source);

	// PWR_MGMT_2 register

	bool getStandbyXAccelEnabled();
	void setStandbyXAccelEnabled(bool enabled);
	bool getStandbyYAccelEnabled();
	void setStandbyYAccelEnabled(bool enabled);
	bool getStandbyZAccelEnabled();
	void setStandbyZAccelEnabled(bool enabled);
	bool getStandbyXGyroEnabled();
	void setStandbyXGyroEnabled(bool enabled);
	bool getStandbyYGyroEnabled();
	void setStandbyYGyroEnabled(bool enabled);
	bool getStandbyZGyroEnabled();
	void setStandbyZGyroEnabled(bool enabled);

	// FIFO_COUNT_* registers
	uint16_t getFIFOCount();

	// FIFO_R_W register
	uint8_t getFIFOByte();
	void setFIFOByte(uint8_t data);

	// WHO_AM_I register
	uint8_t getDeviceID();
	void setDeviceID(uint8_t id);

	// XA_OFFS_* registers
	int16_t getXAccelOffset();
	void setXAccelOffset(int16_t offset);

	// YA_OFFS_* register
	int16_t getYAccelOffset();
	void setYAccelOffset(int16_t offset);

	// ZA_OFFS_* register
	int16_t getZAccelOffset();
	void setZAccelOffset(int16_t offset);

	// INT_ENABLE register (DMP functions)
	bool getIntPLLReadyEnabled();
	void setIntPLLReadyEnabled(bool enabled);
	bool getIntDMPEnabled();
	void setIntDMPEnabled(bool enabled);

	// DMP_INT_STATUS
	bool getDMPInt5Status();
	bool getDMPInt4Status();
	bool getDMPInt3Status();
	bool getDMPInt2Status();
	bool getDMPInt1Status();
	bool getDMPInt0Status();

	// INT_STATUS register (DMP functions)
	bool getIntPLLReadyStatus();
	bool getIntDMPStatus();

	// USER_CTRL register (DMP functions)
	bool getDMPEnabled();
	void setDMPEnabled(bool enabled);
	void resetDMP();

	// BANK_SEL register
	void setMemoryBank(uint8_t bank, bool prefetchEnabled = false, bool userBank = false);

	// MEM_START_ADDR register
	void setMemoryStartAddress(uint8_t address);

	// MEM_R_W register
	uint8_t readMemoryByte();
	void writeMemoryByte(uint8_t data);
	void readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank = 0, uint8_t address = 0);
	bool writeMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank = 0, uint8_t address = 0, bool verify = true, bool useProgMem = false);
	bool writeProgMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank = 0, uint8_t address = 0, bool verify = true);

	// DMP_CFG_1 register
	uint8_t getDMPConfig1();
	void setDMPConfig1(uint8_t config);

	// DMP_CFG_2 register
	uint8_t getDMPConfig2();
	void setDMPConfig2(uint8_t config);

private:
	uint8_t devAddr;
	uint8_t buffer[14];
};  // class MPU9255

#endif // _MPU9255_H_

/*
 Name:		MPU.ino
 Created:	2/23/2017 10:12:41 PM
 Author:	Cisco
*/
// the setup function runs once when you press reset or power the board

#include "I2Cdev.h"
#include "MPU9255.h"

MPU9255 mpu;

float aRes, gRes, mRes;             // scale resolutions per LSB for the sensors

int16_t ax, ay, az;
int16_t gx, gy, gz;
void setup()
{
	Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
	Serial.begin(38400);
	delay(1000);

	// initialize device
	Serial.println(F("Initializing I2C devices..."));
	mpu.initialize();

	// verify connection
	Serial.println(F("Testing device connections..."));
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

	float SelfTest[6];       // holds results of gyro and accelerometer self test
	mpu.SelfTest(SelfTest); // Start by performing self test and reporting values
	Serial.println("MPU9250 Self Test:");
	Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0], 1); Serial.println("% of factory value");
	Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1], 1); Serial.println("% of factory value");
	Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2], 1); Serial.println("% of factory value");
	Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3], 1); Serial.println("% of factory value");
	Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4], 1); Serial.println("% of factory value");
	Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5], 1); Serial.println("% of factory value");
	delay(1000);

	// get sensor resolutions, only need to do this once
	aRes = mpu.getAres();
	gRes = mpu.getGres();
	//getMres();

	Serial.println(" Calibrate MPU9250 gyro and accel");
	accelgyrocalMPU9250(MPU9250gyroBias, MPU9250accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
	Serial.println("accel biases (mg)"); Serial.println(1000.*MPU9250accelBias[0]); Serial.println(1000.*MPU9250accelBias[1]); Serial.println(1000.*MPU9250accelBias[2]);
	Serial.println("gyro biases (dps)"); Serial.println(MPU9250gyroBias[0]); Serial.println(MPU9250gyroBias[1]); Serial.println(MPU9250gyroBias[2]);
}

// the loop function runs over and over again until power down or reset
void loop()
{
	//mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

	//// these methods (and a few others) are also available
	////accelgyro.getAcceleration(&ax, &ay, &az);
	////accelgyro.getRotation(&gx, &gy, &gz);

	//// display tab-separated accel/gyro x/y/z values
	//Serial.print("a/g:\t");
	//Serial.print(ax); Serial.print("\t");
	//Serial.print(ay); Serial.print("\t");
	//Serial.print(az); Serial.print("\t");
	//Serial.print(gx); Serial.print("\t");
	//Serial.print(gy); Serial.print("\t");
	//Serial.println(gz);
}
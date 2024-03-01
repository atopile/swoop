#include <Arduino.h>
#include <Wire.h>
#include "BMI088.h"
#include "LIS3MDL.h"
#include "SensorFusion.h"

LIS3MDL mag;
Bmi088 bmi(Wire, 0x18, 0x68);

SF fusion;
float gx, gy, gz, ax, ay, az, mx, my, mz;
float pitch, roll, yaw;
float deltat;
float T;

char report[80];

void setup()
{
	Serial.begin(115200);
	while (!Serial)
		;
	Serial.println("Serial started!");

	// Configure the i2c bus (Wire) to use:
	//  - i2c.sda ~ gpio20
	//  - i2c.scl ~ gpio21
	Wire.setSDA(20);
	Wire.setSCL(21);
	Wire.begin();
	Serial.println("I2C started!");

	if (!mag.init())
	{
		while (1)
		{
			Serial.println("Failed to detect and initialize magnetometer!");
			delay(1000);
		}
	}
	mag.enableDefault();
	Serial.println("Magnetometer started!");

	// Start the BNO055 sensor
	delay(1000);
	int bmi_status = bmi.begin();
	if (bmi_status < 1)
	{
		while (1)
		{
			snprintf(report, sizeof(report), "Failed to init BMI088! Status: %d", bmi_status);
			Serial.println(report);
			delay(1000);
		}
	}
	Serial.println("BMI088 started!");

	delay(1000);
	Serial.println("Starting...");
}

void loop()
{
	mag.read();
	snprintf(
		report, sizeof(report), "M: %6d %6d %6d",
		mag.m.x, mag.m.y, mag.m.z);
	Serial.println(report);
	mx = mag.m.x;
	my = mag.m.y;
	mz = mag.m.z;

	bmi.readSensor();
	T = bmi.getTemperature_C();
	ax = bmi.getAccelX_mss();
	ay = bmi.getAccelY_mss();
	az = bmi.getAccelZ_mss();
	gx = bmi.getGyroX_rads();
	gy = bmi.getGyroY_rads();
	gz = bmi.getGyroZ_rads();

	snprintf(
		report, sizeof(report), "T: %6.2f A: %6.2f %6.2f %6.2f G: %6.4f %6.4f %6.4f",
		T, ax, ay, az, gx, gy, gz);
	Serial.println(report);

	deltat = fusion.deltatUpdate(); //this have to be done before calling the fusion update
	//choose only one of these two:
	// fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
	fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //else use the magwick, it is slower but more accurate

	pitch = fusion.getPitch();
	roll = fusion.getRoll();    //you could also use getRollRadians() ecc
	yaw = fusion.getYaw();

	Serial.print("Pitch:\t"); Serial.println(pitch);
	Serial.print("Roll:\t"); Serial.println(roll);
	Serial.print("Yaw:\t"); Serial.println(yaw);
	Serial.println();

	delay(100);
}

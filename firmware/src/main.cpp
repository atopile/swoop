#include <Arduino.h>
#include <Wire.h>
#include "BMI088.h"
#include "Adafruit_BNO055.h"
#include "LIS3MDL.h"

LIS3MDL mag;

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
		while (1) {
			Serial.println("Failed to detect and initialize magnetometer!");
			delay(1000);
		}
	}
	// mag.enableDefault();
	// Serial.println("Magnetometer started!");
}

void loop()
{
	// mag.read();

	// snprintf(
	// 	report, sizeof(report), "M: %6d %6d %6d",
	// 	mag.m.x, mag.m.y, mag.m.z);
	// Serial.println(report);
	Serial.println("Hello World!");

	delay(100);
}

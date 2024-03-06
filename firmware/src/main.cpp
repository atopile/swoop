#include <Arduino.h>
#include <Wire.h>
#include "BMI088.h"
#include "LIS3MDL.h"
#include "SensorFusion.h"
// This demo explores two reports (SH2_ARVR_STABILIZED_RV and SH2_GYRO_INTEGRATED_RV) both can be used to give
// quartenion and euler (yaw, pitch roll) angles.  Toggle the FAST_MODE define to see other report.
// Note sensorValue.status gives calibration accuracy (which improves over time)
#include <Adafruit_BNO08x.h>

// #define FAST_MODE

// For SPI mode, we also need a RESET
// #define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1

#define RADIAN_TO_DEG( x )    ( ( x ) * 180.F / ( float ) M_PI )

#define SMALL_EPSILON    ( 0.000001F )
#define SMALL_TILT    ( 0.01F )

struct euler_t
{
	float yaw;
	float pitch;
	float roll;
} ypr;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

#ifdef FAST_MODE
// Top frequency is reported to be 1000Hz (but freq is somewhat variable)
sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
long reportIntervalUs = 2000;
#else
// Top frequency is about 250Hz but this report is more accurate
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;
#endif
void setReports(sh2_SensorId_t reportType, long report_interval)
{
	Serial.println("Setting desired reports");
	if (!bno08x.enableReport(reportType, report_interval))
	{
		Serial.println("Could not enable stabilized remote vector");
	}
}

void setup(void)
{

	Serial.begin(115200);
	while (!Serial)
		delay(10); // will pause Zero, Leonardo, etc until serial console opens

	Serial.println("Adafruit BNO08x test!");

	// Try to initialize!
	Wire.setSDA(20);
	Wire.setSCL(21);
	while (!bno08x.begin_I2C(0x4A))
	{
		Serial.println("Failed to find BNO08x chip");
		delay(1000);
	}
	Serial.println("BNO08x Found!");

	setReports(reportType, reportIntervalUs);

	Serial.println("Reading events");
	delay(100);

	pinMode(2, INPUT);
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t *ypr, bool degrees = false)
{

	float sqr = sq(qr);
	float sqi = sq(qi);
	float sqj = sq(qj);
	float sqk = sq(qk);

	ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
	ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
	ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

	if (degrees)
	{
		ypr->yaw *= RAD_TO_DEG;
		ypr->pitch *= RAD_TO_DEG;
		ypr->roll *= RAD_TO_DEG;
	}
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t *rotational_vector, euler_t *ypr, bool degrees = false)
{
	quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t *rotational_vector, euler_t *ypr, bool degrees = false)
{
	quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToControlAngles(sh2_Quaternion_t *quaternion, euler_t *ypr)
{
	// ypr->pitch = atan2((2 * (quaternion->x * quaternion->y + quaternion->w * quaternion->z)), (quaternion->w*quaternion->w + quaternion->x*quaternion->x - quaternion->y*quaternion->y - quaternion->z*quaternion->z));
	// ypr->roll = atan2((- 2 * (quaternion->x * quaternion->z - quaternion->w * quaternion->y)), (quaternion->w*quaternion->w + quaternion->x*quaternion->x - quaternion->y*quaternion->y - quaternion->z*quaternion->z));

	// float torsion_quat_norm = sqrt(quaternion->w*quaternion->w + quaternion->x*quaternion->x);
	// sh2_Quaternion_t torsion_quaternion = {quaternion->w/torsion_quat_norm, quaternion->x/torsion_quat_norm, 0, 0};
	// ypr->yaw =

    //yaw rotation around x
    //pitch rotation around z
    //roll rotation around y
    float q0 = quaternion->w; //w
    float q1 = quaternion->z; //x
    float q2 = - quaternion->x; //y
    float q3 = quaternion->y; //z

    float fAngle;
    float fInvSqrt_q0q0_q1q1 = 1.0f/sqrtf(q0 * q0 + q1 * q1);

    float r11, r12;

    float xY = 0;
    float xP = 0;
    float xR = 0;

    /* Yaw extraction */

    float q0_tors = q0 * fInvSqrt_q0q0_q1q1;
    float q1_tors = q1 * fInvSqrt_q0q0_q1q1;

    fAngle = 2 * acosf( q0_tors );
    if( 1 - ( q0_tors * q0_tors ) < SMALL_EPSILON )
    {
        xY = q1_tors;
    }
    else
    {
        float fScale = sqrtf( 1 - q0_tors * q0_tors );
        xY = q1_tors / fScale;
    }
    xY = xY * fAngle;

    /* Pitch extraction */
    /* Using euler angles order zyx and using z as pitch */
    r11 = 2*(q1*q2 + q0*q3);
    r12 = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    xP = atan2f(r11, r12);

    /* Roll extraction */
    /* Using euler angles order yzx and using y as roll */
    r11 = -2*(q1*q3 - q0*q2);
    //r12 = q0*q0 + q1*q1 - q2*q2 - q3*q3; // already calculated above
    xR = atan2f(r11, r12);

    ypr->yaw = -RADIAN_TO_DEG( xY ); //yaw
    ypr->pitch = RADIAN_TO_DEG( xP ); //pitch
    ypr->roll = RADIAN_TO_DEG( xR ); //roll

}

int prevButtonState_1 = 1;
int prevButtonState_2 = 1;

void loop()
{

	if (bno08x.wasReset())
	{
		Serial.print("sensor was reset ");
		setReports(reportType, reportIntervalUs);
	}

	if (bno08x.getSensorEvent(&sensorValue))
	{
		// in this demo only one report type will be received depending on FAST_MODE define (above)
		switch (sensorValue.sensorId)
		{
		case SH2_ARVR_STABILIZED_RV:
			quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
		case SH2_GYRO_INTEGRATED_RV:
			// faster (more noise?)
			quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
			break;
		}
		static long last = 0;
		long now = micros();
		sh2_Quaternion_t quat = {sensorValue.un.arvrStabilizedRV.i, sensorValue.un.arvrStabilizedRV.j, sensorValue.un.arvrStabilizedRV.k, sensorValue.un.arvrStabilizedRV.real};
		quaternionToControlAngles(&quat, &ypr);
		float q0 = quat.w; //w
		float q1 = quat.z; //x
		float q2 = - quat.x; //y
		float q3 = quat.y; //z
		// Serial.print(now - last);
		// Serial.print("\t");
		// last = now;
		// Serial.print(sensorValue.status);
		// Serial.print("\t"); // This is accuracy in the range of 0 to 3
		Serial.print("w");
		Serial.print(q0);
		Serial.print("w");
		Serial.print("a");
		Serial.print(q1);
		Serial.print("a");
		Serial.print("b");
		Serial.print(q2);
		Serial.print("b");
		Serial.print("c");
		Serial.print(q3);
		Serial.println("c");
		// Serial.print("yaw: ");
		// Serial.print(ypr.yaw);
		// Serial.print("\t");
		// Serial.print("pitch: ");
		// Serial.print(ypr.pitch);
		// Serial.print("\t");
		// Serial.print("roll: ");
		// Serial.println(ypr.roll);

		int buttonState_1 = digitalRead(2);

		if (!buttonState_1 && prevButtonState_1) {
			prevButtonState_1 = buttonState_1;
			sh2_Quaternion_t myQuaternion = {0, 0, q1, q0};
			sh2_setReorientation(&myQuaternion);
			//Serial.println("hello");
		}
		else {
			prevButtonState_1 = buttonState_1;
		}

		int buttonState_2 = digitalRead(3);

		if (!buttonState_2 && prevButtonState_2) {
			prevButtonState_2 = buttonState_2;
			sh2_Quaternion_t myQuaternion = {0, 0, 0, 1};
			sh2_setReorientation(&myQuaternion);
			//Serial.println("hello");
		}
		else {
			prevButtonState_2 = buttonState_2;
		}
	}
}

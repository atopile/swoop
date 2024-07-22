#include "../lib/RotationMath/RotationMath.hpp"

#include <Arduino.h>
#include "main.h"
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SensorFusion.h>
#include "Adafruit_DRV2605.h"

#include "../lib/LED/LED.hpp"

#include "../lib/RCMapper/rcmapper.hpp"
#include "../lib/PPM/PPMController.hpp"

// constants
#define LONG_PRESS_TIME 2000 // Long press duration in milliseconds

// Sensors
Adafruit_LIS3MDL mag;
Adafruit_MPU6050 mpu;

// Haptic Driver
Adafruit_DRV2605 haptic;

// Sensor Fusion
SF fusion;

LED led;
RotationMath rotation_math;

PPMController ppm_controller;

// Variables
float gx, gy, gz, ax, ay, az, mx, my, mz;
float pitch, roll, yaw, throttle;
float deltat;
volatile int flightMode = 1; // Starts at mode 1
unsigned long buttonPressTime = 0;
bool buttonState = false;
bool lastButtonState = false;
bool deviceState = false;

float getThrottle() {
  int rawValue = analogRead(A0); // Assuming A0 is the throttle input pin
  float throttle = map(rawValue, 730, 200, 0, 100);
  throttle = constrain(throttle, 0, 100); // Ensuring throttle stays within bounds
  return throttle;
}

void changeFlightMode() {
  // Function to change the flight mode of the drone
  // This function is called when the MENU button is pressed
  // It cycles through the flight modes (1, 2, 3) and updates the LEDs accordingly
  static unsigned long lastChangeTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastChangeTime < 200) {
    return; // Exit the function if it's been less than 200ms since the last change
  }
  lastChangeTime = currentTime; // Update the last change time
  flightMode = flightMode % 3 + 1; // Cycles through 1, 2, 3 and wraps around
  led.updateLEDs(deviceState, flightMode);
  Serial.println("save");

  float* quat = fusion.getQuat();
  rotation_math.setZeroQuaternion(matrix::Quatf(fusion.getQuat()));
}

void nonBlockingButtonCheck() {
  // Starts a timer if button is pressed, resets if released
  buttonState = digitalRead(POWER_BUTTON_PIN);
  // Serial.print("Button state: "); Serial.println(buttonState ? "HIGH" : "LOW");

  // if the button is being pressed, and the timer is not running, start the timer
  if (buttonState == LOW) {
    Serial.println("Button is pressed.");
    if (buttonPressTime == 0){
      buttonPressTime = millis();
      Serial.println("Timer started.");
    }
    else if (millis() - buttonPressTime > LONG_PRESS_TIME) {
      // If the button is pressed for LONG_PRESS_TIME, toggle the device state
      deviceState = !deviceState;
      Serial.print("Device state toggled to: "); Serial.println(deviceState ? "ON" : "OFF");
      buttonPressTime = 0;
    }
    else {
      // update the led strip based on the percentage of time the button has been pressed
      int percentage = (millis() - buttonPressTime) * 100 / LONG_PRESS_TIME;
      Serial.print("Button hold percentage: "); Serial.println(percentage);
      led.animatePowerOn(percentage, deviceState);
    }
  }
  // else, if the button is not pressed, reset the timer to 0
  else if (buttonState == HIGH) {
    if (buttonPressTime != 0) {
      Serial.println("Button released. Timer reset.");
    }
    buttonPressTime = 0;
  }

  lastButtonState = buttonState;
}


void setup() {
  // Setup Serial
  Serial.begin(115200);
  led.setup();
  led.updateLEDs(deviceState, flightMode);

  // Setup Buttons
  pinMode(MENU_BUTTON_PIN, INPUT_PULLUP); // Set the button pin as input with pull-up
  attachInterrupt(digitalPinToInterrupt(MENU_BUTTON_PIN), changeFlightMode, FALLING); // Attach interrupt for button press


  // // Setup I2C
  Wire.setSCL(I2C_SDA_PIN);
  Wire.setSDA(I2C_SCL_PIN);

  Wire.begin();

  mag.begin_I2C(LIS3MDLTR_ADDRESS, &Wire);
  mpu.begin(MPU6050_ADDRESS, &Wire);

  ppm_controller.initialize(PPM_PIN);
}

void loop() {
  nonBlockingButtonCheck();
  led.updateLEDs(deviceState, flightMode);

  if (!deviceState) {
    Serial.println("Device is powered off.");
    return;
  }


  mag.read();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);


  deltat = fusion.deltatUpdate();
  // the chips are in different orientation. Following both of those
  // mag datasheet: https://www.mouser.com/datasheet/2/389/lis3mdl-1849592.pdf
  // mpu datasheet: https://www.cdiweb.com/datasheets/invensense/mpu_6500_rev1.0.pdf
  fusion.MahonyUpdate(g.gyro.x, g.gyro.y, g.gyro.z, a.acceleration.x, a.acceleration.y, a.acceleration.z, deltat);

  // pitch = fusion.getPitch();
  // roll = fusion.getRoll();    //you could also use getRollRadians() ecc
  // yaw = fusion.getYaw();
  throttle = getThrottle();

  float* quat = fusion.getQuat();
  matrix::Eulerf control_angles = rotation_math.getControlAngles(matrix::Quatf(quat));
  // Serial.print(">dt:");
  // Serial.println(deltat);
  // Serial.print(">gx:");
  // Serial.println(g.gyro.x);
  // Serial.print(">gy:");
  // Serial.println(g.gyro.y);
  // Serial.print(">gz:");
  // Serial.println(g.gyro.z);
  // Serial.print(">ax:");
  // Serial.println(a.acceleration.x);
  // Serial.print(">ay:");
  // Serial.println(a.acceleration.y);
  // Serial.print(">az:");
  // Serial.println(a.acceleration.z);
  // Serial.print(">mx:");
  // Serial.println(mag.x);
  // Serial.print(">my:");
  // Serial.println(mag.y);
  // Serial.print(">mz:");
  // Serial.println(mag.z);
  // Serial.print(">Throttle:\t"); Serial.println(throttle);
  // Serial.println();
  // print the quaternion
  // Serial.print(">w:");
  // Serial.println(quat[0]);
  // Serial.print(">x:");
  // Serial.println(quat[1]);
  // Serial.print(">y:");
  // Serial.println(quat[2]);
  // Serial.print(">z:");
  // Serial.println(quat[3]);

  // Serial.print(">Roll: ");
  // Serial.println(degrees(control_angles(0)));
  // Serial.print(">Pitch: ");
  // Serial.println(degrees(control_angles(1)));
  // Serial.print(">Yaw: ");
  // Serial.println(degrees(control_angles(2)));
  // Serial.println("");

  RCMapper mapper;

  // Create a test input frame
  inputFrameControlSignals input_frame;
  input_frame.control_angles = control_angles;
  input_frame.throttle = 1000; // Example throttle value
  input_frame.additional_channels[0] = 1000;
  input_frame.additional_channels[1] = 1000;
  input_frame.additional_channels[2] = 1000;
  input_frame.additional_channels[3] = 1000;

  // Map values using the mapper
  ControlSignals output_signals = mapper.mapValues(input_frame);

  ppm_controller.setPPMValues(output_signals);

  Serial.print("Yaw: ");
  Serial.println(output_signals.yaw);
  Serial.print("Pitch: ");
  Serial.println(output_signals.pitch);
  Serial.print("Roll: ");
  Serial.println(output_signals.roll);
  Serial.print("Throttle: ");
  Serial.println(output_signals.throttle);

  delay(100);
}

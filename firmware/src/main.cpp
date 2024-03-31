#include <Arduino.h>
#include "main.h"
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SensorFusion.h>
#include "Adafruit_DRV2605.h"
#include <FastLED.h>

// constants
#define LONG_PRESS_TIME 2000 // Long press duration in milliseconds
#define NUM_LEDS 4

// Sensors
Adafruit_LIS3MDL mag;
Adafruit_MPU6050 mpu;

// Haptic Driver
Adafruit_DRV2605 haptic;

// Sensor Fusion
SF fusion;

// LED Strip
CRGB leds[NUM_LEDS];



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

void animatePowerOn(int percentage) {
  const CRGB lowPowerColor = CRGB::Black;
  CRGB highPowerColor = CRGB::Green;

  if (deviceState) {
    highPowerColor = CRGB::Red;
  }

  // Calculate how many LEDs should be lit based on the percentage
  int numLedsLit = (percentage * NUM_LEDS) / 100;
  int remainder = (percentage * NUM_LEDS) % 100; // For partial LED coloring

  for (int i = 0; i < NUM_LEDS; i++) {
    if (i < numLedsLit) {
      // These LEDs are fully powered/on
      leds[i] = highPowerColor;
    } else if (i == numLedsLit && remainder != 0) {
      // This LED is partially lit based on the remainder, mix colors
      float mixRatio = remainder / 100.0;
      leds[i] = blend(lowPowerColor, highPowerColor, mixRatio * 255);
    } else {
      // These LEDs are off/low power
      leds[i] = lowPowerColor;
    }
  }

  FastLED.show();
}

void updateLEDs() {
  if(digitalRead(POWER_BUTTON_PIN) == LOW){
    FastLED.show();
    return;
  }

  if (!deviceState) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::Black; // Turn off all LEDs
    }
    FastLED.show();
    return;
  }
  else {
    leds[0] = CRGB::Green;
  }
  leds[0] = CRGB::Green;
  FastLED.show();

  switch (flightMode) {
    case 1:
      Serial.println(">Flight Mode: 1");
      leds[1] = CRGB::Blue;
      leds[2] = CRGB::Black; // LED 3 off
      leds[3] = CRGB::Black; // LED 4 off
      FastLED.show();
      break;
    case 2:
      Serial.println(">Flight Mode: 2");
      leds[1] = CRGB::Black; // LED 2 off
      leds[2] = CRGB::Blue;
      leds[3] = CRGB::Black; // LED 4 off
      FastLED.show();
      break;
    case 3:
      Serial.println(">Flight Mode: 3");
      leds[1] = CRGB::Black; // LED 2 off
      leds[2] = CRGB::Black; // LED 3 off
      leds[3] = CRGB::Blue;
      FastLED.show();
      break;
  }
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
  updateLEDs();
}

void nonBlockingButtonCheck() {
  // Starts a timer if button is pressed, resets if released
  buttonState = digitalRead(POWER_BUTTON_PIN);
  Serial.print("Button state: "); Serial.println(buttonState ? "HIGH" : "LOW");

  // if the button is being pressed, and the timer is not running, start the timer
  if (buttonState == LOW ) {
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
      animatePowerOn(percentage);
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
  // while (!Serial); delay(10);

  Serial.println("Adafruit LIS3MDL test!");

  // Setup LEDs
  FastLED.addLeds<NEOPIXEL, LED_DATA_OUT_PIN>(leds, NUM_LEDS);
  updateLEDs();

  // Setup Buttons
  pinMode(MENU_BUTTON_PIN, INPUT_PULLUP); // Set the button pin as input with pull-up
  attachInterrupt(digitalPinToInterrupt(MENU_BUTTON_PIN), changeFlightMode, FALLING); // Attach interrupt for button press


  // // Setup I2C
  Wire.setSCL(I2C_SDA_PIN);
  Wire.setSDA(I2C_SCL_PIN);

  Wire.begin();

  mag.begin_I2C(LIS3MDLTR_ADDRESS, &Wire);
  mpu.begin(MPU6050_ADDRESS, &Wire);
}

void loop() {
  nonBlockingButtonCheck();
  updateLEDs();

  if (!deviceState) {
    Serial.println("Device is powered off.");
    return;
  }


  mag.read();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);


  deltat = fusion.deltatUpdate();
  fusion.MadgwickUpdate(g.gyro.x, g.gyro.y, g.gyro.z, a.acceleration.x, a.acceleration.y, a.acceleration.z, mag.x, mag.y, mag.z, deltat);

  pitch = fusion.getPitch();
  roll = fusion.getRoll();    //you could also use getRollRadians() ecc
  yaw = fusion.getYaw();
  throttle = getThrottle();

  Serial.print(">Pitch:\t"); Serial.println(pitch);
  Serial.print(">Roll:\t"); Serial.println(roll);
  Serial.print(">Yaw:\t"); Serial.println(yaw);
  Serial.print(">Throttle:\t"); Serial.println(throttle);
  Serial.println();

  // animatePowerOn(throttle);

  delay(10);

}

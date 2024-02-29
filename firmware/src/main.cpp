#include <Arduino.h>

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

void loop() {
  // print out a message
  Serial.println("Hello, world!");
  // wait for a second
  delay(1000);
}

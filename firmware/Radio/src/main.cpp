#include <Arduino.h>
#include <SPI.h>                                               // The LoRa device is SPI based, so load the SPI library                                         
#include "Settings.h"                                          // Include the settings file for frequencies, LoRa settings, etc.   

void setup()
{
  pinMode(NSS, OUTPUT);                                       // Set NSS as output
  digitalWrite(NSS, HIGH);                                    // Set NSS high to indicate no communication
  // SPI.setMISO(SPI_MISO);                                      // Set the MISO pin
  // SPI.setMOSI(SPI_MOSI);                                      // Set the MOSI pin
  // SPI.setSCLK(SPI_SCK);                                        // Set the SCK pin
  SPI.begin();              // Initialize SPI with the defined pins
}

void loop()
{
  uint8_t helloWorld[] = "Hello World";                       // Define the message to be sent
  digitalWrite(NSS, LOW);                                     // Set NSS low to start communication

  for (unsigned int i = 0; i < sizeof(helloWorld) - 1; i++) { // Loop through each byte of the message (-1 to exclude the null terminator)
    SPI.transfer(helloWorld[i]);                              // Send each byte via SPI
  }

  digitalWrite(NSS, HIGH);                                    // Set NSS high to end communication
  delay(1000);                                                // Wait for a second before sending the message again
}

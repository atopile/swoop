#pragma once

#include <FastLED.h>

class LED
{
public:
    LED() = default;
    ~LED() = default;
    static constexpr int NUM_LEDS = 4;
    static constexpr uint8_t LED_DATA_OUT_PIN = 7;

    void setup();
    void animatePowerOn(const int percentage, const bool is_device_on);
    void updateLEDs(const bool is_device_on, const int flight_mode);

private:
    NEOPIXEL<LED_DATA_OUT_PIN> _led_controller;
    CRGB _leds[NUM_LEDS]; ///< LED Strip
};

#include "LED.hpp"

void LED::setup() {
    FastLED.addLeds(&_led_controller, _leds, NUM_LEDS);
}

void LED::animatePowerOn(const int percentage, const bool is_device_on) {
    const CRGB lowPowerColor = CRGB::Black;
    CRGB highPowerColor = CRGB::Green;

    if (is_device_on) {
        highPowerColor = CRGB::Red;
    }

    // Calculate how many LEDs should be lit based on the percentage
    int numLedsLit = (percentage * NUM_LEDS) / 100;
    int remainder = (percentage * NUM_LEDS) % 100; // For partial LED coloring

    for (int i = 0; i < NUM_LEDS; i++) {
        if (i < numLedsLit) {
            // These LEDs are fully powered/on
            _leds[i] = highPowerColor;
        } else if (i == numLedsLit && remainder != 0) {
            // This LED is partially lit based on the remainder, mix colors
            float mixRatio = remainder / 100.0;
            _leds[i] = blend(lowPowerColor, highPowerColor, mixRatio * 255);
        } else {
            // These LEDs are off/low power
            _leds[i] = lowPowerColor;
        }
    }

    FastLED.show();
}

#define POWER_BUTTON_PIN 13 // Move Button handling out of here

void LED::updateLEDs(const bool is_device_on, const int flight_mode) {
    if(digitalRead(POWER_BUTTON_PIN) == LOW){
        FastLED.show();
        return;
    }

    if (!is_device_on) {
        for (int i = 0; i < NUM_LEDS; i++) {
            _leds[i] = CRGB::Black; // Turn off all LEDs
        }
        FastLED.show();
        return;
    } else {
        _leds[0] = CRGB::Green;
    }
    _leds[0] = CRGB::Green;
    FastLED.show();

    switch (flight_mode) {
    case 1:
        // Serial.println(">Flight Mode: 1");
        _leds[1] = CRGB::Blue;
        _leds[2] = CRGB::Black; // LED 3 off
        _leds[3] = CRGB::Black; // LED 4 off
        FastLED.show();
        break;
    case 2:
        // Serial.println(">Flight Mode: 2");
        _leds[1] = CRGB::Black; // LED 2 off
        _leds[2] = CRGB::Blue;
        _leds[3] = CRGB::Black; // LED 4 off
        FastLED.show();
        break;
    case 3:
        // Serial.println(">Flight Mode: 3");
        _leds[1] = CRGB::Black; // LED 2 off
        _leds[2] = CRGB::Black; // LED 3 off
        _leds[3] = CRGB::Blue;
        FastLED.show();
        break;
    }
}

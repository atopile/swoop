#include "pico/stdlib.h"

#include "PPMController.hpp"
#include "hardware/pio.h"
#include "ppm.pio.h"

PPMController::PPMController() {
    // Constructor implementation
}

PPMController::~PPMController() {
    // Destructor implementation
}

void PPMController::initialize(const uint& pin_number) {
    ppm_program_init(pio1, pin_number);
}

void PPMController::setPPMValues(const ControlSignals& signals) {
    // Convert and set PPM values for each channel
    ppm_set_value(1, convertToPPM(signals.yaw));
    ppm_set_value(2, convertToPPM(signals.pitch));
    ppm_set_value(3, convertToPPM(signals.roll));
    ppm_set_value(4, convertToPPM(signals.throttle));

    // Not setting the additional channels for now
    // for (int i = 0; i < 4; ++i) {
    //     ppm_set_value(4 + i, convertToPPM(signals.additional_channels[i]));
    // }
}

uint16_t PPMController::convertToPPM(uint16_t signal) {
    // Assuming the signal is already in the 1000 to 2000 range, we directly return it.
    // If further scaling or conversion is needed, it can be added here.
    return signal;
}

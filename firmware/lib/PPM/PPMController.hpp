#pragma once

#ifndef PPMCONTROLLER_H
#define PPMCONTROLLER_H

#include "../RCMapper/rcmapper.hpp"

class PPMController {
public:
    PPMController();
    ~PPMController();

    // Initialize the PPM controller
    void initialize(const uint& pin_number);

    // Function to set the PPM values based on control signals
    void setPPMValues(const ControlSignals& signals);

private:
    // Function to convert a control signal to PPM value
    uint16_t convertToPPM(uint16_t signal);
};

#endif // PPMCONTROLLER_H
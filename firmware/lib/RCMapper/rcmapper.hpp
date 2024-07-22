#pragma once

#ifndef RCMAPPER_H
#define RCMAPPER_H

#include "../matrix/matrix/math.hpp"

#define MIN_ANGLE -30.0f
#define MAX_ANGLE 30.0f
#define MIN_SIGNAL 1000
#define MAX_SIGNAL 2000

struct ControlSignals {
    uint16_t yaw;
    uint16_t pitch;
    uint16_t roll;
    uint16_t throttle;
    uint16_t additional_channels[4];
};

struct inputFrameControlSignals {
    matrix::Eulerf control_angles;
    uint16_t throttle;
    uint16_t additional_channels[4];
};

class RCMapper {
public:
    RCMapper();
    ~RCMapper();

    // Function to map input angles and values to output values
    ControlSignals mapValues(inputFrameControlSignals input_frame);

private:
    // Map a single angle to the 1000 to 2000 range
    uint16_t mapSingleAngle(float angle);
};

#endif // RCMAPPER_HPP

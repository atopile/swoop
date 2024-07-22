#include "rcmapper.hpp"
#include <Arduino.h>
#include <cmath> // For mathematical operations
#include <algorithm> // For std::clamp

RCMapper::RCMapper() {
    // Constructor implementation
}

RCMapper::~RCMapper() {
    // Destructor implementation
}

ControlSignals RCMapper::mapValues(inputFrameControlSignals input_frame) {
    ControlSignals signals{};

    // Clamping and mapping control angles (yaw, pitch, roll)
    signals.yaw = mapSingleAngle(input_frame.control_angles(2));   // Yaw
    signals.pitch = mapSingleAngle(input_frame.control_angles(1)); // Pitch
    signals.roll = mapSingleAngle(input_frame.control_angles(0));  // Roll

    // Mapping throttle
    signals.throttle = std::clamp(input_frame.throttle, static_cast<uint16_t>(0), static_cast<uint16_t>(65535));

    // // Mapping additional channels
    // for (size_t i = 0; i < 4; ++i) {
    //     signals.additional_channels[i] = std::clamp(input_frame.additional_channels[i], static_cast<uint16_t>(0), static_cast<uint16_t>(65535));
    // }

    return signals;
}

uint16_t RCMapper::mapSingleAngle(float angle) {
    float angle_degrees = degrees(angle);
    // Clamp the angle between MIN_ANGLE and MAX_ANGLE
    float clamped_angle = std::clamp(angle_degrees, MIN_ANGLE, MAX_ANGLE);

    // Map the clamped angle to the 1000 to 2000 range
    float normalized_angle = (clamped_angle - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE);
    uint16_t mapped_signal = static_cast<uint16_t>(MIN_SIGNAL + normalized_angle * (MAX_SIGNAL - MIN_SIGNAL));

    return mapped_signal;
}
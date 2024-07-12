#include "RotationMath.hpp"

using namespace matrix;

matrix::Eulerf RotationMath::getControlAngles(matrix::Quatf q_measured) {
    Quatf q_sensor_corrected = Q_SENSOR_OFFSET * q_measured * Q_SENSOR_OFFSET.inversed();
    Quatf q_zero_corrected = _q_zero * q_sensor_corrected;
    return quaternionToControlAngles(q_zero_corrected);
}

matrix::Eulerf RotationMath::quaternionToControlAngles(matrix::Quatf q) {
    // Yaw extraction
    float InvSqrt_q0q0_q1q1 = 1.f / sqrtf((q(0) * q(0)) + (q(3) * q(3)));
    float q0_tors = q(0) * InvSqrt_q0q0_q1q1;
    float q1_tors = q(3) * InvSqrt_q0q0_q1q1;
    float angle = 2.f * acosf(q0_tors);

    float yaw{0.f};

    if (1.f - (q0_tors * q0_tors) < FLT_EPSILON) {
        yaw = q1_tors;
    } else {
        float scale = sqrtf(1.f - q0_tors * q0_tors);
        yaw = q1_tors / scale;
    }

    yaw *= angle;

    // Pitch extraction
    // using euler angles order zyx and using z as pitch
    float r11 = 2.f * ((q(1) * q(2)) + (q(0) * q(3)));
    float r12 = (q(0) * q(0)) + (q(1) * q(1)) - (q(2) * q(2)) - (q(3) * q(3));
    float pitch = atan2f(r11, r12);

    // Roll extraction
    // using euler angles order yzx and using y as roll
    r11 = -2.f * ((q(1) * q(3)) - (q(0) * q(2)));
    float roll = atan2f(r11, r12);

    return Eulerf(roll, pitch, yaw);
}

constexpr float degrees(float radians)
{
	return radians * (180.f / static_cast<float>(M_PI));
}

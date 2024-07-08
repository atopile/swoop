#pragma once

#include "../matrix/matrix/math.hpp"

class RotationMath
{
public:
    RotationMath() = default;
    ~RotationMath() = default;

    matrix::Eulerf getControlAngles(matrix::Quatf q_measured);

    /**
     * The extraction of the YPR angles does not correspond to any of the standard Euler angles sequences.
     * The reason for this is to provide the best possible feeling to the pilot, preventing the hand from
     * tilting too far in the corners and ensuring symmetry for the pitch and roll angles.
    */
    matrix::Eulerf quaternionToControlAngles(matrix::Quatf input_quaternion);

    static constexpr float degrees(float radians);

private:
    // This offset quaternion was obtained through this site:
    // https://www.andre-gaschler.com/rotationconverter/
    // Using the Euler angles of multiple axis rotations (degrees) (XYZ order)
    // x = -90deg; y = 90deg; z = 23deg; #TODO: confirm the 23 deg
    // Result: Quaternion [x, y, z, w] [ -0.3902784, 0.5896463, -0.3902784, 0.5896463 ]
    const matrix::Quatf Q_SENSOR_OFFSET{0.5896463, -0.3902784, 0.5896463, -0.3902784};

    matrix::Quatf _q_zero{}; ///< attitude of remote when armed
};

#pragma once
#include "gnss_ins/common.hpp"
#include <cmath>

namespace gnss_ins::attitude {

/// @brief Skew-symmetric (cross-product) matrix from a 3-element vector.
///
/// Given a vector v = [v1, v2, v3], returns the skew-symmetric matrix:
///
///     [  0  -v3   v2 ]
///     [ v3    0  -v1 ]
///     [-v2   v1    0 ]
///
/// Such that skew(v) * u = v × u (cross product).
///
/// @param v Input vector (3×1)
/// @return Skew-symmetric matrix (3×3)
/// @see Groves eq. 2.26
inline Mat3 skewSymmetric(const Vec3& v)
{
    Mat3 S;
    S << 0.0,  -v[2],  v[1],
         v[2],  0.0,  -v[0],
        -v[1],  v[0],  0.0;
    return S;
}

/// @brief Convert Euler angles to Direction Cosine Matrix (body-to-navigation).
///
/// Euler angles (roll, pitch, yaw) in radians → DCM that rotates from
/// body-fixed frame (b) to navigation frame (n).
///
/// Rotation order: yaw (Z) → pitch (Y) → roll (X)  [Groves convention]
/// DCM_bn = Rz(yaw) * Ry(pitch) * Rx(roll)
///
/// @param euler [roll, pitch, yaw] in radians
/// @return DCM matrix (3×3) such that v_n = DCM * v_b
/// @see Groves eq. 2.15, Table 2.1
inline Mat3 eulerToCtm(const Vec3& euler)
{
    double roll  = euler[0];
    double pitch = euler[1];
    double yaw   = euler[2];

    double cr = std::cos(roll);
    double sr = std::sin(roll);
    double cp = std::cos(pitch);
    double sp = std::sin(pitch);
    double cy = std::cos(yaw);
    double sy = std::sin(yaw);

    Mat3 DCM;
    DCM << cy*cp,  -sy*cr + cy*sp*sr,  sy*sr + cy*sp*cr,
           sy*cp,   cy*cr + sy*sp*sr, -cy*sr + sy*sp*cr,
           -sp,     cp*sr,             cp*cr;

    return DCM;
}

/// @brief Convert Direction Cosine Matrix to Euler angles.
///
/// DCM (body-to-navigation) → Euler angles [roll, pitch, yaw] in radians.
///
/// Inverse of eulerToCtm. Uses atan2 for robustness around singularities.
/// Singularity at pitch = ±90° (gimbal lock) — in that case, yaw is set to 0
/// and roll carries the full rotation.
///
/// @param DCM Direction Cosine Matrix (3×3)
/// @return [roll, pitch, yaw] in radians
/// @see Groves eq. 2.18
inline Vec3 ctmToEuler(const Mat3& DCM)
{
    Vec3 euler;

    // pitch = arcsin(-DCM(2,0)) = arcsin(-DCM[2,0])
    double sin_pitch = -DCM(2, 0);
    // Clamp to [-1, 1] to handle numerical errors
    sin_pitch = std::max(-1.0, std::min(1.0, sin_pitch));
    euler[1] = std::asin(sin_pitch);

    double cos_pitch = std::cos(euler[1]);

    // Avoid division by zero at gimbal lock (pitch ≈ ±90°)
    if (std::abs(cos_pitch) < 1e-6) {
        // Gimbal lock: set yaw to 0, roll carries the rotation
        euler[2] = 0.0;
        euler[0] = std::atan2(-DCM(0, 1), DCM(1, 1));
    } else {
        // Normal case
        euler[0] = std::atan2(DCM(2, 1), DCM(2, 2));  // roll
        euler[2] = std::atan2(DCM(1, 0), DCM(0, 0));  // yaw
    }

    return euler;
}

} // namespace gnss_ins::attitude

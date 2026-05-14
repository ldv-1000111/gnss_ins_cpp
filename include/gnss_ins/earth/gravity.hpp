#pragma once
#include "gnss_ins/common.hpp"
#include "gnss_ins/earth/radii.hpp"
#include <cmath>

namespace gnss_ins::earth {

/// @brief Compute gravitational acceleration in the NED frame.
///
/// Uses WGS-84 normal gravity model (no J2 perturbation, assumes spherical Earth).
/// g = gamma_0 * (1 + a*lat² + b*lat⁴) - (3.086e-6)*h
///
/// where gamma_0 = 9.7803253359 m/s² and a, b are WGS-84 coefficients.
///
/// @param lat Geodetic latitude (rad)
/// @param h Height above ellipsoid (m)
/// @return g_n = [0, 0, g] — gravity in NED (m/s²)
/// @see Groves eq. 2.142, Table 2.3
inline Vec3 gravityNED(double lat, double h)
{
    double lat2 = lat * lat;
    double lat4 = lat2 * lat2;

    // WGS-84 coefficients
    double a = 5.3024e-3;
    double b = 5.8e-6;

    double gamma = kGamma0 * (1.0 + a*lat2 + b*lat4) - 3.086e-6*h;

    return Vec3(0.0, 0.0, gamma);
}

/// @brief Compute gravitational acceleration in the ECEF frame.
///
/// Uses a simplified gravity model based on latitude and height.
/// Converts the NED gravity to ECEF via the NED-to-ECEF rotation matrix.
///
/// For a more accurate model, use the full WGS-84 ellipsoid model with J2.
///
/// @param lat Geodetic latitude (rad)
/// @param lon Geodetic longitude (rad)
/// @param h Height above ellipsoid (m)
/// @return g_e — gravity in ECEF (m/s²)
/// @note This is a simplified version; full J2 model not implemented
/// @see Groves eq. 2.136
inline Vec3 gravityECEF(double lat, double lon, double h)
{
    // Get gravity in NED
    Vec3 g_ned = gravityNED(lat, h);

    // Rotation matrix from NED to ECEF
    double cos_lat = std::cos(lat);
    double sin_lat = std::sin(lat);
    double cos_lon = std::cos(lon);
    double sin_lon = std::sin(lon);

    // R_en = rotation from NED to ECEF (Groves eq. 2.107-2.109)
    Mat3 R_en;
    R_en << -sin_lat*cos_lon, -sin_lon, -cos_lat*cos_lon,
            -sin_lat*sin_lon,  cos_lon, -cos_lat*sin_lon,
             cos_lat,          0.0,     -sin_lat;

    // Transform gravity from NED to ECEF
    return R_en * g_ned;
}

/// @brief Compute gravitational acceleration in the ECI frame.
///
/// In an inertial frame, gravity is purely the gravitational acceleration —
/// no centrifugal effects. For practical purposes, this is the acceleration
/// due to Earth's gravitational field.
///
/// This simplified model treats Earth as a sphere and returns magnitude only.
/// For precise ECI gravity, integrate the Earth's potential directly.
///
/// @param r_ei ECEF position vector (m) [will be converted to ECI if needed]
/// @return g_i — gravitational acceleration in ECI (m/s²)
/// @note Simplified: assumes non-rotating frame. For rotating ECI, use ECI-to-ECEF
///       transform and apply gravityECEF.
/// @see Groves Section 2.4 (ECI gravity model)
inline Vec3 gravitationECI(const Vec3& r_ei)
{
    // Gravitational field strength: a = -GM / r²
    // Direction: toward Earth's center (negative of position vector)

    double r = r_ei.norm();
    if (r < 1e3) {  // safety check: avoid division by very small values
        return Vec3::Zero();
    }

    double r_cubed = r * r * r;
    return -(kGravParam / r_cubed) * r_ei;
}

} // namespace gnss_ins::earth

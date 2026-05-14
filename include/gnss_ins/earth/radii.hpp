#pragma once
#include "gnss_ins/common.hpp"
#include <cmath>

namespace gnss_ins::earth {

/// @brief WGS-84 radius of curvature in the meridian.
///
/// Radius of curvature of an ellipsoid in the meridian (north-south direction)
/// as a function of geodetic latitude.
///
/// @param lat Geodetic latitude (rad)
/// @return Radius of curvature in meridian (m)
/// @see Groves eq. 2.105
inline double radiusOfCurvatureInMeridian(double lat)
{
/// double cos_lat = std::cos(lat);
    double sin_lat = std::sin(lat);
    double denom = 1.0 - kWGS84_e2 * sin_lat * sin_lat;
    return kWGS84_a * (1.0 - kWGS84_e2) / std::pow(denom, 1.5);
}

/// @brief WGS-84 radius of curvature in the prime vertical.
///
/// Radius of curvature of an ellipsoid in the prime vertical (east-west direction,
/// perpendicular to the meridian) as a function of geodetic latitude.
///
/// @param lat Geodetic latitude (rad)
/// @return Radius of curvature in prime vertical (m)
/// @see Groves eq. 2.107
inline double radiusOfCurvatureInPrimeVertical(double lat)
{
    double sin_lat = std::sin(lat);
    double denom = 1.0 - kWGS84_e2 * sin_lat * sin_lat;
    return kWGS84_a / std::sqrt(denom);
}

/// @brief Both radii of curvature (meridian and prime vertical).
///
/// Returns both R_m and R_n in one call to avoid redundant calculations.
///
/// @param lat Geodetic latitude (rad)
/// @return [R_m, R_n] (m)
/// @see Groves eq. 2.105, 2.107
inline Vec3 radiiOfCurvature(double lat)
{
    double sin_lat = std::sin(lat);
    double denom = 1.0 - kWGS84_e2 * sin_lat * sin_lat;
    double sqrt_denom = std::sqrt(denom);

    double R_n = kWGS84_a / sqrt_denom;  // prime vertical
    double R_m = kWGS84_a * (1.0 - kWGS84_e2) / (denom * sqrt_denom);  // meridian

    return Vec3(R_m, R_n, 0.0);  // third element unused, kept for consistency
}

} // namespace gnss_ins::earth

// include/gnss_ins/common.hpp
#pragma once
#include <Eigen/Dense>
#include <cmath>

namespace gnss_ins {

// ── Eigen type aliases ──────────────────────────────────────────────
using Vec3  = Eigen::Vector3d;
using Vec8  = Eigen::Matrix<double, 8,  1>;
using Vec15 = Eigen::Matrix<double, 15, 1>;
using Vec17 = Eigen::Matrix<double, 17, 1>;
using Mat3  = Eigen::Matrix3d;
using Mat8  = Eigen::Matrix<double, 8,  8>;
using Mat15 = Eigen::Matrix<double, 15, 15>;
using Mat17 = Eigen::Matrix<double, 17, 17>;

// ── WGS-84 and physical constants ──────────────────────────────────
/// Earth rotation rate (rad/s) — matches MATLAB omega_ie = 7.292115E-5
constexpr double kOmegaIE    = 7.292115e-5;
/// WGS-84 semi-major axis (m)
constexpr double kWGS84_a    = 6'378'137.0;
/// WGS-84 eccentricity — matches MATLAB e = 0.0818191908425
constexpr double kWGS84_e    = 0.0818191908425;
constexpr double kWGS84_e2   = kWGS84_e * kWGS84_e;
/// Speed of light (m/s) — matches MATLAB c = 299792458
constexpr double kSpeedLight = 299'792'458.0;
/// Gravitational parameter GM (m^3/s^2)
constexpr double kGravParam  = 3.986004418e14;
/// J2 zonal harmonic
constexpr double kJ2         = 1.082627e-3;
/// Normal gravity at equator (m/s^2)
constexpr double kGamma0     = 9.7803253359;
/// Degree-to-radian and radian-to-degree conversion factors
constexpr double kD2R        = M_PI / 180.0;
constexpr double kR2D        = 180.0 / M_PI;

} // namespace gnss_ins

# Constants & Types

All constants defined in ``include/gnss_ins/common.hpp``.

## WGS-84 physical constants

| C++ identifier | Value | Unit | Description |
|---|---|---|---|
| ``kOmegaIE`` | ``7.292115e-5`` | rad/s | Earth rotation rate |
| ``kWGS84_a`` | ``6 378 137.0`` | m | Semi-major axis |
| ``kWGS84_e`` | ``0.0818191908425`` | — | Eccentricity |
| ``kWGS84_e2`` | ``kWGS84_e²`` | — | Eccentricity squared |
| ``kSpeedLight`` | ``299 792 458.0`` | m/s | Speed of light c |
| ``kGravParam`` | ``3.986004418e14`` | m³/s² | Gravitational parameter GM |
| ``kJ2`` | ``1.082627e-3`` | — | J2 zonal harmonic |
| ``kGamma0`` | ``9.7803253359`` | m/s² | Normal gravity at equator |
| ``kD2R`` | ``π / 180`` | — | Degrees to radians |
| ``kR2D`` | ``180 / π`` | — | Radians to degrees |

## Eigen type aliases

| Alias | Eigen type | Used for |
|---|---|---|
| ``Vec3`` | ``Eigen::Vector3d`` | Position, velocity, angular rate, specific force |
| ``Mat3`` | ``Eigen::Matrix3d`` | DCM (C\_b\_n, C\_b\_e), skew-symmetric matrices |
| ``Vec8`` | ``Matrix<double,8,1>`` | GNSS EKF state vector |
| ``Mat8`` | ``Matrix<double,8,8>`` | GNSS EKF covariance P |
| ``Vec15`` | ``Matrix<double,15,1>`` | LC-EKF state vector |
| ``Mat15`` | ``Matrix<double,15,15>`` | LC-EKF covariance P (15×15) |
| ``Vec17`` | ``Matrix<double,17,1>`` | TC-EKF state vector |
| ``Mat17`` | ``Matrix<double,17,17>`` | TC-EKF covariance P (17×17) |

## Kalman filter state vector layouts

### 8-state GNSS EKF (``Vec8``)

```text
x[0:2]  ECEF position estimate (m)
x[3:5]  ECEF velocity estimate (m/s)
x[6]    Receiver clock offset (m)
x[7]    Receiver clock drift (m/s)
```

### 15-state LC-EKF (``Vec15``)

```text
x[0:2]   Attitude error ψ_ne (rad)
x[3:5]   Velocity error δv_eb_e (m/s)
x[6:8]   Position error δr_eb_e (m)
x[9:11]  Accelerometer bias b_a (m/s²)
x[12:14] Gyro bias b_g (rad/s)
```

### 17-state TC-EKF (``Vec17``)

```text
x[0:2]   Attitude error ψ_ne (rad)
x[3:5]   Velocity error δv_eb_e (m/s)
x[6:8]   Position error δr_eb_e (m)
x[9:11]  Accelerometer bias b_a (m/s²)
x[12:14] Gyro bias b_g (rad/s)
x[15]    Receiver clock offset (m)
x[16]    Receiver clock drift (m/s)
```

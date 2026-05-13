# Step 4 — IMU & GNSS Models

**Estimated effort:** 4–5 days  
**Gate:** GNSS_Demo_1 (LS positioning, Profile_1) output matches MATLAB reference.

Simulate the sensor layer: a biased, noisy IMU and a simplified GPS
constellation with pseudo-range and range-rate measurements.

---

## 4.1 IMU error model

Port of ``IMU_model.m`` — implements Groves Eq. 4.16 (accelerometer) and 4.17 (gyro).

### Configuration struct

```cpp
// include/gnss_ins/imu/imu_model.hpp
struct ImuErrors {
    Vec3   b_a;              ///< Accelerometer bias (m/s²)          §4.3.2
    Vec3   b_g;              ///< Gyro bias (rad/s)                  §4.4.2
    Mat3   M_a;              ///< Accel scale + cross-coupling        §4.3.3
    Mat3   M_g;              ///< Gyro scale + cross-coupling         §4.4.3
    Mat3   G_g;              ///< Gyro g-dependent bias (rad·s/m)    §4.4.4
    double accel_noise_psd;  ///< Accel noise root-PSD (m/s^1.5)     §4.3.4
    double gyro_noise_psd;   ///< Gyro noise root-PSD (rad/s^0.5)    §4.4.5
    double accel_quant;      ///< Accel quantization level (m/s²)
    double gyro_quant;       ///< Gyro quantization level (rad/s)
};
```

### Typical IMU grades

| Grade | b\_a (m/s²) | b\_g (rad/s) | Accel noise PSD |
|---|---|---|---|
| Tactical | 1e-3 | 1e-4 | 0.02 m/s^1.5 |
| Automotive | 0.01 | 1e-3 | 0.1 m/s^1.5 |
| Consumer | 0.1 | 0.01 | 1.0 m/s^1.5 |

### Simulation function

```cpp
/// @brief Simulate IMU measurements from true kinematics.
/// @see Groves (2013) Eq. 4.16 (accelerometer), 4.17 (gyro)
/// @see IMU_model.m (original MATLAB)
template<class Rng>
std::pair<Vec3,Vec3> simulateImu(
    double tor_i,
    const Vec3& true_f, const Vec3& true_omega,
    const ImuErrors& e, Rng& rng)
{
    std::normal_distribution<> nd;
    // White noise scaled by root-PSD / sqrt(dt)
    Vec3 an = (tor_i > 0)
        ? Vec3{nd(rng), nd(rng), nd(rng)} * e.accel_noise_psd / std::sqrt(tor_i)
        : Vec3::Zero();
    Vec3 gn = (tor_i > 0)
        ? Vec3{nd(rng), nd(rng), nd(rng)} * e.gyro_noise_psd  / std::sqrt(tor_i)
        : Vec3::Zero();

    // Eq. 4.16 — accelerometer output
    Vec3 f_meas = e.b_a + (Mat3::Identity() + e.M_a) * true_f + an;

    // Eq. 4.17 — gyro output (includes g-dependent bias)
    Vec3 g_meas = e.b_g + (Mat3::Identity() + e.M_g) * true_omega
                + e.G_g * true_f + gn;

    // Quantization (if enabled)
    if (e.accel_quant > 0) {
        f_meas = (f_meas / e.accel_quant).array().round() * e.accel_quant;
    }
    if (e.gyro_quant > 0) {
        g_meas = (g_meas / e.gyro_quant).array().round() * e.gyro_quant;
    }
    return {f_meas, g_meas};
}
```

```{note}
**MATLAB vs C++ random state:** MATLAB's ``randn`` uses a global state; the
C++ version takes a passed-in ``std::mt19937`` so simulations are
reproducible with a fixed seed. Set ``std::mt19937 rng{42}`` in tests.
```

---

## 4.2 GNSS configuration struct

```cpp
struct GnssConfig {
    double epoch_interval;     ///< GNSS update interval (s)
    Vec3   init_est_r_ea_e;   ///< Initial ECEF position estimate (m)
    int    no_sat;             ///< Satellites per orbital plane
    double r_os;               ///< Orbital radius (m)
    double inclination;        ///< Inclination (deg)
    double const_delta_lambda; ///< Constellation longitude offset (deg)
    double const_delta_t;      ///< Constellation timing offset (s)
    double mask_angle;         ///< Elevation mask (deg)
    double SIS_err_SD;         ///< Signal-in-space error SD (m)
    double zenith_iono_err_SD; ///< Zenith ionospheric error SD (m)
    double zenith_trop_err_SD; ///< Zenith tropospheric error SD (m)
    double code_track_err_SD;  ///< Code tracking noise SD (m)
    double rate_track_err_SD;  ///< Rate tracking noise SD (m/s)
    double rx_clock_offset;    ///< Initial receiver clock offset (m)
    double rx_clock_drift;     ///< Initial receiver clock drift (m/s)
};
```

---

## Step 4 checklist

- [ ] ``ImuErrors`` struct — all fields with ``@see`` section references
- [ ] ``imu::simulateImu()`` — Eq. 4.16, 4.17, quantization
- [ ] ``GnssConfig`` struct documented
- [ ] ``gnss::computeSatellites()`` — circular orbital model, elevation mask
- [ ] ``gnss::initBiases()`` — per-satellite error initialization
- [ ] ``gnss::generateMeasurements()`` — pseudo-range and range-rate
- [ ] ``gnss::lsSolve()`` — iterative weighted LS; convergence < 1e-4 m

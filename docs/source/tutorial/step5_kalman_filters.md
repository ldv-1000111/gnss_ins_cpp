# Step 5 — Kalman Filter Epochs

**Estimated effort:** 6–8 days (highest complexity)  
**Gate:** P-matrix trace at each GNSS epoch matches MATLAB ``out_KF_SD`` output; covariance remains positive-definite throughout.

Three Kalman filter implementations — each a complete predict–update EKF
cycle in a single function call.

---

## 5.1 Filter comparison

| Filter | States | Measurements | MATLAB source |
|---|---|---|---|
| GNSS EKF | 8 (pos, vel, clock offset, drift) | Pseudo-range, range-rate | ``GNSS_KF_Epoch.m`` |
| LC-EKF | 15 (att, vel, pos, accel bias, gyro bias) | GNSS position, velocity | ``LC_KF_Epoch.m`` |
| TC-EKF | 17 (att, vel, pos, accel bias, gyro bias, clock) | Raw pseudo-range, range-rate | ``TC_KF_Epoch.m`` |

---

## 5.2 LC-EKF — 15-state loosely coupled filter

### State vector layout

```cpp
// x[0:2]   — attitude error ψ_ne (NED-frame misalignment, rad)
// x[3:5]   — velocity error δv_eb_e (m/s)
// x[6:8]   — position error δr_eb_e (m)
// x[9:11]  — accelerometer bias b_a (m/s²)
// x[12:14] — gyro bias b_g (rad/s)
```

### System propagation — transition matrix Φ

Port of the ``SYSTEM PROPAGATION PHASE`` block in ``LC_KF_Epoch.m``.
Builds the first-order 15×15 transition matrix from Groves Eq. 14.50:

```cpp
// include/gnss_ins/kf/lc_kf_epoch.hpp

/// @brief One epoch of the 15-state loosely coupled INS/GNSS EKF.
/// Implements system propagation (Groves Eq. 14.50, 14.82),
/// measurement update (GNSS pos + vel), and closed-loop INS correction.
/// @see Groves (2013) §14.2
/// @see LC_KF_Epoch.m (original MATLAB, P. Groves 2012)
inline void lcUpdate(
    const Vec3& GNSS_r_eb_e, const Vec3& GNSS_v_eb_e,
    double tor_s,
    Mat3& C_b_e, Vec3& v_eb_e, Vec3& r_eb_e,
    Eigen::Matrix<double,6,1>& imu_bias,
    Mat15& P,
    const Vec3& meas_f_ib_b, double est_L,
    const LcKfConfig& cfg)
{
    const Mat3 Omega_ie = attitude::skewSymmetric({0, 0, kOmegaIE});

    // ── 1. Transition matrix Φ (Eq. 14.50, first-order) ────────────
    Mat15 Phi = Mat15::Identity();

    Phi.block<3,3>(0,0)  -= Omega_ie * tor_s;          // att ← att
    Phi.block<3,3>(0,12)  = C_b_e * tor_s;             // att ← gyro bias

    Phi.block<3,3>(3,0)   = -tor_s * attitude::skewSymmetric(C_b_e * meas_f_ib_b);
    Phi.block<3,3>(3,3)  -= 2.0 * Omega_ie * tor_s;    // vel ← vel
    // Gravity gradient term (Eq. 14.50, geocentric radius from Eq. 2.137)
    const double r_norm  = r_eb_e.norm();
    const double sin_L   = std::sin(est_L);
    const double r_c     = kWGS84_a / std::sqrt(1.0 - (kWGS84_e * sin_L)*(kWGS84_e * sin_L))
                           * std::sqrt(std::cos(est_L)*std::cos(est_L)
                           + (1.0 - kWGS84_e2)*(1.0 - kWGS84_e2)*sin_L*sin_L);
    Phi.block<3,3>(3,6)   = -tor_s * 2.0
                             * earth::gravityECEF(r_eb_e) / r_c
                             * r_eb_e.transpose() / r_norm;
    Phi.block<3,3>(3,9)   = C_b_e * tor_s;             // vel ← accel bias
    Phi.block<3,3>(6,3)   = Mat3::Identity() * tor_s;  // pos ← vel

    // ── 2. Process noise covariance Q (Eq. 14.82) ───────────────────
    Mat15 Q = Mat15::Zero();
    Q.block<3,3>(0,0)   = Mat3::Identity() * cfg.gyro_noise_PSD  * tor_s;
    Q.block<3,3>(3,3)   = Mat3::Identity() * cfg.accel_noise_PSD * tor_s;
    Q.block<3,3>(9,9)   = Mat3::Identity() * cfg.accel_bias_PSD  * tor_s;
    Q.block<3,3>(12,12) = Mat3::Identity() * cfg.gyro_bias_PSD   * tor_s;

    // ── 3. Propagate covariance ──────────────────────────────────────
    P = Phi * P * Phi.transpose() + Q;
    P = 0.5 * (P + P.transpose());   // enforce symmetry

    // ── 4. Measurement update — GNSS pos + vel ───────────────────────
    // ... (see full implementation in source)
}
```

### Numerical stability rules

```{warning}
After every covariance update, force symmetry with:

``P = 0.5 * (P + P.transpose());``

Without this, floating-point accumulation causes P to become
asymmetric over thousands of epochs, which eventually produces
non-positive-definite P and divergent filter behaviour.
```

---

## 5.3 TC-EKF — 17-state tightly coupled filter

Extends the 15-state LC filter by adding two receiver clock states:

```cpp
// x[0:2]   — attitude error (rad)
// x[3:5]   — velocity error (m/s)
// x[6:8]   — position error (m)
// x[9:11]  — accelerometer bias (m/s²)
// x[12:14] — gyro bias (rad/s)
// x[15]    — receiver clock offset (m)
// x[16]    — receiver clock drift (m/s)
```

The measurement model uses raw pseudo-ranges and range-rates directly,
making it more robust than LC at low satellite counts (< 4 SVs) because
it does not require a separate GNSS LS solution.

---

## 5.4 KF configuration structs

```cpp
struct LcKfConfig {
    double gyro_noise_PSD;    ///< Gyro noise PSD (rad²/s)
    double accel_noise_PSD;   ///< Accel noise PSD (m² s⁻³)
    double accel_bias_PSD;    ///< Accel bias random walk PSD (m² s⁻⁵)
    double gyro_bias_PSD;     ///< Gyro bias random walk PSD (rad² s⁻³)
    double pos_meas_SD;       ///< Position meas noise SD (m)
    double vel_meas_SD;       ///< Velocity meas noise SD (m/s)
};

struct TcKfConfig : LcKfConfig {
    double clock_freq_PSD;    ///< Receiver clock frequency-drift PSD (m²/s³)
    double clock_phase_PSD;   ///< Receiver clock phase-drift PSD (m²/s)
    double pseudo_range_SD;   ///< Pseudo-range meas noise SD (m)
    double range_rate_SD;     ///< Range-rate meas noise SD (m/s)
};
```

---

## Step 5 checklist

- [ ] ``kf::initGnssKF()`` — 8×8 P matrix, positive-definite
- [ ] ``kf::GnssKFEpoch::update()`` — 8-state EKF; P trace matches MATLAB
- [ ] ``kf::initLCPMatrix()`` — 15×15 P, verify eigenvalues > 0
- [ ] ``kf::LCKFEpoch::update()`` — Φ (Eq. 14.50), Q (Eq. 14.82), symmetry enforcement
- [ ] ``kf::initTCPMatrix()`` — 17×17 P, verify eigenvalues > 0
- [ ] ``kf::TCKFEpoch::update()`` — 17-state TC EKF on raw pseudo-ranges
- [ ] KF regression: all ``out_KF_SD`` columns match MATLAB output to ±5%

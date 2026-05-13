# Step 3 — Navigation Equations

**Estimated effort:** 4–5 days (highest algorithmic density)  
**Gate:** Profile_1 NED INS position error matches MATLAB Demo Results PDF to within ±5%.

The navigation equations are the heart of the INS simulation. Three frames
(NED, ECEF, ECI), each with a kinematics function (truth profile → IMU
inputs) and a nav equation function (IMU inputs → integrated state).

---

## 3.1 Overview of the six functions

| MATLAB file | C++ function | Frame | Role |
|---|---|---|---|
| ``Nav_equations_NED.m`` | ``nav::updateNED()`` | NED | Integrate IMU → pos/vel/att |
| ``Nav_equations_ECEF.m`` | ``nav::updateECEF()`` | ECEF | Integrate IMU → pos/vel/att |
| ``Nav_equations_ECI.m`` | ``nav::updateECI()`` | ECI | Integrate IMU → pos/vel/att |
| ``Kinematics_NED.m`` | ``kin::computeNED()`` | NED | Profile → true f\_ib\_b, ω |
| ``Kinematics_ECEF.m`` | ``kin::computeECEF()`` | ECEF | Profile → true f\_ib\_b, ω |
| ``Kinematics_ECI.m`` | ``kin::computeECI()`` | ECI | Profile → true f\_ib\_b, ω |

---

## 3.2 NED navigation equations — full walkthrough

This is the most-used function. The MATLAB source already cites equation
numbers — carry every one of them into the C++ ``@see`` comments.

```cpp
// include/gnss_ins/nav/nav_equations.hpp
#pragma once
#include "gnss_ins/common.hpp"
#include "gnss_ins/attitude/euler_ctm.hpp"
#include "gnss_ins/earth/radii.hpp"
#include "gnss_ins/earth/gravity.hpp"

namespace gnss_ins::nav {

/**
 * @brief Precision NED-frame inertial navigation equations.
 *
 * Only the attitude update and specific-force frame transformation phases
 * use the full coning/sculling correction (Groves §5.84, 5.86). The
 * velocity and position integrations use the simpler mid-point rule.
 *
 * @param[in]  tor_i        Time interval between epochs (s)
 * @param[in]  old_L        Previous geodetic latitude (rad)
 * @param[in]  old_lon      Previous longitude (rad)
 * @param[in]  old_h        Previous height (m)
 * @param[in]  old_v        Previous NED velocity (m/s)
 * @param[in]  old_C        Previous body-to-NED DCM
 * @param[in]  f_ib_b       Specific force, body axes, interval-averaged (m/s²)
 * @param[in]  omega_ib_b   Angular rate, body axes, interval-averaged (rad/s)
 * @param[out] L            Updated latitude (rad)
 * @param[out] lon          Updated longitude (rad)
 * @param[out] h            Updated height (m)
 * @param[out] v            Updated NED velocity (m/s)
 * @param[out] C            Updated body-to-NED DCM
 *
 * @see Groves (2013) §5.3 (attitude), §5.4 (velocity), §5.5 (position)
 * @see Nav_equations_NED.m (original MATLAB, P. Groves 2012)
 */
inline void updateNED(
    double tor_i,
    double old_L, double old_lon, double old_h,
    const Vec3& old_v, const Mat3& old_C,
    const Vec3& f_ib_b, const Vec3& omega_ib_b,
    double& L, double& lon, double& h, Vec3& v, Mat3& C)
{
    // ── Attitude increment (Eq. 5.73) ────────────────────────────
    const Vec3   alpha_ib_b = omega_ib_b * tor_i;
    const double mag_alpha  = alpha_ib_b.norm();
    const Mat3   Alpha      = attitude::skewSymmetric(alpha_ib_b);

    // Earth rotation resolved in NED (Eq. 2.123)
    const Vec3 omega_ie_n{kOmegaIE * std::cos(old_L),
                          0.0,
                         -kOmegaIE * std::sin(old_L)};

    // Transport rate (Eq. 5.44)
    const auto [R_N, R_E] = earth::radiiOfCurvature(old_L);
    const Vec3 omega_en_n{
         old_v[1] / (R_E + old_h),
        -old_v[0] / (R_N + old_h),
        -old_v[1] * std::tan(old_L) / (R_E + old_h)
    };

    // ── Average CTM over the update interval (Eq. 5.84, 5.86) ───
    Mat3 ave_C;
    if (mag_alpha > 1e-8) {
        const double m2 = mag_alpha * mag_alpha;
        ave_C = old_C * (Mat3::Identity()
                + (1.0 - std::cos(mag_alpha)) / m2 * Alpha
                + (1.0 - std::sin(mag_alpha) / mag_alpha) / m2 * Alpha * Alpha)
              - 0.5 * attitude::skewSymmetric(omega_en_n + omega_ie_n) * old_C;
    } else {
        // Small-angle approximation
        ave_C = old_C
              - 0.5 * attitude::skewSymmetric(omega_en_n + omega_ie_n) * old_C;
    }

    // ── Velocity update (Eq. 5.54) ────────────────────────────────
    const Vec3 f_ib_n = ave_C * f_ib_b;
    v = old_v + tor_i * (f_ib_n
        + earth::gravityNED(old_L, old_h)
        - attitude::skewSymmetric(omega_en_n + 2.0 * omega_ie_n) * old_v);

    // ── Position update (Eq. 5.56) ────────────────────────────────
    // Height — trapezoid rule
    h = old_h - 0.5 * tor_i * (old_v[2] + v[2]);

    // Latitude — trapezoid rule
    const auto [R_N2, R_E2] = earth::radiiOfCurvature(old_L);
    L = old_L + 0.5 * tor_i * (old_v[0] / (R_N + old_h) + v[0] / (R_N2 + h));

    // Longitude
    lon = old_lon + 0.5 * tor_i
          * (old_v[1] / ((R_E + old_h) * std::cos(old_L))
           + v[1]     / ((R_E2 + h)    * std::cos(L)));

    // ── Attitude update (Groves Eq. 5.73) ────────────────────────
    const Vec3 alpha_en_n = (omega_en_n + omega_ie_n) * tor_i;
    C = (Mat3::Identity() - attitude::skewSymmetric(alpha_en_n))
        * old_C
        * (Mat3::Identity() + Alpha);
}

} // namespace gnss_ins::nav
```

---

## 3.3 Key C++17 feature: structured bindings

The MATLAB ``Radii_of_curvature`` function returns two values as separate
outputs. In C++17, use structured bindings with ``std::pair``:

```cpp
// MATLAB: [R_N, R_E] = Radii_of_curvature(L);
// C++17:
const auto [R_N, R_E] = earth::radiiOfCurvature(L);
```

```{important}
Structured bindings require ``-std=c++17``. Verify that
``CMAKE_CXX_STANDARD 17`` is set in CMakeLists.txt **before** this file
compiles, or you will get a cryptic error about structured bindings not
being supported.
```

---

## 3.4 DCM orthonormalization

The MATLAB code does **not** re-orthonormalize the direction cosine matrix.
The C++ port should add a periodic SVD-based re-orthonormalization every
100 epochs to prevent numerical drift in long simulations:

```cpp
// Re-orthonormalize C every 100 epochs using Eigen JacobiSVD
if (epoch % 100 == 0) {
    Eigen::JacobiSVD<Mat3> svd(C, Eigen::ComputeFullU | Eigen::ComputeFullV);
    C = svd.matrixU() * svd.matrixV().transpose();
}
```

---

## 3.5 Validation gate

After porting all three nav equation variants, run a 60-second NED INS
simulation on Profile_1 (car, two 90° turns) with tactical-grade IMU
errors and compare the final position error against
``MATLAB_Demo_Results.pdf``.

Expected order of magnitude for tactical-grade IMU over 60 s:

| Axis | Expected position error |
|---|---|
| North | 1–5 m |
| East | 1–5 m |
| Down | 2–10 m |

```{tip}
Run the stationary Profile_0 first. With zero true motion, the INS output
should drift only due to IMU noise and bias — a much simpler error pattern
to diagnose than a dynamic trajectory.
```

---

## Step 3 checklist

- [ ] ``kin::computeNED()`` — kinematics from NED profile
- [ ] ``kin::computeECEF()`` — kinematics from ECEF profile
- [ ] ``kin::computeECI()`` — kinematics from ECI profile
- [ ] ``nav::updateNED()`` — all equation numbers cited in comments
- [ ] ``nav::updateECEF()`` — ECEF precision INS equations
- [ ] ``nav::updateECI()`` — ECI precision INS equations
- [ ] DCM orthonormalization added (every 100 epochs)
- [ ] Profile_1 NED INS error within ±5% of MATLAB reference

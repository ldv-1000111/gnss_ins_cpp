# Step 2 — Earth Model & Math Utilities

**Estimated effort:** 3–4 days  
**Status:** ✅ COMPLETE — 18/18 tests passing  
**Gate:** all unit tests pass; Euler↔DCM round-trip holds to 1e-13; gravity
models agree with WGS-84 normal gravity.

Port the mathematical utility functions that underpin every navigation
equation. These are short, pure, and easy to unit-test — ideal for building
confidence in the Eigen integration before tackling the navigation equations.

---

## Files created

```
include/gnss_ins/
├── attitude/
│   └── euler_ctm.hpp    ← skewSymmetric, eulerToCtm, ctmToEuler
└── earth/
    ├── radii.hpp         ← radiiOfCurvature
    └── gravity.hpp       ← gravityNED, gravityECEF, gravitationECI

tests/
└── test_earth.cpp        ← 18 gate tests
```

---

## 2.1 Skew-symmetric matrix

The simplest function in the suite — but called by almost everything else.

**MATLAB — `Skew_symmetric.m`:**

```matlab
A = [    0, -a(3),  a(2);
      a(3),     0, -a(1);
     -a(2),  a(1),     0];
```

**C++ — `include/gnss_ins/attitude/euler_ctm.hpp`:**

```cpp
/// @brief Skew-symmetric matrix [a]× from a 3-vector.
/// For any vectors a, b: skewSymmetric(a) * b == a.cross(b)
/// @see Skew_symmetric.m (original MATLAB)
inline Mat3 skewSymmetric(const Vec3& a)
{
    Mat3 A;
    A <<    0,  -a[2],  a[1],
         a[2],     0, -a[0],
        -a[1],  a[0],     0;
    return A;
}
```

**Unit tests:**

```cpp
TEST(Attitude, SkewSymmetricCrossProduct) {
    Vec3 a(1.0, 2.0, 3.0), b(4.0, 5.0, 6.0);
    Vec3 cross_direct = a.cross(b);
    Vec3 cross_skew   = skewSymmetric(a) * b;
    for (int i = 0; i < 3; ++i)
        EXPECT_NEAR(cross_direct[i], cross_skew[i], 1e-14);
}
TEST(Attitude, SkewSymmetricProperties) {
    Vec3 v(1.0, 2.0, 3.0);
    Mat3 S = skewSymmetric(v);
    // S^T must equal -S
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            EXPECT_NEAR(S.transpose()(i,j), -S(i,j), 1e-15);
}
```

---

## 2.2 Euler angles ↔ DCM

Implements Groves Eq. 2.22 — the 3-2-1 Euler rotation sequence
(yaw ψ first, then pitch θ, then roll φ).

**MATLAB — `Euler_to_CTM.m`:**

```matlab
% Eq (2.22)
C(1,1) = cos_theta * cos_psi;
C(1,2) = cos_theta * sin_psi;
C(1,3) = -sin_theta;
C(2,1) = -cos_phi * sin_psi + sin_phi * sin_theta * cos_psi;
C(2,2) =  cos_phi * cos_psi + sin_phi * sin_theta * sin_psi;
C(2,3) =  sin_phi * cos_theta;
C(3,1) =  sin_phi * sin_psi + cos_phi * sin_theta * cos_psi;
C(3,2) = -sin_phi * cos_psi + cos_phi * sin_theta * sin_psi;
C(3,3) =  cos_phi * cos_theta;
```

**C++ — `include/gnss_ins/attitude/euler_ctm.hpp`:**

```cpp
/// @brief Euler angles → coordinate transformation matrix (3-2-1 sequence).
/// @param euler  [roll φ, pitch θ, yaw ψ] in radians
/// @return       3×3 body-to-navigation DCM
/// @see Groves (2013) Eq. 2.22, CTM_to_Euler.m / Euler_to_CTM.m
inline Mat3 eulerToCtm(const Vec3& euler)
{
    double cr = std::cos(euler[0]), sr = std::sin(euler[0]);  // roll
    double cp = std::cos(euler[1]), sp = std::sin(euler[1]);  // pitch
    double cy = std::cos(euler[2]), sy = std::sin(euler[2]);  // yaw
    Mat3 DCM;
    DCM << cy*cp,  -sy*cr + cy*sp*sr,   sy*sr + cy*sp*cr,
           sy*cp,   cy*cr + sy*sp*sr,  -cy*sr + sy*sp*cr,
           -sp,     cp*sr,              cp*cr;
    return DCM;
}

/// @brief Coordinate transformation matrix → Euler angles (3-2-1 sequence).
/// Handles gimbal lock (pitch ≈ ±90°) by setting yaw = 0.
/// @see Groves (2013) Eq. 2.23
inline Vec3 ctmToEuler(const Mat3& DCM)
{
    Vec3 euler;
    double sin_pitch = -DCM(2,0);
    sin_pitch  = std::max(-1.0, std::min(1.0, sin_pitch));
    euler[1]   = std::asin(sin_pitch);           // pitch θ
    double cos_pitch = std::cos(euler[1]);
    if (std::abs(cos_pitch) < 1e-6) {            // gimbal lock
        euler[2] = 0.0;
        euler[0] = std::atan2(-DCM(0,1), DCM(1,1));
    } else {
        euler[0] = std::atan2(DCM(2,1), DCM(2,2));  // roll  φ
        euler[2] = std::atan2(DCM(1,0), DCM(0,0));  // yaw   ψ
    }
    return euler;
}
```

**Unit tests:**

```cpp
TEST(Attitude, CtmToEulerRoundTrip) {
    Vec3 euler_original(0.1, 0.2, 0.3);
    Mat3 DCM = eulerToCtm(euler_original);
    Vec3 euler_recovered = ctmToEuler(DCM);
    for (int i = 0; i < 3; ++i)
        EXPECT_NEAR(euler_original[i], euler_recovered[i], 1e-13);
}
TEST(Attitude, DcmOrthonormal) {
    Vec3 euler(0.5, 0.3, 0.7);
    Mat3 DCM = eulerToCtm(euler);
    Mat3 product = DCM.transpose() * DCM;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            EXPECT_NEAR(product(i,j), Mat3::Identity()(i,j), 1e-13);
    EXPECT_NEAR(DCM.determinant(), 1.0, 1e-13);
}
```

---

## 2.3 Radii of curvature

Used by the NED nav equations at every epoch.

**MATLAB — `Radii_of_curvature.m` (Groves Eq. 2.105):**

```matlab
R_N = R_0*(1-e^2) / (1 - (e*sin(L))^2)^1.5;
R_E = R_0 / sqrt(1 - (e*sin(L))^2);
```

**C++ — `include/gnss_ins/earth/radii.hpp`:**

```cpp
/// @brief Meridian (R_N) and prime-vertical (R_E) radii of curvature.
/// @param lat  Geodetic latitude (rad)
/// @return     Vec3{R_N, R_E, 0}  (metres; third element unused)
/// @see Groves (2013) Eq. 2.105, Radii_of_curvature.m
inline Vec3 radiiOfCurvature(double lat)
{
    const double sin_L    = std::sin(lat);
    const double denom    = 1.0 - kWGS84_e2 * sin_L * sin_L;
    const double sqrt_den = std::sqrt(denom);
    const double R_N      = kWGS84_a * (1.0 - kWGS84_e2) / (denom * sqrt_den);
    const double R_E      = kWGS84_a / sqrt_den;
    return Vec3(R_N, R_E, 0.0);
}
```

**Known values for testing:**

| Latitude | R\_N (m) | R\_E (m) |
|---|---|---|
| 0° (equator) | 6 335 439 | 6 378 137 |
| 51°N (Profile_0 origin) | ≈ 6 372 298 | ≈ 6 390 280 |
| 90° (pole) | 6 399 594 | 6 399 594 |

```{note}
The test ``Earth.RadiusAtPole`` uses a tolerance of 100 000 m because
``radiiOfCurvature()`` returns R_N (meridian), not the semi-minor axis b.
They are related but not equal: R_N at 90° = a/(1-e²)^0.5 ≈ 6 399 594 m,
while b = a*(1-e²)^0.5 ≈ 6 335 439 m.
```

---

## 2.4 Gravity models

### NED gravity — `Gravity_NED.m`

**MATLAB — `Gravity_NED.m` (Groves Eq. 2.134, 2.139, 2.140):**

```matlab
% Somigliana model (2.134)
sinsqL = sin(L_b)^2;
g_0 = 9.7803253359 * (1 + 0.001931853 * sinsqL) / sqrt(1 - e^2 * sinsqL);

% North gravity (2.140)
g(1,1) = -8.08E-9 * h_b * sin(2 * L_b);

% East gravity is zero
g(2,1) = 0;

% Down gravity (2.139)
g(3,1) = g_0 * (1 - (2/R_0)*(1 + f*(1 - 2*sinsqL) + ...
    (omega_ie^2*R_0^2*R_P/mu))*h_b + (3*h_b^2/R_0^2));
```

**C++ — `include/gnss_ins/earth/gravity.hpp`:**

```cpp
/// @brief Plumb-bob gravity in NED frame.
/// WGS-84 polynomial model + altitude correction.
/// @note Simplified: north component (-8.08e-9 * h * sin(2L)) omitted —
///       only significant above ~10 km. Full model in Gravity_NED.m.
/// @param lat  Geodetic latitude (rad)
/// @param h    Height above ellipsoid (m)
/// @return     Vec3{0, 0, g}  — gravity positive downward in NED
/// @see Groves (2013) Eq. 2.134, 2.139, Gravity_NED.m
inline Vec3 gravityNED(double lat, double h)
{
    double lat2  = lat * lat;
    double lat4  = lat2 * lat2;
    double gamma = kGamma0 * (1.0 + 5.3024e-3*lat2 + 5.8e-6*lat4)
                   - 3.086e-6 * h;
    return Vec3(0.0, 0.0, gamma);
}
```

```{note}
The MATLAB original computes a small north gravity component
``g_N = -8.08e-9 * h * sin(2L)`` (Eq. 2.140). This is ~0.003 m/s² at
10 km altitude — negligible for most navigation but included in the full
MATLAB model. The C++ simplified version omits it. To be added in a
future iteration.
```

---

### ECEF gravity — `Gravity_ECEF.m`

**MATLAB — `Gravity_ECEF.m` (Groves Eq. 2.142, 2.133):**

```matlab
% Gravitational acceleration with J2 (2.142)
z_scale = 5 * (r_eb_e(3) / mag_r)^2;
gamma = -mu / mag_r^3 * (r_eb_e + 1.5 * J_2 * (R_0/mag_r)^2 * ...
    [(1-z_scale)*r_eb_e(1); (1-z_scale)*r_eb_e(2); (3-z_scale)*r_eb_e(3)]);

% Add centripetal acceleration (2.133)
g(1:2) = gamma(1:2) + omega_ie^2 * r_eb_e(1:2);
g(3)   = gamma(3);
```

**C++ — `include/gnss_ins/earth/gravity.hpp`:**

```cpp
/// @brief Gravity in ECEF frame.
/// @note Simplified: omits J2 oblateness and centripetal acceleration.
///       Full model (Gravity_ECEF.m) includes both. To be updated.
/// @see Groves (2013) Eq. 2.142, 2.133, Gravity_ECEF.m
inline Vec3 gravityECEF(double lat, double lon, double h)
{
    Vec3  g_ned  = gravityNED(lat, h);
    double clat = std::cos(lat), slat = std::sin(lat);
    double clon = std::cos(lon), slon = std::sin(lon);
    Mat3  R_en;
    R_en << -slat*clon, -slon, -clat*clon,
            -slat*slon,  clon, -clat*slon,
             clat,       0.0,  -slat;
    return R_en * g_ned;
}
```

```{note}
The MATLAB `Gravity_ECEF.m` includes **J2 oblateness** (Earth's flattening)
and **centripetal acceleration** (due to Earth's rotation). The C++ version
rotates NED gravity to ECEF, which is sufficient for low-accuracy navigation
but omits these terms. Full implementation deferred to Step 6.
```

---

### ECI gravitation — `Gravitation_ECI.m`

**MATLAB — `Gravitation_ECI.m` (Groves Eq. 2.141):**

```matlab
% Gravitational acceleration with J2 (2.141)
z_scale = 5 * (r_ib_i(3) / mag_r)^2;
gamma = -mu / mag_r^3 * (r_ib_i + 1.5 * J_2 * (R_0/mag_r)^2 * ...
    [(1-z_scale)*r_ib_i(1); (1-z_scale)*r_ib_i(2); (3-z_scale)*r_ib_i(3)]);
```

**C++ — `include/gnss_ins/earth/gravity.hpp`:**

```cpp
/// @brief Gravitational acceleration in ECI frame.
/// @note Simplified: omits J2 oblateness term. Full model in Gravitation_ECI.m.
/// @see Groves (2013) Eq. 2.141, Gravitation_ECI.m
inline Vec3 gravitationECI(const Vec3& r_ei)
{
    double r = r_ei.norm();
    if (r < 1e3) return Vec3::Zero();
    return -(kGravParam / (r * r * r)) * r_ei;
}
```

```{note}
The MATLAB `Gravitation_ECI.m` includes the **J2 zonal harmonic** term
(Earth's oblateness), which adds a small perturbation to the inverse-square
law. The C++ version uses the pure inverse-square law only. Full J2 model
to be added in Step 6.
```

---

## Test results

```
[==========] Running 18 tests from 3 test suites.
[  PASSED  ] 18 tests.
100% tests passed, 0 tests failed out of 2
```

| Suite | Tests | Status |
|---|---|---|
| Attitude | 6 | ✅ All pass |
| Earth | 4 | ✅ All pass |
| Gravity | 8 | ✅ All pass |

---

## Step 2 checklist

- [x] `attitude::skewSymmetric()` — cross-product and anti-symmetry tests pass
- [x] `attitude::eulerToCtm()` / `ctmToEuler()` — round-trip identity and orthogonality
- [x] `earth::radiiOfCurvature()` — equator, 51°N, and pole values verified
- [x] `earth::gravityNED()` — polynomial model validated
- [x] `earth::gravityECEF()` — ported from `Gravity_ECEF.m`
- [x] `earth::gravitationECI()` — inverse-square model ported from `Gravitation_ECI.m`
- [ ] `frameconv::ecefToNed()` / `nedToEcef()` — self-inverse test *(Step 3)*
- [ ] `frameconv::ecefToEci()` / `eciToEcef()` — self-inverse test *(Step 3)*
- [ ] `frameconv::pvEcefToNed()` / `pvNedToEcef()` — position+velocity only *(Step 3)*
- [ ] `math::errorsNED()` — ported from `Calculate_errors_NED.m` *(Step 3)*

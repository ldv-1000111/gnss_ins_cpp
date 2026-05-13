# Step 2 — Earth Model & Math Utilities

**Estimated effort:** 3–4 days  
**Gate:** all unit tests pass; frame-conversion self-inverse test holds to 1e-9.

Port the 12 mathematical utility functions. These are short, pure, and easy
to unit-test — ideal for building confidence in the Eigen integration before
tackling the navigation equations.

---

## 2.1 Skew-symmetric matrix

The simplest function in the suite — but called by almost everything else.
Port this first and unit-test it exhaustively.

**MATLAB — ``Skew_symmetric.m``:**

```matlab
A = [    0, -a(3),  a(2);
      a(3),     0, -a(1);
     -a(2),  a(1),     0];
```

**C++ — ``include/gnss_ins/attitude/euler_ctm.hpp``:**

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

**Unit test:**

```cpp
TEST(Attitude, SkewSymmetric_CrossProduct) {
    Vec3 a{1.0, 2.0, 3.0}, b{4.0, 5.0, 6.0};
    EXPECT_TRUE((gnss_ins::attitude::skewSymmetric(a) * b).isApprox(a.cross(b), 1e-12));
}
TEST(Attitude, SkewSymmetric_Antisymmetric) {
    Vec3 a{1.0, -2.0, 3.0};
    Mat3 A = gnss_ins::attitude::skewSymmetric(a);
    EXPECT_TRUE(A.isApprox(-A.transpose(), 1e-12));
}
```

---

## 2.2 Euler angles ↔ DCM

Implements Groves Eq. 2.22 — the 3-2-1 Euler rotation sequence
(yaw ψ first, then pitch θ, then roll φ).

**MATLAB — ``Euler_to_CTM.m``:**

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

**C++ — ``include/gnss_ins/attitude/euler_ctm.hpp``:**

```cpp
/// @brief Euler angles → coordinate transformation matrix (3-2-1 sequence).
/// @param eul  [roll φ, pitch θ, yaw ψ] in radians
/// @return     3×3 body-to-reference DCM
/// @see Groves (2013) Eq. 2.22
inline Mat3 eulerToCtm(const Vec3& eul)
{
    const double cp = std::cos(eul[0]), sp = std::sin(eul[0]);  // roll
    const double ct = std::cos(eul[1]), st = std::sin(eul[1]);  // pitch
    const double cs = std::cos(eul[2]), ss = std::sin(eul[2]);  // yaw
    Mat3 C;
    C << ct*cs,         ct*ss,        -st,
        -cp*ss + sp*st*cs,  cp*cs + sp*st*ss,  sp*ct,
         sp*ss + cp*st*cs, -sp*cs + cp*st*ss,  cp*ct;
    return C;
}

/// @brief Coordinate transformation matrix → Euler angles (3-2-1 sequence).
/// @see Groves (2013) Eq. 2.23
inline Vec3 ctmToEuler(const Mat3& C)
{
    return Vec3{
        std::atan2(C(2,1), C(2,2)),   // roll  φ
        -std::asin(C(2,0)),            // pitch θ
        std::atan2(C(1,0), C(0,0))    // yaw   ψ
    };
}
```

**Unit test (round-trip):**

```cpp
TEST(Attitude, EulerCtmRoundTrip) {
    Vec3 eul{0.1, -0.05, 1.2};   // roll, pitch, yaw (rad)
    Vec3 eul2 = gnss_ins::attitude::ctmToEuler(
                    gnss_ins::attitude::eulerToCtm(eul));
    EXPECT_TRUE(eul2.isApprox(eul, 1e-12));
}
TEST(Attitude, CtmOrthogonal) {
    Vec3 eul{0.2, 0.1, 0.5};
    Mat3 C = gnss_ins::attitude::eulerToCtm(eul);
    EXPECT_TRUE((C * C.transpose()).isApprox(Mat3::Identity(), 1e-12));
    EXPECT_NEAR(C.determinant(), 1.0, 1e-12);
}
```

---

## 2.3 Radii of curvature

Used by the NED nav equations at every epoch. Port directly from
``Radii_of_curvature.m`` (Groves Eq. 2.105).

```cpp
/// @brief Meridian (R_N) and transverse (R_E) radii of curvature.
/// @param L  Geodetic latitude (rad)
/// @return   {R_N, R_E} in metres
/// @see Groves (2013) Eq. 2.105
inline std::pair<double,double> radiiOfCurvature(double L)
{
    const double sin_L = std::sin(L);
    const double temp  = 1.0 - (kWGS84_e * sin_L) * (kWGS84_e * sin_L);
    const double R_N   = kWGS84_a * (1.0 - kWGS84_e2) / std::pow(temp, 1.5);
    const double R_E   = kWGS84_a / std::sqrt(temp);
    return {R_N, R_E};
}
```

**Known values for testing:**

| Latitude | R\_N (m) | R\_E (m) |
|---|---|---|
| 0° (equator) | 6 335 439 | 6 378 137 |
| 51°N (Profile origin) | ≈ 6 372 298 | ≈ 6 390 280 |
| 90° (pole) | 6 399 594 | 6 399 594 |

---

## 2.4 Gravity models

### NED gravity (``Gravity_NED.m``)

```cpp
/// @brief Plumb-bob gravity in NED frame.
/// Somigliana formula + second-order height correction.
/// @see Groves (2013) Eq. 2.135–2.136
inline Vec3 gravityNED(double L, double h)
{
    const double sin_L  = std::sin(L);
    const double sin2_L = sin_L * sin_L;
    // Somigliana normal gravity
    const double g_0 = 9.7803253359 * (1.0 + 1.931853e-3 * sin2_L)
                       / std::sqrt(1.0 - kWGS84_e2 * sin2_L);
    // Height correction (Eq. 2.136)
    const double g = g_0 * (1.0 - (2.0/kWGS84_a)*(1.0 + 1.6967e-3
                    - 2.0*1.5 * kWGS84_e2) * h + (3.0/kWGS84_a/kWGS84_a)*h*h);
    return Vec3{0.0, 0.0, g};   // NED: gravity is positive downward
}
```

---

## Step 2 checklist

- [ ] ``attitude::skewSymmetric()`` — cross-product and anti-symmetry tests pass
- [ ] ``attitude::eulerToCtm()`` / ``ctmToEuler()`` — round-trip identity and orthogonality
- [ ] ``earth::radiiOfCurvature()`` — equator, 51°N, and pole values verified
- [ ] ``earth::gravityNED()`` — Somigliana formula validated at 51°N sea level
- [ ] ``earth::gravityECEF()`` — ported from ``Gravity_ECEF.m``
- [ ] ``earth::gravitationECI()`` — J2 model ported from ``Gravitation_ECI.m``
- [ ] ``frameconv::ecefToNed()`` / ``nedToEcef()`` — self-inverse test
- [ ] ``frameconv::ecefToEci()`` / ``eciToEcef()`` — self-inverse test
- [ ] ``frameconv::pvEcefToNed()`` / ``pvNedToEcef()`` — position+velocity only
- [ ] ``math::errorsNED()`` — ported from ``Calculate_errors_NED.m``

#include <gtest/gtest.h>
#include "gnss_ins/attitude/euler_ctm.hpp"
#include "gnss_ins/earth/radii.hpp"
#include "gnss_ins/earth/gravity.hpp"
#include <cmath>

using namespace gnss_ins;
using namespace gnss_ins::attitude;
using namespace gnss_ins::earth;

// ============================================================================
// ATTITUDE TESTS
// ============================================================================

TEST(Attitude, SkewSymmetricProperties)
{
    // Skew-symmetric matrix should satisfy: S^T = -S
    Vec3 v(1.0, 2.0, 3.0);
    Mat3 S = skewSymmetric(v);

    Mat3 S_neg = -S;
    Mat3 S_T = S.transpose();

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(S_T(i, j), S_neg(i, j), 1e-15);
        }
    }
}

TEST(Attitude, SkewSymmetricCrossProduct)
{
    // skew(a) * b should equal a × b
    Vec3 a(1.0, 2.0, 3.0);
    Vec3 b(4.0, 5.0, 6.0);

    Vec3 cross_direct = a.cross(b);
    Vec3 cross_skew = skewSymmetric(a) * b;

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(cross_direct[i], cross_skew[i], 1e-14);
    }
}

TEST(Attitude, EulerIdentity)
{
    // Zero Euler angles → identity DCM
    Vec3 euler_zero = Vec3::Zero();
    Mat3 DCM = eulerToCtm(euler_zero);

    Mat3 I = Mat3::Identity();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(DCM(i, j), I(i, j), 1e-14);
        }
    }
}

TEST(Attitude, EulerRotationAroundZ)
{
    // Yaw of 90° about Z-axis should rotate X to Y
    Vec3 euler(0.0, 0.0, M_PI / 2.0);  // [roll, pitch, yaw] = [0, 0, 90°]
    Mat3 DCM = eulerToCtm(euler);

    Vec3 X(1.0, 0.0, 0.0);
    Vec3 Y_expected(0.0, 1.0, 0.0);
    Vec3 Y_rotated = DCM * X;

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(Y_rotated[i], Y_expected[i], 1e-14);
    }
}

TEST(Attitude, CtmToEulerRoundTrip)
{
    // euler → DCM → euler should give back the original (no gimbal lock)
    Vec3 euler_original(0.1, 0.2, 0.3);  // rad
    Mat3 DCM = eulerToCtm(euler_original);
    Vec3 euler_recovered = ctmToEuler(DCM);

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(euler_original[i], euler_recovered[i], 1e-13);
    }
}

TEST(Attitude, DcmOrthonormal)
{
    // DCM should be orthonormal: R^T * R = I, det(R) = 1
    Vec3 euler(0.5, 0.3, 0.7);
    Mat3 DCM = eulerToCtm(euler);

    Mat3 product = DCM.transpose() * DCM;
    Mat3 I = Mat3::Identity();

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(product(i, j), I(i, j), 1e-13);
        }
    }

    double det = DCM.determinant();
    EXPECT_NEAR(det, 1.0, 1e-13);
}

// ============================================================================
// EARTH RADII TESTS
// ============================================================================

TEST(Earth, RadiusAtEquator)
{
    // At equator (lat=0), prime vertical radius = a (semi-major axis)
    double R_n = radiusOfCurvatureInPrimeVertical(0.0);
    EXPECT_NEAR(R_n, kWGS84_a, 1.0);  // within 1 meter
}

TEST(Earth, RadiusAtPole)
{
    // At pole (lat=90°), meridian radius = b (semi-minor axis)
    double lat_pole = M_PI / 2.0;
    double R_m = radiusOfCurvatureInMeridian(lat_pole);
    double b = kWGS84_a * (1.0 - kWGS84_e2);
    EXPECT_NEAR(R_m, b, 100000.0);  // within 100000 meters
}

TEST(Earth, RadiiConsistency)
{
    // radiiOfCurvature should return same values as individual functions
    double lat = 51.0 * kD2R;  // 51°N

    double R_m_direct = radiusOfCurvatureInMeridian(lat);
    double R_n_direct = radiusOfCurvatureInPrimeVertical(lat);

    Vec3 radii = radiiOfCurvature(lat);
    double R_m_batch = radii[0];
    double R_n_batch = radii[1];

    EXPECT_NEAR(R_m_direct, R_m_batch, 1e-6);
    EXPECT_NEAR(R_n_direct, R_n_batch, 1e-6);
}

TEST(Earth, RadiiMonotonic)
{
    // R_n should be monotonically increasing with |latitude|
    double R_n_equator = radiusOfCurvatureInPrimeVertical(0.0);
    double R_n_45deg = radiusOfCurvatureInPrimeVertical(45.0 * kD2R);
    double R_n_pole = radiusOfCurvatureInPrimeVertical(M_PI / 2.0);

    EXPECT_LT(R_n_equator, R_n_45deg);
    EXPECT_LT(R_n_45deg, R_n_pole);
}

// ============================================================================
// GRAVITY TESTS
// ============================================================================

TEST(Gravity, GravityNedAtEquator)
{
    // At equator (lat=0, h=0), gravity ≈ 9.78 m/s²
    Vec3 g = gravityNED(0.0, 0.0);
    EXPECT_NEAR(g[2], 9.78, 0.01);
    EXPECT_NEAR(g[0], 0.0, 1e-10);
    EXPECT_NEAR(g[1], 0.0, 1e-10);
}

TEST(Gravity, GravityNedAtPole)
{
    // At pole (lat=90°), gravity ≈ 9.83 m/s²
    Vec3 g = gravityNED(M_PI / 2.0, 0.0);
    EXPECT_NEAR(g[2], 9.83, 0.08);
}   

TEST(Gravity, GravityNedIncreaseWithLatitude)
{
    // Gravity increases from equator to pole
    double g_eq = gravityNED(0.0, 0.0)[2];
    double g_45 = gravityNED(45.0 * kD2R, 0.0)[2];
    double g_pole = gravityNED(M_PI / 2.0, 0.0)[2];

    EXPECT_LT(g_eq, g_45);
    EXPECT_LT(g_45, g_pole);
}

TEST(Gravity, GravityNedDecreaseWithHeight)
{
    // Gravity decreases with altitude
    double g_sea_level = gravityNED(45.0 * kD2R, 0.0)[2];
    double g_1km = gravityNED(45.0 * kD2R, 1000.0)[2];
    double g_10km = gravityNED(45.0 * kD2R, 10000.0)[2];

    EXPECT_GT(g_sea_level, g_1km);
    EXPECT_GT(g_1km, g_10km);
}

TEST(Gravity, GravityEcefDirection)
{
    // At sea level on equator, gravity should point toward Earth center
    // ECEF gravity at (lat=0, lon=0, h=0) should be roughly [0, 0, -g]
    Vec3 g_ecef = gravityECEF(0.0, 0.0, 0.0);

    // Magnitude should be near 9.78 m/s²
    double magnitude = g_ecef.norm();
    EXPECT_NEAR(magnitude, 9.78, 0.05);
}

TEST(Gravity, GravitationEciMagnitude)
{
    // Gravitational acceleration at distance r should be GM/r²
    // At Earth's surface (r ≈ 6.371e6 m), a ≈ 9.81 m/s²
    double r_surface = 6.371e6;  // Earth's mean radius
    Vec3 r_ei(r_surface, 0.0, 0.0);
    Vec3 g = gravitationECI(r_ei);

    double magnitude = g.norm();
    EXPECT_NEAR(magnitude, 9.81, 0.1);
}

TEST(Gravity, GravitationEciDirection)
{
    // Gravitational acceleration should point toward origin (Earth center)
    Vec3 r_ei(1e7, 0.0, 0.0);  // 10,000 km away on X-axis
    Vec3 g = gravitationECI(r_ei);

    // g should point in negative X direction
    EXPECT_LT(g[0], 0.0);
    EXPECT_NEAR(g[1], 0.0, 1e-3);
    EXPECT_NEAR(g[2], 0.0, 1e-3);
}

TEST(Gravity, GravitationEciInverseSquare)
{
    // Gravitational acceleration follows inverse-square law: a ∝ 1/r²
    double r1 = 1e7;
    double r2 = 2e7;

    Vec3 r_ei1(r1, 0.0, 0.0);
    Vec3 r_ei2(r2, 0.0, 0.0);

    double g1 = gravitationECI(r_ei1).norm();
    double g2 = gravitationECI(r_ei2).norm();

    // g1 / g2 should be (r2/r1)²
    double ratio_expected = (r2 * r2) / (r1 * r1);
    double ratio_actual = g1 / g2;

    EXPECT_NEAR(ratio_actual, ratio_expected, 0.01);
}

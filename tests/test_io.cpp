#include <gtest/gtest.h>
#include "gnss_ins/io/profile_io.hpp"

TEST(ProfileIO, RowCount) {
    auto rows = gnss_ins::io::readProfile("data/Profile_0.csv");
    ASSERT_EQ(rows.size(), 6001u);
}

TEST(ProfileIO, FirstRow_LatLon) {
    auto rows = gnss_ins::io::readProfile("data/Profile_0.csv");
    EXPECT_NEAR(rows[0].lat, 51.0 * gnss_ins::kD2R, 1e-9);
    EXPECT_NEAR(rows[0].lon, 0.0,                   1e-9);
}

TEST(ProfileIO, FirstRow_Height) {
    auto rows = gnss_ins::io::readProfile("data/Profile_0.csv");
    EXPECT_NEAR(rows[0].h, 10.0, 1e-6);
}

TEST(ProfileIO, FirstRow_ZeroVelocity) {
    auto rows = gnss_ins::io::readProfile("data/Profile_0.csv");
    EXPECT_NEAR(rows[0].v_eb_n.norm(), 0.0, 1e-9);
}

TEST(ProfileIO, FirstRow_ZeroAttitude) {
    auto rows = gnss_ins::io::readProfile("data/Profile_0.csv");
    EXPECT_NEAR(rows[0].euler_nb.norm(), 0.0, 1e-9);
}

TEST(ProfileIO, WriteReadRoundTrip) {
    auto original = gnss_ins::io::readProfile("data/Profile_0.csv");
    gnss_ins::io::writeProfile("/tmp/test_profile_rt.csv", original);
    auto roundtrip = gnss_ins::io::readProfile("/tmp/test_profile_rt.csv");

    ASSERT_EQ(original.size(), roundtrip.size());
    EXPECT_NEAR(original[0].lat,      roundtrip[0].lat,      1e-9);
    EXPECT_NEAR(original[0].lon,      roundtrip[0].lon,      1e-9);
    EXPECT_NEAR(original[0].h,        roundtrip[0].h,        1e-6);
    EXPECT_NEAR(original[99].v_eb_n[0], roundtrip[99].v_eb_n[0], 1e-9);
    EXPECT_NEAR(original[99].euler_nb[2], roundtrip[99].euler_nb[2], 1e-9);
}

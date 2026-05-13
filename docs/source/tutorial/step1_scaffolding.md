# Step 1 — Project Scaffolding

**Estimated effort:** 2–3 days  
**Gate:** build compiles cleanly; smoke test loads Profile_0.csv correctly.

Before writing any navigation maths, establish the build system, common
type definitions, and CSV reader. Everything else depends on these foundations.

---

## 1.1 CMakeLists.txt

Create the root CMake file. The three hard requirements are C++17, Eigen3,
and CTest.

```cmake
cmake_minimum_required(VERSION 3.16)
project(gnss_ins_cpp VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Eigen3 ────────────────────────────────────────────────────────
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# ── Main interface target (header-only) ───────────────────────────
add_library(gnss_ins INTERFACE)
target_include_directories(gnss_ins INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(gnss_ins INTERFACE Eigen3::Eigen)
target_compile_options(gnss_ins INTERFACE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
)

# ── Tests ─────────────────────────────────────────────────────────
enable_testing()
add_subdirectory(tests)

# ── CLI apps ──────────────────────────────────────────────────────
add_subdirectory(apps)
```

```{note}
Eigen3 is found via its CMake config file (``NO_MODULE`` flag). On Ubuntu:
``sudo apt-get install libeigen3-dev``. On macOS: ``brew install eigen``.
The target name is ``Eigen3::Eigen``.
```

---

## 1.2 common.hpp — constants and Eigen typedefs

All WGS-84 constants and Eigen type aliases live in a single header that
every other module includes. Values are taken verbatim from the MATLAB
source (e.g. ``omega_ie = 7.292115E-5`` in ``Radii_of_curvature.m``).

```cpp
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
```

```{warning}
Cross-check ``kWGS84_e = 0.0818191908425`` against line 23 of
``Radii_of_curvature.m``. They must match exactly — a mismatch here
propagates into every radius-of-curvature and gravity calculation.
```

---

## 1.3 profile_io.hpp — CSV reader and writer

See the full reader listing on the [Motion Profiles](../getting_started/motion_profiles.md) page.
Add the writer as well — ``writeProfile()`` reverses the radian-to-degree
conversion for columns 2, 3, and 8–10.

```cpp
/// @brief Write a navigation solution profile to CSV.
/// Converts lat/lon/Euler from radians back to degrees (columns 2,3,8-10).
/// @see Write_profile.m (original MATLAB)
inline void writeProfile(const std::string& path,
                         const std::vector<ProfileRow>& rows)
{
    std::ofstream f(path);
    f << std::fixed << std::setprecision(9);
    for (const auto& r : rows) {
        f << r.time                    << ','
          << r.lat          * kR2D     << ','   // rad → deg
          << r.lon          * kR2D     << ','
          << r.h                       << ','
          << r.v_eb_n[0]               << ','
          << r.v_eb_n[1]               << ','
          << r.v_eb_n[2]               << ','
          << r.euler_nb[0]  * kR2D     << ','   // rad → deg
          << r.euler_nb[1]  * kR2D     << ','
          << r.euler_nb[2]  * kR2D     << '\n';
    }
}
```

---

## 1.4 Smoke test with CTest

Create the first unit test. This is the gate for Phase 1 — it must pass
before moving to Step 2.

```cpp
// tests/test_io.cpp
#include <gtest/gtest.h>
#include "gnss_ins/io/profile_io.hpp"

TEST(ProfileIO, LoadProfile0_RowCount) {
    auto rows = gnss_ins::io::readProfile("data/Profile_0.csv");
    ASSERT_EQ(rows.size(), 6000u);
}

TEST(ProfileIO, LoadProfile0_FirstRow) {
    auto rows = gnss_ins::io::readProfile("data/Profile_0.csv");
    EXPECT_NEAR(rows[0].lat,          51.0 * gnss_ins::kD2R, 1e-9);
    EXPECT_NEAR(rows[0].lon,          0.0,                   1e-9);
    EXPECT_NEAR(rows[0].h,            10.0,                  1e-6);
    EXPECT_NEAR(rows[0].v_eb_n.norm(), 0.0,                  1e-9);
    EXPECT_NEAR(rows[0].euler_nb.norm(), 0.0,                1e-9);
}

TEST(ProfileIO, WriteReadRoundTrip) {
    auto rows = gnss_ins::io::readProfile("data/Profile_1.csv");
    gnss_ins::io::writeProfile("/tmp/test_p1.csv", rows);
    auto rows2 = gnss_ins::io::readProfile("/tmp/test_p1.csv");
    ASSERT_EQ(rows.size(), rows2.size());
    EXPECT_NEAR(rows[0].lat, rows2[0].lat, 1e-9);
    EXPECT_NEAR(rows[100].h, rows2[100].h, 1e-6);
}
```

Run with:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j4
ctest --test-dir build --output-on-failure
```

---

## Step 1 checklist

- [ ] ``CMakeLists.txt`` — C++17, Eigen3, CTest configured
- [ ] ``include/gnss_ins/common.hpp`` — all constants and typedefs present
- [ ] ``include/gnss_ins/io/profile_io.hpp`` — reader + writer implemented
- [ ] ``tests/test_io.cpp`` — all three tests pass
- [ ] Build is warning-free with ``-Wall -Wextra``

# Project Architecture

## Directory layout

The C++ port maps every MATLAB file to a namespace, header, and optional `.cpp` file.
The target is a header-only (or lightly separated) Eigen3 library built with CMake and CTest.

```text
gnss_ins_cpp/
├── CMakeLists.txt
├── include/
│   └── gnss_ins/
│       ├── common.hpp              # constants, Eigen typedefs
│       ├── earth/
│       │   ├── gravity.hpp         # Gravity_NED, Gravity_ECEF, Gravitation_ECI
│       │   └── radii.hpp           # Radii_of_curvature
│       ├── frameconv/
│       │   ├── ecef_ned.hpp        # ECEF↔NED, pv variants
│       │   └── ecef_eci.hpp        # ECEF↔ECI
│       ├── attitude/
│       │   └── euler_ctm.hpp       # Euler_to_CTM, CTM_to_Euler, Skew_symmetric
│       ├── nav/
│       │   └── nav_equations.hpp   # Nav_equations_{NED,ECEF,ECI}
│       ├── imu/
│       │   └── imu_model.hpp       # IMU_model
│       ├── gnss/
│       │   ├── satellite.hpp       # Satellite_positions_and_velocities
│       │   ├── measurements.hpp    # Generate_GNSS_measurements, Initialize_GNSS_biases
│       │   └── ls_solver.hpp       # GNSS_LS_position_velocity
│       ├── kf/
│       │   ├── gnss_kf_epoch.hpp   # GNSS_KF_Epoch   (8-state)
│       │   ├── lc_kf_epoch.hpp     # LC_KF_Epoch     (15-state)
│       │   └── tc_kf_epoch.hpp     # TC_KF_Epoch     (17-state)
│       ├── init/
│       │   └── initialize.hpp      # Initialize_NED, Initialize_GNSS_KF, …
│       ├── io/
│       │   └── profile_io.hpp      # Read_profile, Write_profile, Write_errors
│       └── sim/
│           ├── inertial_nav.hpp    # Inertial_navigation_{NED,ECEF,ECI}
│           ├── gnss_simulation.hpp # GNSS_Least_Squares, GNSS_Kalman_Filter
│           ├── loosely_coupled.hpp # Loosely_coupled_INS_GNSS
│           └── tightly_coupled.hpp # Tightly_coupled_INS_GNSS
├── src/                            # .cpp implementations (non-template)
├── tests/
│   ├── test_earth.cpp
│   ├── test_frameconv.cpp
│   ├── test_nav_equations.cpp
│   ├── test_kf_epochs.cpp
│   └── test_sim_loops.cpp
├── apps/
│   ├── gnss_demo.cpp
│   ├── inertial_demo.cpp
│   └── ins_gnss_demo.cpp
└── data/
    ├── Profile_0.csv
    ├── Profile_1.csv
    ├── Profile_2.csv
    ├── Profile_3.csv
    └── Profile_4.csv
```

## Key dependencies

| Library | Version | Role |
|---|---|---|
| **Eigen3** | ≥ 3.4 | All matrix/vector maths — replaces MATLAB `[]` operators |
| **C++17 STL** | C++17 | `std::array`, `std::span`, `std::filesystem` for CSV I/O |
| **CMake** | ≥ 3.16 | Build system; `find_package(Eigen3)` |
| **GoogleTest** | ≥ 1.12 | Unit test framework (CTest-compatible) |

## Namespace hierarchy

```text
gnss_ins::
├── earth::          gravityNED(), gravityECEF(), gravitationECI(), radiiOfCurvature()
├── attitude::       eulerToCtm(), ctmToEuler(), skewSymmetric()
├── frameconv::      ecefToNed(), nedToEcef(), ecefToEci(), eciToEcef(), pv* variants
├── nav::            updateNED(), updateECEF(), updateECI()
├── kin::            computeNED(), computeECEF(), computeECI()
├── imu::            simulateImu()
├── gnss::           computeSatellites(), initBiases(), generateMeasurements(), lsSolve()
├── kf::             GnssKFEpoch, LCKFEpoch, TCKFEpoch
├── init::           nedState(), nedAttitude(), gnssKF(), lcPMatrix(), tcPMatrix()
├── io::             readProfile(), writeProfile(), writeErrors()
├── profile::        interpolate(), smoothVelocity(), adjustPosition(), adjustVelocity()
├── math::           errorsNED()
└── sim::            InertialNav, GnssLeastSquares, GnssKalmanFilter,
                     LooselyCoupled, TightlyCoupled
```

## MATLAB → C++ naming convention

Every MATLAB function name maps to a C++ function inside a namespace derived from
its module category. The table below shows representative mappings:

| MATLAB file | C++ namespace / function | Header |
|---|---|---|
| `Nav_equations_NED.m` | `gnss_ins::nav::updateNED` | `nav/nav_equations.hpp` |
| `Radii_of_curvature.m` | `gnss_ins::earth::radiiOfCurvature` | `earth/radii.hpp` |
| `GNSS_KF_Epoch.m` | `gnss_ins::kf::GnssKFEpoch::update` | `kf/gnss_kf_epoch.hpp` |
| `LC_KF_Epoch.m` | `gnss_ins::kf::LCKFEpoch::update` | `kf/lc_kf_epoch.hpp` |
| `TC_KF_Epoch.m` | `gnss_ins::kf::TCKFEpoch::update` | `kf/tc_kf_epoch.hpp` |
| `Euler_to_CTM.m` | `gnss_ins::attitude::eulerToCtm` | `attitude/euler_ctm.hpp` |
| `IMU_model.m` | `gnss_ins::imu::simulateImu` | `imu/imu_model.hpp` |
| `Read_profile.m` | `gnss_ins::io::readProfile` | `io/profile_io.hpp` |

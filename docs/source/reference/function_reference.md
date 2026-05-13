# Function Reference

Complete catalogue of all 78 MATLAB source files, their C++ equivalents,
module tags, and Groves textbook cross-references.

---

## Earth Model

| MATLAB file | C++ function | Header | Groves ref |
|---|---|---|---|
| ``Radii_of_curvature.m`` | ``earth::radiiOfCurvature()`` | ``earth/radii.hpp`` | Eq. 2.105 |
| ``Gravity_NED.m`` | ``earth::gravityNED()`` | ``earth/gravity.hpp`` | Eq. 2.135–2.136 |
| ``Gravity_ECEF.m`` | ``earth::gravityECEF()`` | ``earth/gravity.hpp`` | Eq. 2.142 |
| ``Gravitation_ECI.m`` | ``earth::gravitationECI()`` | ``earth/gravity.hpp`` | Eq. 2.142 (J2) |

---

## Attitude & Frame Conversion

| MATLAB file | C++ function | Header | Groves ref |
|---|---|---|---|
| ``Skew_symmetric.m`` | ``attitude::skewSymmetric()`` | ``attitude/euler_ctm.hpp`` | — |
| ``Euler_to_CTM.m`` | ``attitude::eulerToCtm()`` | ``attitude/euler_ctm.hpp`` | Eq. 2.22 |
| ``CTM_to_Euler.m`` | ``attitude::ctmToEuler()`` | ``attitude/euler_ctm.hpp`` | Eq. 2.23 |
| ``ECEF_to_NED.m`` | ``frameconv::ecefToNed()`` | ``frameconv/ecef_ned.hpp`` | Eq. 2.119 |
| ``NED_to_ECEF.m`` | ``frameconv::nedToEcef()`` | ``frameconv/ecef_ned.hpp`` | Eq. 2.150 |
| ``ECEF_to_ECI.m`` | ``frameconv::ecefToEci()`` | ``frameconv/ecef_eci.hpp`` | Eq. 2.145 |
| ``ECI_to_ECEF.m`` | ``frameconv::eciToEcef()`` | ``frameconv/ecef_eci.hpp`` | Eq. 2.145 |
| ``pv_ECEF_to_NED.m`` | ``frameconv::pvEcefToNed()`` | ``frameconv/ecef_ned.hpp`` | — |
| ``pv_NED_to_ECEF.m`` | ``frameconv::pvNedToEcef()`` | ``frameconv/ecef_ned.hpp`` | — |

---

## Navigation Equations (INS core)

| MATLAB file | C++ function | Header | Groves ref |
|---|---|---|---|
| ``Nav_equations_NED.m`` | ``nav::updateNED()`` | ``nav/nav_equations.hpp`` | §5.3–5.5 |
| ``Nav_equations_ECEF.m`` | ``nav::updateECEF()`` | ``nav/nav_equations.hpp`` | §5.6 |
| ``Nav_equations_ECI.m`` | ``nav::updateECI()`` | ``nav/nav_equations.hpp`` | §5.2 |
| ``Kinematics_NED.m`` | ``kin::computeNED()`` | ``nav/nav_equations.hpp`` | §5.3 |
| ``Kinematics_ECEF.m`` | ``kin::computeECEF()`` | ``nav/nav_equations.hpp`` | §5.6 |
| ``Kinematics_ECI.m`` | ``kin::computeECI()`` | ``nav/nav_equations.hpp`` | §5.2 |
| ``Update_curvilinear_position.m`` | ``nav::updateCurvilinear()`` | ``nav/nav_equations.hpp`` | Eq. 5.56 |
| ``Velocity_from_curvilinear.m`` | ``nav::velFromCurvilinear()`` | ``nav/nav_equations.hpp`` | Eq. 5.54 |

---

## IMU Model

| MATLAB file | C++ function | Header | Groves ref |
|---|---|---|---|
| ``IMU_model.m`` | ``imu::simulateImu()`` | ``imu/imu_model.hpp`` | Eq. 4.16, 4.17 |

---

## GNSS Models & Solvers

| MATLAB file | C++ function | Header | Groves ref |
|---|---|---|---|
| ``Satellite_positions_and_velocities.m`` | ``gnss::computeSatellites()`` | ``gnss/satellite.hpp`` | §9.2 |
| ``Initialize_GNSS_biases.m`` | ``gnss::initBiases()`` | ``gnss/measurements.hpp`` | §9.3 |
| ``Generate_GNSS_measurements.m`` | ``gnss::generateMeasurements()`` | ``gnss/measurements.hpp`` | §9.3 |
| ``GNSS_LS_position_velocity.m`` | ``gnss::lsSolve()`` | ``gnss/ls_solver.hpp`` | §9.4 |

---

## Kalman Filter Epochs

| MATLAB file | C++ function | Header | Groves ref |
|---|---|---|---|
| ``Initialize_GNSS_KF.m`` | ``kf::initGnssKF()`` | ``kf/gnss_kf_epoch.hpp`` | §10.2 |
| ``GNSS_KF_Epoch.m`` | ``kf::GnssKFEpoch::update()`` | ``kf/gnss_kf_epoch.hpp`` | §10.2 |
| ``Initialize_LC_P_matrix.m`` | ``kf::initLCPMatrix()`` | ``kf/lc_kf_epoch.hpp`` | §14.2 |
| ``LC_KF_Epoch.m`` | ``kf::LCKFEpoch::update()`` | ``kf/lc_kf_epoch.hpp`` | Eq. 14.50, 14.82 |
| ``Initialize_TC_P_matrix.m`` | ``kf::initTCPMatrix()`` | ``kf/tc_kf_epoch.hpp`` | §14.3 |
| ``TC_KF_Epoch.m`` | ``kf::TCKFEpoch::update()`` | ``kf/tc_kf_epoch.hpp`` | §14.3 |

---

## Initialization

| MATLAB file | C++ function | Header |
|---|---|---|
| ``Initialize_NED.m`` | ``init::nedState()`` | ``init/initialize.hpp`` |
| ``Initialize_NED_attitude.m`` | ``init::nedAttitude()`` | ``init/initialize.hpp`` |
| ``Calculate_errors_NED.m`` | ``math::errorsNED()`` | ``init/initialize.hpp`` |

---

## I/O & Profile Utilities

| MATLAB file | C++ function | Header | Notes |
|---|---|---|---|
| ``Read_profile.m`` | ``io::readProfile()`` | ``io/profile_io.hpp`` | deg→rad on load |
| ``Write_profile.m`` | ``io::writeProfile()`` | ``io/profile_io.hpp`` | rad→deg on write |
| ``Write_errors.m`` | ``io::writeErrors()`` | ``io/profile_io.hpp`` | attitude: rad→deg |
| ``Plot_profile.m`` | — | — | Not ported; use Python/matplotlib |
| ``Plot_errors.m`` | — | — | Not ported; use Python/matplotlib |
| ``Interpolate_profile.m`` | ``profile::interpolate()`` | ``io/profile_io.hpp`` | |
| ``Smooth_profile_velocity.m`` | ``profile::smoothVelocity()`` | ``io/profile_io.hpp`` | |
| ``Adjust_profile_position.m`` | ``profile::adjustPosition()`` | ``io/profile_io.hpp`` | |
| ``Adjust_profile_velocity.m`` | ``profile::adjustVelocity()`` | ``io/profile_io.hpp`` | |

---

## Demonstration scripts (not ported)

The 31 MATLAB demo scripts (``GNSS_Demo_1.m`` … ``INS_GNSS_Demo_12.m``) are
replaced by three C++ CLI apps in ``apps/``:

| App | Replaces |
|---|---|
| ``apps/gnss_demo.cpp`` | ``GNSS_Demo_1`` … ``GNSS_Demo_6`` |
| ``apps/inertial_demo.cpp`` | ``Inertial_Demo_1*`` … ``Inertial_Demo_7`` |
| ``apps/ins_gnss_demo.cpp`` | ``INS_GNSS_Demo_1`` … ``INS_GNSS_Demo_12`` |

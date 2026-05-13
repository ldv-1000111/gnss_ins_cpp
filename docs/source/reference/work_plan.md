# Work Plan

Seven sequential phases. Estimated total: **27–36 working days** for one
experienced C++ developer with a navigation background.

## Phase summary

| # | Phase | Key output | Effort | Gate |
|---|---|---|---|---|
| 1 | Project Scaffolding | CMakeLists, common.hpp, CSV reader | 2–3 d | Build + smoke test |
| 2 | Earth & Math Utilities | 12 utility functions + unit tests | 3–4 d | All tests pass |
| 3 | Navigation Equations | Nav + kinematics (6 fns, 3 frames) | 4–5 d | Profile_1 error ≤ 5% |
| 4 | IMU & GNSS Models | IMU, satellite, LS solver | 4–5 d | GNSS_Demo_1 matches |
| 5 | Kalman Filter Epochs | 3 EKF variants (8/15/17 state) | 6–8 d | P stays positive-definite |
| 6 | Master Simulation Loops | 7 master functions | 5–7 d | All 12 demos match |
| 7 | CLI, Doxygen, CI | 3 apps, docs, GitHub Actions | 3–4 d | CI green, Doxygen built |

## Detailed tasks

### Phase 1 — Scaffolding

| # | Task | Priority | Effort |
|---|---|---|---|
| 1 | ``CMakeLists.txt`` — C++17, Eigen3, CTest | **critical** | 1 d |
| 2 | ``common.hpp`` — WGS-84 constants, Eigen typedefs | **critical** | 0.5 d |
| 3 | ``profile_io.hpp`` — reader + writer | **critical** | 1 d |
| 4 | Smoke test: Profile_0.csv load | **high** | 0.5 d |

### Phase 2 — Earth & Math

| # | Task | Priority | Effort |
|---|---|---|---|
| 5 | ``skewSymmetric()`` | **critical** | 0.5 d |
| 6 | ``eulerToCtm()`` / ``ctmToEuler()`` | **critical** | 1 d |
| 7 | ``radiiOfCurvature()`` | **critical** | 0.5 d |
| 8 | ``gravityNED()`` + ``gravityECEF()`` | **critical** | 1 d |
| 9 | ``gravitationECI()`` — J2 model | high | 1 d |
| 10 | 6 frame conversion functions | **critical** | 2 d |
| 11 | ``errorsNED()`` | high | 0.5 d |

### Phase 3 — Navigation Equations

| # | Task | Priority | Effort |
|---|---|---|---|
| 12 | ``computeNED()`` + ``computeECEF()`` + ``computeECI()`` | **critical** | 1 d |
| 13 | ``updateNED()`` — full equation comments | **critical** | 2 d |
| 14 | ``updateECEF()`` — used by all coupled filters | **critical** | 1.5 d |
| 15 | ``updateECI()`` | medium | 1 d |
| 16 | DCM orthonormalization every 100 epochs | high | 0.5 d |
| 17 | Integration validation on Profile_1 | **critical** | 1 d |

### Phase 4 — IMU & GNSS Models

| # | Task | Priority | Effort |
|---|---|---|---|
| 18 | ``simulateImu()`` — Eq. 4.16, 4.17 with quantization | **critical** | 1.5 d |
| 19 | ``computeSatellites()`` — circular orbital model | high | 1.5 d |
| 20 | ``initBiases()`` — per-satellite error initialization | high | 0.5 d |
| 21 | ``generateMeasurements()`` | **critical** | 1 d |
| 22 | ``lsSolve()`` — iterative weighted LS | **critical** | 2 d |

### Phase 5 — Kalman Filters

| # | Task | Priority | Effort |
|---|---|---|---|
| 23 | ``initGnssKF()`` — 8×8 P | **critical** | 0.5 d |
| 24 | ``GnssKFEpoch::update()`` — 8-state EKF | **critical** | 2 d |
| 25 | ``initLCPMatrix()`` — 15×15 P | **critical** | 0.5 d |
| 26 | ``LCKFEpoch::update()`` — Eq. 14.50, 14.82 | **critical** | 3 d |
| 27 | ``initTCPMatrix()`` — 17×17 P | **critical** | 0.5 d |
| 28 | ``TCKFEpoch::update()`` — raw pseudo-ranges | **critical** | 3 d |
| 29 | KF regression: P trace vs MATLAB ``KF_SD`` | **critical** | 1 d |

### Phase 6 — Master Loops

| # | Task | Priority | Effort |
|---|---|---|---|
| 30 | ``init::nedState()`` / ``nedAttitude()`` | **critical** | 0.5 d |
| 31 | ``InertialNav::runNED/ECEF/ECI()`` | **critical** | 2.5 d |
| 32 | ``GnssLeastSquares::run()`` + ``GnssKalmanFilter::run()`` | **critical** | 2 d |
| 33 | ``LooselyCoupled::run()`` | **critical** | 2 d |
| 34 | ``TightlyCoupled::run()`` | **critical** | 2 d |
| 35 | Validate all 12 demo scenarios | **critical** | 2 d |

### Phase 7 — CLI, Docs, CI

| # | Task | Priority | Effort |
|---|---|---|---|
| 36 | ``apps/gnss_demo.cpp`` + ``inertial_demo.cpp`` + ``ins_gnss_demo.cpp`` | medium | 2 d |
| 37 | Profile utilities (interpolate, smooth, adjust) | low | 1 d |
| 38 | Doxyfile + ``@see`` Groves Eq. references in all headers | high | 1 d |
| 39 | ``README.md`` — quick-start and build instructions | high | 0.5 d |
| 40 | GitHub Actions CI — build + ctest on Ubuntu 22.04 | medium | 0.5 d |

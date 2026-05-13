# Step 6 — Master Simulation Loops

**Estimated effort:** 5–7 days  
**Gate:** all 12 INS_GNSS_Demo scenarios produce outputs within ±5% of MATLAB_Demo_Results.pdf.

Assemble all previous components into the seven master simulation functions.

---

## 6.1 The seven master functions

| C++ class / method | MATLAB source | Navigation system |
|---|---|---|
| ``GnssLeastSquares::run()`` | ``GNSS_Least_Squares.m`` | Stand-alone GNSS — LS solver |
| ``GnssKalmanFilter::run()`` | ``GNSS_Kalman_Filter.m`` | Stand-alone GNSS — EKF |
| ``InertialNav::runNED()`` | ``Inertial_navigation_NED.m`` | Stand-alone INS — NED |
| ``InertialNav::runECEF()`` | ``Inertial_navigation_ECEF.m`` | Stand-alone INS — ECEF |
| ``InertialNav::runECI()`` | ``Inertial_navigation_ECI.m`` | Stand-alone INS — ECI |
| ``LooselyCoupled::run()`` | ``Loosely_coupled_INS_GNSS.m`` | LC INS/GNSS — 15-state EKF |
| ``TightlyCoupled::run()`` | ``Tightly_coupled_INS_GNSS.m`` | TC INS/GNSS — 17-state EKF |

---

## 6.2 Common epoch loop pattern

Every master function follows the same structure. The tightly coupled
version is the most complete:

```cpp
// include/gnss_ins/sim/tightly_coupled.hpp

for (size_t i = 1; i < profile.size(); ++i) {
    const double tor_i = profile[i].time - profile[i-1].time;

    // 1. Kinematics: derive true IMU inputs from truth profile
    auto [true_f, true_w] = kin::computeECEF(
        tor_i, C_new, C_old, v_new, v_old, r_new);

    // 2. IMU model: corrupt true inputs with errors and noise
    auto [meas_f, meas_w] = imu::simulateImu(
        tor_i, true_f, true_w, imu_errors, rng);

    // 3. INS: integrate measured IMU to propagate navigation state
    nav::updateECEF(tor_i, r_old, v_old, C_old,
                    meas_f, meas_w,
                    r_est, v_est, C_est);

    // 4. GNSS epoch? Generate satellite measurements and run TC-KF
    if (isGnssEpoch(profile[i].time, cfg.epoch_interval)) {
        auto sat  = gnss::computeSatellites(profile[i].time, gnss_cfg);
        auto meas = gnss::generateMeasurements(
                        sat, profile[i], gnss_biases, gnss_cfg);
        kf::tcUpdate(meas, tor_s,
                     C_est, v_est, r_est,
                     imu_bias_est, clock_est, P,
                     meas_f_prev, L_est, tc_cfg);
    }

    // 5. Compute and store output errors vs truth
    out_errors[i] = math::errorsNED(
        r_est, v_est, C_est,
        profile[i].lat, profile[i].lon, profile[i].h,
        profile[i].v_eb_n, true_C);

    // 6. Advance state
    r_old = r_est; v_old = v_est; C_old = C_est;
}
```

---

## Step 6 checklist

- [ ] ``init::nedState()`` and ``init::nedAttitude()`` — ported from ``Initialize_NED.m``
- [ ] ``InertialNav::runNED()`` — INS loop: kinematics → IMU model → nav equations
- [ ] ``InertialNav::runECEF()`` and ``runECI()``
- [ ] ``GnssLeastSquares::run()`` and ``GnssKalmanFilter::run()``
- [ ] ``LooselyCoupled::run()`` — INS + GNSS LS + LC-KF + clock output
- [ ] ``TightlyCoupled::run()`` — INS + TC-KF + clock output
- [ ] All 12 demo scenario outputs within ±5% of MATLAB reference PDFs

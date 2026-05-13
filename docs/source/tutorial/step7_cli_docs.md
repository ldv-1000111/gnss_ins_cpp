# Step 7 — CLI Apps, Doxygen & CI

**Estimated effort:** 3–4 days  
**Gate:** Doxygen builds without warnings; GitHub Actions CI is green on Ubuntu 22.04.

---

## 7.1 Doxygen comment template

Every ported function must carry a Doxygen header. The ``@see`` tag
cross-references the Groves textbook equation — this mirrors the style
already present in the MATLAB block comments.

```cpp
/**
 * @brief  Meridian (R_N) and transverse (R_E) radii of curvature.
 *
 * Implements the WGS-84 radii formulas used throughout the navigation
 * equations. Called at every INS epoch in the NED frame simulation.
 *
 * @param[in]  L  Geodetic latitude (rad)
 * @return  std::pair<double,double>{R_N, R_E} in metres
 *
 * @note  At the equator: R_N ≈ 6,335,439 m, R_E = 6,378,137 m.
 *        At the poles:   R_N ≈ 6,399,594 m, R_E ≈ 6,399,594 m.
 *
 * @see Groves (2013) Eq. 2.105
 * @author Paul D. Groves (original MATLAB, 2012)
 * @author [Your name] (C++ port, [date])
 * @copyright BSD License
 */
```

## 7.2 Doxyfile (key settings)

```ini
PROJECT_NAME           = "GNSS/INS Navigation — C++ Port"
PROJECT_NUMBER         = 2.0-cpp
INPUT                  = include/ src/ apps/
RECURSIVE              = YES
GENERATE_HTML          = YES
GENERATE_LATEX         = NO
EXTRACT_ALL            = YES
HAVE_DOT               = YES
CALL_GRAPH             = YES
CALLER_GRAPH           = YES
WARN_NO_PARAMDOC       = YES
QUIET                  = NO
```

## 7.3 GitHub Actions CI

```yaml
# .github/workflows/ci.yml
name: Build & Test
on: [push, pull_request]
jobs:
  build:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v4
      - name: Install Eigen3
        run: sudo apt-get install -y libeigen3-dev libgtest-dev
      - name: Configure
        run: cmake -B build -DCMAKE_BUILD_TYPE=Release
      - name: Build
        run: cmake --build build -j4
      - name: Test
        run: ctest --test-dir build --output-on-failure
      - name: Doxygen
        run: |
          sudo apt-get install -y doxygen graphviz
          doxygen Doxyfile
```

## 7.4 Profile utility functions (low priority)

These four functions are used to pre-process or repair motion profile
CSV files. Port them last — they are not needed for any navigation
simulation to run.

| MATLAB | C++ | Purpose |
|---|---|---|
| ``Interpolate_profile.m`` | ``profile::interpolate()`` | Halve timestep by linear interpolation |
| ``Smooth_profile_velocity.m`` | ``profile::smoothVelocity()`` | Remove numerical jitter from velocity |
| ``Adjust_profile_position.m`` | ``profile::adjustPosition()`` | Re-derive position from velocity |
| ``Adjust_profile_velocity.m`` | ``profile::adjustVelocity()`` | Re-derive velocity from position |

---

## Step 7 checklist

- [ ] Doxyfile generated and ``doxygen Doxyfile`` builds without errors
- [ ] All headers have ``@brief``, ``@param``, ``@return``, ``@see Groves Eq.``
- [ ] ``apps/gnss_demo.cpp`` accepts ``--profile`` and ``--config`` CLI flags
- [ ] ``apps/inertial_demo.cpp`` accepts ``--profile``, ``--imu-grade`` flags
- [ ] ``apps/ins_gnss_demo.cpp`` accepts ``--mode {lc|tc}`` flag
- [ ] ``README.md`` — quick-start, build, dependency, demo run instructions
- [ ] GitHub Actions CI green on Ubuntu 22.04
- [ ] Profile utility functions ported (optional)

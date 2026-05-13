# Appendices

The project includes 11 PDF appendices from Groves (2013). Every C++ ``@see``
comment should reference the relevant appendix or equation number.

| File | Appendix | C++ modules |
|---|---|---|
| ``Appendix_A.pdf`` | Coordinate Frames & Transformations | ``frameconv/*``, ``attitude/*`` |
| ``Appendix_B.pdf`` | Quaternions (supplementary attitude) | ``attitude/*`` (optional future extension) |
| ``Appendix_C.pdf`` | Earth Model — WGS-84 | ``earth/gravity.hpp``, ``earth/radii.hpp`` |
| ``Appendix_D.pdf`` | Inertial Navigation Equations | ``nav/nav_equations.hpp`` |
| ``Appendix_E.pdf`` | Error Analysis — INS propagation | ``sim/inertial_nav.hpp`` |
| ``Appendix_F.pdf`` | GNSS Measurement Model | ``gnss/measurements.hpp``, ``gnss/satellite.hpp`` |
| ``Appendix_G.pdf`` | Kalman Filter Theory | ``kf/gnss_kf_epoch.hpp``, ``kf/lc_kf_epoch.hpp``, ``kf/tc_kf_epoch.hpp`` |
| ``Appendix_H.pdf`` | Loosely Coupled Integration | ``kf/lc_kf_epoch.hpp``, ``sim/loosely_coupled.hpp`` |
| ``Appendix_I.pdf`` | Tightly Coupled Integration | ``kf/tc_kf_epoch.hpp``, ``sim/tightly_coupled.hpp`` |
| ``Appendix_J.pdf`` | Simulation of Navigation Systems | All ``sim/*`` modules |
| ``Appendix_K.pdf`` | Historical Navigation Systems | Documentation reference only |

## Known simplifications in the original MATLAB code

The following simplifications are **intentional** — they must be preserved
in the C++ port, not treated as bugs:

- IMU and GNSS error sources are modelled as constants or white noise (no
  time-correlated errors)
- No GNSS innovation filtering in the EKF update step
- GNSS constellation uses circular orbits, not true Keplerian elements
- Satellite clock and atmospheric errors are treated as constant throughout
  each simulation run (appropriate for simulations under ~30 min)
- No GNSS receiver signal tracking simulation — receiver errors are modelled
  as additive white noise
- No simulation of signal blockage, multipath, jamming, or atmospheric
  diffraction

## Not ported

- ``Plot_profile.m`` and ``Plot_errors.m`` — visualization is out of scope;
  the C++ apps write CSV output for post-processing with Python/matplotlib
- The 31 ``.m`` demo scripts — replaced by 3 CLI apps in ``apps/``

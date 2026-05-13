# Overview

This documentation covers the complete migration of **78 MATLAB functions** from
Paul D. Groves' *Principles of GNSS, Inertial, and Multisensor Integrated
Navigation Systems* (2nd ed.) into C++17.

## What the codebase simulates

The original MATLAB suite covers three families of navigation system:

| System | Frames | Key files |
|---|---|---|
| **Stand-alone INS** | ECI, ECEF, NED | `Inertial_navigation_{NED,ECEF,ECI}.m` |
| **Stand-alone GNSS** | ECEF | `GNSS_Least_Squares.m`, `GNSS_Kalman_Filter.m` |
| **Coupled INS/GNSS** | ECEF | `Loosely_coupled_INS_GNSS.m`, `Tightly_coupled_INS_GNSS.m` |

## Source file inventory

| Category | Count | C++ target |
|---|---|---|
| Demonstration scripts | 31 | Replaced by CLI `apps/` |
| Master simulation functions | 7 | `include/gnss_ins/sim/` |
| General navigation functions | 28 | `include/gnss_ins/{nav,kf,gnss,earth,frameconv}/` |
| Tool / I/O functions | 9 | `include/gnss_ins/io/` |
| Motion profiles (CSV) | 5 | `data/Profile_N.csv` |

## Design goals for the C++ port

- **Header-only** or lightly-separated `hpp/cpp` layout — easy to drop into any project
- **Eigen3** for all matrix/vector maths — drop-in replacement for MATLAB `[]` operators
- **Zero dynamic allocation** in the filter hot path
- **Doxygen-compatible** `/** */` blocks on every function, with `@see` cross-references to Groves equation numbers
- **Unit-testable** modules — each MATLAB function maps to one C++ function or class method
- **CMake** build system with **CTest** unit tests
- **CSV profile reader** compatible with the existing `Profile_N.csv` format

## Estimation

| Item | Figure |
|---|---|
| Total MATLAB lines | ~5 000 |
| Estimated C++ lines | ~12 000 |
| Number of unit tests | ≥ 30 |
| Total effort (1 developer) | **27–36 working days** |

```{note}
Original software license: **BSD** — Copyright 2012, Paul D. Groves.
The C++ port must retain the original copyright notice and BSD license in all derived files.
```
